#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <filesystem>
#include <mutex>
#include <queue>

#include "interface/srv/is_valid.hpp"
#include "interface/srv/relocalize.hpp"
#include "localizers/commons.h"
#include "localizers/icp_localizer.h"

using namespace std::chrono_literals;

struct NodeConfig
{
  std::string cloud_topic = "/fastlio2/body_cloud";
  std::string odom_topic = "/fastlio2/lio_odom";
  std::string map_frame = "map";
  std::string local_frame = "lidar";
  // system.ini 的 [map] pcd，由 launch 以 parameter override 傳進來。initialpose 在地圖
  // 尚未載入時（localizer 重啟後還沒跑過 relocalize）用它自動 loadMap；
  // 空字串表示沒設定，此時 initialpose 只能在 relocalize 之後使用
  std::string map_path = "";
  // system.ini 的 [initial_pose]：機器人開機時的已知起點（map frame）。設了以後
  // 收到第一筆 odom 就自動套用一次當 ICP 的 initial guess，等同於自動跑一次
  // relocalize，不必手動呼叫 service。
  // 只有 x / y / yaw：z / roll / pitch 一律取自當下估計，理由見 applyPlanarGuess。
  bool set_initial_pose = false;
  double initial_pose_x = 0.0;
  double initial_pose_y = 0.0;
  double initial_pose_yaw = 0.0;
  double update_hz = 1.0;
};

struct NodeState
{
  std::mutex message_mutex;
  std::mutex service_mutex;

  bool message_received = false;
  bool service_received = false;
  bool localize_success = false;
  // config 的起點還沒套用。只有 timer 那條 thread 會碰它（建構期設定，spin
  // 開始後只在 timerCB 讀寫），所以不需要鎖
  bool initial_pose_pending = false;
  rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
  builtin_interfaces::msg::Time last_message_time;
  CloudType::Ptr last_cloud = std::make_shared<CloudType>();
  M3D last_r;                           // localmap_body_r
  V3D last_t;                           // localmap_body_t
  M3D last_offset_r = M3D::Identity();  // map_localmap_r
  V3D last_offset_t = V3D::Zero();      // map_localmap_t
  M4F initial_guess = M4F::Identity();
};

class LocalizerNode : public rclcpp::Node
{
public:
  LocalizerNode() : Node("localizer_node")
  {
    RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
    loadParameters();

    // message filter subscribe body pointcloud & lio_odom topic
    rclcpp::QoS qos = rclcpp::QoS(10);
    m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
    m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

    // transform broadcaster
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    m_sync = std::make_shared<
      message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
      m_cloud_sub, m_odom_sub);
    m_sync->setAgePenalty(0.1);
    m_sync->registerCallback(
      std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));

    // localizer
    m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

    // services 放在獨立的 callback group（搭配 main 的 MultiThreadedExecutor），
    // relocCB 裡耗時的 loadMap 才不會卡住 timer/subscriber 那組的 TF 重播
    m_srv_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_reloc_srv = this->create_service<interface::srv::Relocalize>(
      "relocalize",
      std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, m_srv_cb_group);

    m_reloc_check_srv = this->create_service<interface::srv::IsValid>(
      "relocalize_check",
      std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, m_srv_cb_group);

    // RViz「2D Pose Estimate」等來源的 initial guess：走 relocalize 同一條
    // initial_guess 路徑（下一輪 timerCB 拿去餵 ICP）。地圖正常情況下已在建構
    // 期載好（loadInitialMap），callback 裡只剩補救用的 loadMap 分支。
    // 那個分支可能耗時數秒，所以跟 services 放同一個 callback group，
    // 才不會卡住 timer 那組的 TF 重播
    rclcpp::SubscriptionOptions initialpose_options;
    initialpose_options.callback_group = m_srv_cb_group;
    m_initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10, std::bind(&LocalizerNode::initialPoseCB, this, std::placeholders::_1),
      initialpose_options);

    // 地圖是靜態的：transient_local（latched）+ loadMap 成功後發一次，
    // 晚連上的訂閱者也收得到（RViz 端的 QoS 也要設成 transient_local）
    m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_cloud", rclcpp::QoS(1).transient_local());

    // 地圖在建構期就載好。以前只有 relocalize 才 loadMap，重啟後先發
    // initialpose 會撞上空 target（align() 直接 return false）；每個入口各自
    // 補一次 loadMap 也讓「地圖是否已載入」變成隱性狀態。現在 launch 保證
    // map_path 存在（缺檔就不啟動 node），這裡一次載完，relocalize /
    // initialpose 只需要處理 guess。
    // 必須放在 m_map_cloud_pub 之後（載完要 latched 發一次），而 spin 還沒
    // 開始，同步載入數秒不會卡到任何 callback。
    loadInitialMap();

    // 起點只能延後到第一筆 odom 之後才套用（applyPlanarGuess 要拿當下估計補
    // roll/pitch/z），所以建構期只立旗標，實際套用在 timerCB
    m_state.initial_pose_pending = m_config.set_initial_pose;
    if (m_config.set_initial_pose) {
      RCLCPP_INFO(
        this->get_logger(),
        "initial pose from config: x=%.3f y=%.3f yaw=%.3f (applied on first odom)",
        m_config.initial_pose_x, m_config.initial_pose_y, m_config.initial_pose_yaw);
    }

    m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
  }

  void loadInitialMap()
  {
    if (m_config.map_path.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "map_path is empty: starting with no map — localization stays idle until "
        "the relocalize service loads one");
      return;
    }
    if (!std::filesystem::exists(m_config.map_path)) {
      RCLCPP_ERROR(
        this->get_logger(), "map_path '%s' does not exist: starting with no map",
        m_config.map_path.c_str());
      return;
    }
    if (!m_localizer->loadMap(m_config.map_path)) {
      RCLCPP_ERROR(
        this->get_logger(), "failed to load map from '%s': starting with no map",
        m_config.map_path.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "map preloaded from %s", m_config.map_path.c_str());
    builtin_interfaces::msg::Time stamp = this->now();
    publishMapCloud(stamp);
  }

  void loadParameters()
  {
    // 走標準 ROS 2 parameters（config/localizer.yaml 是 /**/localizer_node 的
    // params 檔，launch 再疊上 robot_id 前綴的 topic / map_path 覆寫）。
    // 舊版是自己用 yaml-cpp 讀 config_path 指到的扁平 YAML：ros2 param 只看得到
    // config_path、不能用 --params-file，而且 config_path 沒帶時 YAML::LoadFile
    // 會丟例外、必填 key 少一個就 InvalidNode，node 直接在建構期掛掉。
    // 現在每個參數都以 struct 的預設值當 default，缺項只是沿用預設。
    m_config.cloud_topic = declare_parameter<std::string>("cloud_topic", m_config.cloud_topic);
    m_config.odom_topic = declare_parameter<std::string>("odom_topic", m_config.odom_topic);
    m_config.map_frame = declare_parameter<std::string>("map_frame", m_config.map_frame);
    m_config.local_frame = declare_parameter<std::string>("local_frame", m_config.local_frame);
    m_config.map_path = declare_parameter<std::string>("map_path", m_config.map_path);
    m_config.update_hz = declare_parameter<double>("update_hz", m_config.update_hz);

    // 巢狀名稱（initial_pose.x）跟 syncai_amcl 的 set_initial_pose /
    // initial_pose.* 對齊，兩邊都是由 launch 從 system.ini 的 [initial_pose]
    // 覆寫。這裡刻意不收 z：見 applyPlanarGuess
    m_config.set_initial_pose =
      declare_parameter<bool>("set_initial_pose", m_config.set_initial_pose);
    m_config.initial_pose_x = declare_parameter<double>("initial_pose.x", m_config.initial_pose_x);
    m_config.initial_pose_y = declare_parameter<double>("initial_pose.y", m_config.initial_pose_y);
    m_config.initial_pose_yaw =
      declare_parameter<double>("initial_pose.yaw", m_config.initial_pose_yaw);

    // small_gicp 後端的共用參數，兩段配準共享
    m_localizer_config.num_threads =
      declare_parameter<int>("num_threads", m_localizer_config.num_threads);
    m_localizer_config.num_neighbors =
      declare_parameter<int>("num_neighbors", m_localizer_config.num_neighbors);

    m_localizer_config.rough_scan_resolution =
      declare_parameter<double>("rough_scan_resolution", m_localizer_config.rough_scan_resolution);
    m_localizer_config.rough_map_resolution =
      declare_parameter<double>("rough_map_resolution", m_localizer_config.rough_map_resolution);
    m_localizer_config.rough_max_iteration =
      declare_parameter<int>("rough_max_iteration", m_localizer_config.rough_max_iteration);
    m_localizer_config.rough_score_thresh =
      declare_parameter<double>("rough_score_thresh", m_localizer_config.rough_score_thresh);
    m_localizer_config.rough_max_corr_dist =
      declare_parameter<double>("rough_max_corr_dist", m_localizer_config.rough_max_corr_dist);
    m_localizer_config.rough_registration_type = declare_parameter<std::string>(
      "rough_registration_type", m_localizer_config.rough_registration_type);
    m_localizer_config.rough_voxel_resolution = declare_parameter<double>(
      "rough_voxel_resolution", m_localizer_config.rough_voxel_resolution);

    m_localizer_config.refine_scan_resolution = declare_parameter<double>(
      "refine_scan_resolution", m_localizer_config.refine_scan_resolution);
    m_localizer_config.refine_map_resolution =
      declare_parameter<double>("refine_map_resolution", m_localizer_config.refine_map_resolution);
    m_localizer_config.refine_max_iteration =
      declare_parameter<int>("refine_max_iteration", m_localizer_config.refine_max_iteration);
    m_localizer_config.refine_score_thresh =
      declare_parameter<double>("refine_score_thresh", m_localizer_config.refine_score_thresh);
    m_localizer_config.refine_max_corr_dist =
      declare_parameter<double>("refine_max_corr_dist", m_localizer_config.refine_max_corr_dist);
    m_localizer_config.refine_registration_type = declare_parameter<std::string>(
      "refine_registration_type", m_localizer_config.refine_registration_type);
    m_localizer_config.refine_voxel_resolution = declare_parameter<double>(
      "refine_voxel_resolution", m_localizer_config.refine_voxel_resolution);

    RCLCPP_INFO(
      this->get_logger(), "params: cloud_topic=%s odom_topic=%s map_frame=%s update_hz=%.2f",
      m_config.cloud_topic.c_str(), m_config.odom_topic.c_str(), m_config.map_frame.c_str(),
      m_config.update_hz);
  }

  void timerCB()
  {
    if (!m_state.message_received) return;

    // config 的已知起點：第一筆 odom 到了才有「當下估計」可以補 roll/pitch/z，
    // 所以在這裡套用而不是建構期。只有一次機會——旗標先清掉，就算地圖沒載入
    // 而跳過，也不會留到之後某次 relocalize 載完圖才突然蓋掉那次的 guess。
    if (m_state.initial_pose_pending) {
      m_state.initial_pose_pending = false;
      if (m_localizer->isMapLoaded()) {
        applyPlanarGuess(
          m_config.initial_pose_x, m_config.initial_pose_y, m_config.initial_pose_yaw, "config");
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "initial pose from config skipped: no map loaded — use the relocalize service");
      }
    }

    rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;

    // 判斷距離上次發佈 TF 的時間，小於 1 / update_hz 就發佈上次的 TF
    bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

    if (!update_tf) {
      sendBroadCastTF(m_state.last_message_time);
      return;
    }

    m_state.last_send_tf_time = rclcpp::Clock().now();

    // 跑 rough + refine ICP，成攻就更新 offset
    M4F initial_guess = M4F::Identity();
    bool service_pending = false;
    {
      std::lock_guard<std::mutex> lock(m_state.service_mutex);
      service_pending = m_state.service_received;
      if (service_pending) initial_guess = m_state.initial_guess;
    }
    if (!service_pending) {
      std::lock_guard<std::mutex> lock(m_state.message_mutex);
      initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
      initial_guess.block<3, 1>(0, 3) =
        (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
    }

    M3D current_local_r;
    V3D current_local_t;
    builtin_interfaces::msg::Time current_time;
    {
      std::lock_guard<std::mutex> lock(m_state.message_mutex);
      current_local_r = m_state.last_r;
      current_local_t = m_state.last_t;
      current_time = m_state.last_message_time;
      m_localizer->setInput(m_state.last_cloud);
    }

    // 地圖只有在 relocCB 才會 loadMap, target pointcloud 是空的, align 會直接 return false
    bool result = m_localizer->align(initial_guess);
    if (result) {
      M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
      V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
      m_state.last_offset_r = map_body_r * current_local_r.transpose();
      // 重正交化：ICP 解與 odom 旋轉是 float 連乘（下一輪還會回饋進 initial_guess），
      // 誤差累積會讓 R 偏離 SO(3)，broadcast 時轉成 quaternion 的 norm 就會漂移
      // （tf2 denormalized-quaternion warning，且 skew/scale 被當成假位移）。拉回合法旋轉。
      Eigen::Quaterniond offset_q(m_state.last_offset_r);
      offset_q.normalize();
      m_state.last_offset_r = offset_q.toRotationMatrix();
      // 平移用同一個已正交化的 last_offset_r，保持旋轉/平移一致
      m_state.last_offset_t = -m_state.last_offset_r * current_local_t + map_body_t;
      std::lock_guard<std::mutex> lock(m_state.service_mutex);
      if (!m_state.localize_success && m_state.service_received) {
        m_state.localize_success = true;
        m_state.service_received = false;
      }
    }

    sendBroadCastTF(current_time);
  }

  void syncCB(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);

    pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

    m_state.last_r = Eigen::Quaterniond(
                       odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                       odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)
                       .toRotationMatrix();
    m_state.last_t = V3D(
      odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z);
    m_state.last_message_time = cloud_msg->header.stamp;
    if (!m_state.message_received) {
      m_state.message_received = true;
      m_config.local_frame = odom_msg->header.frame_id;
    }
  }

  void sendBroadCastTF(builtin_interfaces::msg::Time & time)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = m_config.map_frame;
    transformStamped.child_frame_id = m_config.local_frame;
    transformStamped.header.stamp = time;
    Eigen::Quaterniond q(m_state.last_offset_r);
    V3D t = m_state.last_offset_t;
    transformStamped.transform.translation.x = t.x();
    transformStamped.transform.translation.y = t.y();
    transformStamped.transform.translation.z = t.z();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    m_tf_broadcaster->sendTransform(transformStamped);
  }

  void relocCB(
    const std::shared_ptr<interface::srv::Relocalize::Request> request,
    std::shared_ptr<interface::srv::Relocalize::Response> response)
  {
    std::string pcd_path = request->pcd_path;
    float x = request->x;
    float y = request->y;
    float z = request->z;
    float yaw = request->yaw;
    float roll = request->roll;
    float pitch = request->pitch;

    if (!std::filesystem::exists(pcd_path)) {
      response->success = false;
      response->message = "pcd file not found";
      return;
    }

    Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    bool load_flag = m_localizer->loadMap(pcd_path);
    if (!load_flag) {
      response->success = false;
      response->message = "load map failed";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(m_state.service_mutex);
      m_state.initial_guess.setIdentity();
      m_state.initial_guess.block<3, 3>(0, 0) =
        (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
      m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
      m_state.service_received = true;
      m_state.localize_success = false;
    }

    // 地圖只在這裡才會改變，latched 發一次給 RViz 等訂閱者
    builtin_interfaces::msg::Time stamp = this->now();
    publishMapCloud(stamp);

    response->success = true;
    response->message = "relocalize success";
    return;
  }

  void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // 正常路徑上地圖已經在建構期載好（loadInitialMap），這段只在 map_path 沒設
    // 或當時載入失敗時才會走到。保留它是因為地圖沒載入時 align() 會在空 target
    // 上直接 return false（連 guess 都不會 log），initialpose 看起來就像完全沒
    // 反應——實機上就是這樣中招過。這裡要嘛補載成功、要嘛明確警告並忽略，
    // 不留下一個等地圖載入後才生效的過期 guess。
    if (!m_localizer->isMapLoaded()) {
      if (m_config.map_path.empty() || !std::filesystem::exists(m_config.map_path)) {
        RCLCPP_WARN(
          this->get_logger(),
          "initialpose ignored: map not loaded and map_path '%s' not usable — "
          "call the relocalize service first",
          m_config.map_path.c_str());
        return;
      }
      RCLCPP_INFO(
        this->get_logger(), "initialpose with no map loaded: loading map from %s",
        m_config.map_path.c_str());
      if (!m_localizer->loadMap(m_config.map_path)) {
        RCLCPP_WARN(
          this->get_logger(), "initialpose ignored: failed to load map from %s",
          m_config.map_path.c_str());
        return;
      }
      builtin_interfaces::msg::Time stamp = this->now();
      publishMapCloud(stamp);
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != m_config.map_frame) {
      RCLCPP_WARN(
        this->get_logger(), "initialpose frame_id '%s' != map_frame '%s', treating it as %s",
        msg->header.frame_id.c_str(), m_config.map_frame.c_str(), m_config.map_frame.c_str());
    }

    Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    q.normalize();
    M3D msg_r = q.toRotationMatrix();
    applyPlanarGuess(
      msg->pose.pose.position.x, msg->pose.pose.position.y,
      std::atan2(msg_r(1, 0), msg_r(0, 0)), "initialpose");
  }

  // 把 2D 起點（x, y, yaw）組成 ICP 的 initial guess。兩個來源共用：RViz /
  // 前端 gridmap 的 initialpose，以及 system.ini [initial_pose] 的開機起點。
  //
  // 2D 來源只給得出 x, y, yaw（z=0、roll=pitch=0）。但雷達斜裝 + map 重力對齊
  // 時，map_T_body 恆帶 ~15° 的 mount pitch：直接拿 2D 姿態當 guess 會讓 scan
  // 相對地圖整體傾斜，遠處點的誤差超過 rough_max_corr_dist，rough ICP 永遠過不
  // 了 score 門檻——而 pending 期間 timerCB 又跳過 odom 回饋路徑，定位就凍在舊
  // offset 上無限重試（實機在 (5.0, 0.982) 發 initialpose 卡死，補上 14.6°
  // pitch 後同一位置第一輪就收斂）。
  // 所以只取 x, y, yaw；roll/pitch/z 一律從目前估計補齊。
  void applyPlanarGuess(double x, double y, double yaw, const char * source)
  {
    M3D guess_r = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();
    V3D guess_t(x, y, 0.0);
    {
      std::lock_guard<std::mutex> lock(m_state.message_mutex);
      if (m_state.message_received) {
        M3D cur_r = m_state.last_offset_r * m_state.last_r;
        V3D cur_t = m_state.last_offset_r * m_state.last_t + m_state.last_offset_t;
        double cur_yaw = std::atan2(cur_r(1, 0), cur_r(0, 0));
        // 保留目前姿態的重力傾角，只把 heading 轉到指定的 yaw
        guess_r = Eigen::AngleAxisd(yaw - cur_yaw, V3D::UnitZ()).toRotationMatrix() * cur_r;
        guess_t.z() = cur_t.z();
      }
      // 開機初期還沒收到任何 odom 時沒有估計可補，就用純 2D 的姿態
      // （config 的起點是等到第一筆 odom 才套用，不會走到這裡）
    }

    std::lock_guard<std::mutex> lock(m_state.service_mutex);
    m_state.initial_guess.setIdentity();
    m_state.initial_guess.block<3, 3>(0, 0) = guess_r.cast<float>();
    m_state.initial_guess.block<3, 1>(0, 3) = guess_t.cast<float>();
    m_state.service_received = true;
    m_state.localize_success = false;
    RCLCPP_INFO(
      this->get_logger(),
      "guess from %s applied: x=%.3f y=%.3f z=%.3f yaw=%.3f (roll/pitch/z from current estimate)",
      source, guess_t.x(), guess_t.y(), guess_t.z(), yaw);
  }

  void relocCheckCB(
    const std::shared_ptr<interface::srv::IsValid::Request> request,
    std::shared_ptr<interface::srv::IsValid::Response> response)
  {
    std::lock_guard<std::mutex> lock(m_state.service_mutex);
    if (request->code == 1)
      response->valid = true;
    else
      response->valid = m_state.localize_success;
    return;
  }
  void publishMapCloud(builtin_interfaces::msg::Time & time)
  {
    // latched：即使當下沒有訂閱者也要發，DDS 會快取給晚連上的訂閱者
    CloudType::Ptr map_cloud = m_localizer->refineMap();
    if (map_cloud->size() < 1) return;
    sensor_msgs::msg::PointCloud2 map_cloud_msg;
    pcl::toROSMsg(*map_cloud, map_cloud_msg);
    map_cloud_msg.header.frame_id = m_config.map_frame;
    map_cloud_msg.header.stamp = time;
    m_map_cloud_pub->publish(map_cloud_msg);
  }

private:
  NodeConfig m_config;
  NodeState m_state;

  ICPConfig m_localizer_config;
  std::shared_ptr<ICPLocalizer> m_localizer;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  rclcpp::TimerBase::SharedPtr m_timer;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>
    m_sync;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  rclcpp::CallbackGroup::SharedPtr m_srv_cb_group;
  rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
  rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialpose_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizerNode>();
  // 兩條線程：timer/subscriber 一組、services 一組，
  // relocalize 的 loadMap 進行中 TF 重播不中斷
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
