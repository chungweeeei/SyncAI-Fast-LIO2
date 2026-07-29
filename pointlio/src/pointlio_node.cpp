#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "map_builder/commons.h"
#include "map_builder/map_builder.h"
#include "tf2_ros/transform_broadcaster.h"
#include "utils.h"

using namespace std::chrono_literals;
// Input topics are NOT parameters: the node subscribes to the relative names
// "lidar" and "imu" and the launch file remaps them onto the driver's
// /<robot_id>/livox/{lidar,imu} (a relative name alone cannot reach them —
// the node runs in the /<robot_id>/pointlio namespace, so "lidar" would resolve
// to /<robot_id>/pointlio/lidar). Remapping is the ROS-native knob for this and
// keeps `ros2 node info` honest about where the data comes from.
struct NodeConfig
{
  std::string body_frame = "body";
  std::string world_frame = "lidar";
  bool print_time_cost = false;
  int lidar_type = 0;
  // 0: livox_ros_driver2 CustomMsg, 1: sensor_msgs/PointCloud2 (e.g. Isaac Sim)
  // Scale applied to linear_acceleration. Livox IMU reports g-units -> ~10.0 to get m/s^2.
  // Sensors already in m/s^2 (e.g. Isaac Sim, z~9.81 at rest) must use 1.0, since the
  // filter's gravity magnitude is fixed at 9.81 (point_ekf.cpp). satu_acc is compared
  // against the scaled value.
  double imu_acc_scale = 10.0;
};
struct StateData
{
  bool lidar_pushed = false;

  std::mutex imu_mutex;
  std::mutex lidar_mutex;

  double last_lidar_time = -1.0;
  double last_imu_time = -1.0;

  std::deque<IMUData> imu_buffer;
  std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;

  nav_msgs::msg::Path path;
};

class PointLIONode : public rclcpp::Node
{
public:
  PointLIONode() : Node("pointlio_node")
  {
    RCLCPP_INFO(this->get_logger(), "[PointLIONode][%s] Point-LIO Node Started", __func__);
    loadParameters();

    // register imu subscriber
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&PointLIONode::imuCB, this, std::placeholders::_1));

    // register scan subscriber. Log the resolved name so a missing/incorrect
    // remapping is visible at startup instead of showing up as "no data".
    if (m_node_config.lidar_type == 1) {
      m_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar", 10, std::bind(&PointLIONode::pc2CB, this, std::placeholders::_1));
      RCLCPP_INFO(
        this->get_logger(), "[PointLIONode][%s] Lidar input: sensor_msgs/PointCloud2 (%s)",
        __func__, m_pc2_sub->get_topic_name());
    } else {
      m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "lidar", 10, std::bind(&PointLIONode::lidarCB, this, std::placeholders::_1));
      RCLCPP_INFO(
        this->get_logger(), "[PointLIONode][%s] Lidar input: livox_ros_driver2/CustomMsg (%s)",
        __func__, m_lidar_sub->get_topic_name());
    }
    RCLCPP_INFO(
      this->get_logger(), "[PointLIONode][%s] IMU input: %s", __func__,
      m_imu_sub->get_topic_name());

    // register publisher
    m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
    m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
    m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);

    // register transform broadcaster
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // clear state data
    m_state_data.path.poses.clear();
    m_state_data.path.header.frame_id = m_node_config.world_frame;

    // initialize EKF & Map builder
    m_kf = std::make_shared<PointEKF>();
    m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);

    // register timer
    m_timer = this->create_wall_timer(20ms, std::bind(&PointLIONode::timerCB, this));
  }

  // Everything is a declared ROS parameter, fed by config/pointlio.yaml through
  // the launch file's `parameters=[...]`. The node used to take a single
  // `config_path` parameter and parse that YAML itself with yaml-cpp, which put
  // it outside every ROS tool: `ros2 param list/get/dump` showed only
  // config_path, a launch-level override could not touch a single value, and
  // the launch file had to rewrite the whole YAML into a /tmp file just to
  // inject the robot_id prefix. Declaring them properly is what lets the launch
  // layer the two robot_id-dependent frame names on top of the shared file.
  //
  // The defaults below deliberately repeat the struct defaults (NodeConfig here,
  // Config in map_builder/commons.h) so a missing key degrades exactly the way
  // running the node without any params file does, instead of throwing.
  void loadParameters()
  {
    // Frame names. TF frame ids are NOT namespaced by ROS, so the launch file
    // overrides both with the <robot_id>/ prefix; these are only the fallbacks
    // for running pointlio_node bare.
    m_node_config.body_frame = this->declare_parameter("body_frame", m_node_config.body_frame);
    m_node_config.world_frame = this->declare_parameter("world_frame", m_node_config.world_frame);
    m_node_config.print_time_cost =
      this->declare_parameter("print_time_cost", m_node_config.print_time_cost);
    m_node_config.lidar_type = this->declare_parameter("lidar_type", m_node_config.lidar_type);
    m_node_config.imu_acc_scale =
      this->declare_parameter("imu_acc_scale", m_node_config.imu_acc_scale);

    // config for map builder
    m_builder_config.lidar_filter_num =
      this->declare_parameter("lidar_filter_num", m_builder_config.lidar_filter_num);
    m_builder_config.lidar_min_range =
      this->declare_parameter("lidar_min_range", m_builder_config.lidar_min_range);
    m_builder_config.lidar_max_range =
      this->declare_parameter("lidar_max_range", m_builder_config.lidar_max_range);
    m_builder_config.scan_resolution =
      this->declare_parameter("scan_resolution", m_builder_config.scan_resolution);
    m_builder_config.map_resolution =
      this->declare_parameter("map_resolution", m_builder_config.map_resolution);
    m_builder_config.cube_len = this->declare_parameter("cube_len", m_builder_config.cube_len);
    m_builder_config.det_range = this->declare_parameter("det_range", m_builder_config.det_range);
    m_builder_config.move_thresh =
      this->declare_parameter("move_thresh", m_builder_config.move_thresh);

    // Point-LIO output model
    m_builder_config.gyr_cov_output =
      this->declare_parameter("gyr_cov_output", m_builder_config.gyr_cov_output);
    m_builder_config.acc_cov_output =
      this->declare_parameter("acc_cov_output", m_builder_config.acc_cov_output);
    m_builder_config.b_gyr_cov = this->declare_parameter("b_gyr_cov", m_builder_config.b_gyr_cov);
    m_builder_config.b_acc_cov = this->declare_parameter("b_acc_cov", m_builder_config.b_acc_cov);
    m_builder_config.imu_meas_omg_cov =
      this->declare_parameter("imu_meas_omg_cov", m_builder_config.imu_meas_omg_cov);
    m_builder_config.imu_meas_acc_cov =
      this->declare_parameter("imu_meas_acc_cov", m_builder_config.imu_meas_acc_cov);
    m_builder_config.satu_gyro = this->declare_parameter("satu_gyro", m_builder_config.satu_gyro);
    m_builder_config.satu_acc = this->declare_parameter("satu_acc", m_builder_config.satu_acc);
    m_builder_config.lidar_meas_cov =
      this->declare_parameter("lidar_meas_cov", m_builder_config.lidar_meas_cov);
    m_builder_config.plane_thr = this->declare_parameter("plane_thr", m_builder_config.plane_thr);
    m_builder_config.batch_max_points =
      this->declare_parameter("batch_max_points", m_builder_config.batch_max_points);

    m_builder_config.imu_init_num =
      this->declare_parameter("imu_init_num", m_builder_config.imu_init_num);
    m_builder_config.near_search_num =
      this->declare_parameter("near_search_num", m_builder_config.near_search_num);
    m_builder_config.gravity_align =
      this->declare_parameter("gravity_align", m_builder_config.gravity_align);
    m_builder_config.esti_il = this->declare_parameter("esti_il", m_builder_config.esti_il);

    // IMU <- lidar extrinsic. Flat double arrays because ROS parameters have no
    // nested/matrix type; the size checks matter because the old yaml-cpp path
    // indexed r_il_vec[8] with no validation, i.e. a short list in the config
    // was undefined behaviour rather than an error message.
    std::vector<double> t_il_vec = this->declare_parameter<std::vector<double>>(
      "t_il", {m_builder_config.t_il.x(), m_builder_config.t_il.y(), m_builder_config.t_il.z()});
    std::vector<double> r_il_vec = this->declare_parameter<std::vector<double>>(
      "r_il", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    if (t_il_vec.size() == 3) {
      m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "[PointLIONode][%s] t_il needs 3 elements, got %zu; keeping default",
        __func__, t_il_vec.size());
    }
    if (r_il_vec.size() == 9) {
      m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4],
        r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "[PointLIONode][%s] r_il needs 9 elements (row-major 3x3), got %zu; keeping identity",
        __func__, r_il_vec.size());
    }
  }

  void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // only use to store imu data into data structure
    std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
    double timestamp = Utils::getSec(msg->header);
    if (timestamp < m_state_data.last_imu_time) {
      RCLCPP_WARN(this->get_logger(), "[PointLIONode][%s] IMU Message is out of order", __func__);
      std::deque<IMUData>().swap(m_state_data.imu_buffer);
    }

    // push latest imu data into imu_buffer
    m_state_data.imu_buffer.emplace_back(
      V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) *
        m_node_config.imu_acc_scale,
      V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), timestamp);
    m_state_data.last_imu_time = timestamp;
  }

  void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    // only use to store lidar data into data structure
    CloudType::Ptr cloud = Utils::livox2PCL(
      msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range,
      m_builder_config.lidar_max_range);
    std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
    double timestamp = Utils::getSec(msg->header);
    if (timestamp < m_state_data.last_lidar_time) {
      RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
      std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(
        m_state_data.lidar_buffer);
    }
    m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
    m_state_data.last_lidar_time = timestamp;
  }

  void pc2CB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // CloudType => pcl::PointCloud<pcl::PointXYZINormal>
    CloudType::Ptr cloud = Utils::pc2ToPCL(
      msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range,
      m_builder_config.lidar_max_range);
    // PointCloud2 carries no per-point time (curvature = 0): every point in the
    // scan shares one timestamp, so the point-by-point update degenerates into a
    // single batch update per scan (no distortion compensation).
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "[PointLIONode][%s] PointCloud2 input has no per-point time; Point-LIO degrades to "
      "one batch update per scan",
      __func__);
    std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
    double timestamp = Utils::getSec(msg->header);
    if (timestamp < m_state_data.last_lidar_time) {
      RCLCPP_WARN(this->get_logger(), "[PointLIONode][%s] Lidar Message is out of order", __func__);
      std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(
        m_state_data.lidar_buffer);
    }
    m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
    m_state_data.last_lidar_time = timestamp;
  }

  bool syncPackage()
  {
    if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty()) return false;

    // 一幀 lidar 並不是一收到就處理，必須等到 IMU 「補齊」到掃描結束時刻才行。
    if (!m_state_data.lidar_pushed) {
      m_package.cloud = m_state_data.lidar_buffer.front().second;

      // pointcloud 中的 curvature 欄位被用來存放每一點的時間搓, 排序目的就是讓 pointcloud 最後一個點是最晚的點否則就只是點雲陣列的最後一個元素與時間無關
      std::sort(
        m_package.cloud->points.begin(), m_package.cloud->points.end(),
        [](PointType & p1, PointType & p2) { return p1.curvature < p2.curvature; });

      m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
      m_package.cloud_end_time =
        m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
      m_state_data.lidar_pushed = true;
    }

    // 如果 IMU 資料還無法涵蓋一幀的 pointcloud 就先等待 (point-by-point 更新需要
    // 整幀期間的 IMU 事件)
    if (m_state_data.last_imu_time < m_package.cloud_end_time) return false;

    // 清空 + 釋放 m_package 中的 imu 資料
    Vec<IMUData>().swap(m_package.imus);

    // 將該幀需要用的 IMU 資料撈出來並把 imu_buffer 清乾淨
    while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time) {
      // 把需要用到的 imu 資料塞回 m_package.
      m_package.imus.emplace_back(m_state_data.imu_buffer.front());
      m_state_data.imu_buffer.pop_front();
    }

    // 清除用到的那幀 lidar 資料
    m_state_data.lidar_buffer.pop_front();
    m_state_data.lidar_pushed = false;
    return true;
  }

  void publishCloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud,
    std::string frame_id, const double & time)
  {
    if (pub->get_subscription_count() <= 0) return;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = Utils::getTime(time);
    pub->publish(cloud_msg);
  }

  void publishOdometry(
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id,
    std::string child_frame, const double & time)
  {
    if (odom_pub->get_subscription_count() <= 0) return;
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = frame_id;
    odom.header.stamp = Utils::getTime(time);
    odom.child_frame_id = child_frame;
    odom.pose.pose.position.x = m_kf->x().t_wi.x();
    odom.pose.pose.position.y = m_kf->x().t_wi.y();
    odom.pose.pose.position.z = m_kf->x().t_wi.z();
    Eigen::Quaterniond q(m_kf->x().r_wi);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();

    // output model 直接估測角速度, 一併輸出
    odom.twist.twist.angular.x = m_kf->x().omg.x();
    odom.twist.twist.angular.y = m_kf->x().omg.y();
    odom.twist.twist.angular.z = m_kf->x().omg.z();
    odom_pub->publish(odom);
  }

  void publishPath(
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id,
    const double & time)
  {
    if (path_pub->get_subscription_count() <= 0) return;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = Utils::getTime(time);
    pose.pose.position.x = m_kf->x().t_wi.x();
    pose.pose.position.y = m_kf->x().t_wi.y();
    pose.pose.position.z = m_kf->x().t_wi.z();
    Eigen::Quaterniond q(m_kf->x().r_wi);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    m_state_data.path.poses.push_back(pose);
    path_pub->publish(m_state_data.path);
  }

  void broadCastTF(
    std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id,
    std::string child_frame, const double & time)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame;
    transformStamped.header.stamp = Utils::getTime(time);

    // 下標 wi 讀作 w ← i，右到左：「從 i 到 w」。跟 r_il 是 i ← l（lidar → IMU）
    // 完整的鏈：lidar 系 ──T_il──> IMU 系 ──T_wi──> 世界系
    Eigen::Quaterniond q(m_kf->x().r_wi);
    V3D t = m_kf->x().t_wi;

    transformStamped.transform.translation.x = t.x();
    transformStamped.transform.translation.y = t.y();
    transformStamped.transform.translation.z = t.z();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    broad_caster->sendTransform(transformStamped);
  }

  void timerCB()
  {
    if (!syncPackage()) return;

    auto t1 = std::chrono::high_resolution_clock::now();
    m_builder->process(m_package);
    auto t2 = std::chrono::high_resolution_clock::now();

    if (m_node_config.print_time_cost) {
      auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
      RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
    }

    if (m_builder->status() != BuilderStatus::MAPPING) return;

    // world frame -> odom3D
    // body_frame -> laser3D
    broadCastTF(
      m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame,
      m_package.cloud_end_time);

    publishOdometry(
      m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

    CloudType::Ptr body_cloud =
      m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
    publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

    // 注意: world_cloud 是整幀用「掃描結束時刻」的 pose 轉換的, 僅供顯示;
    // 進地圖的點是 point-by-point 在各自時間點轉換的 (lidar_processor)。
    CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(
      m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());
    publishCloud(
      m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

    // odom path
    publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
  }

private:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc2_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

  rclcpp::TimerBase::SharedPtr m_timer;
  StateData m_state_data;
  SyncPackage m_package;
  NodeConfig m_node_config;
  Config m_builder_config;
  std::shared_ptr<PointEKF> m_kf;
  std::shared_ptr<MapBuilder> m_builder;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointLIONode>());
  rclcpp::shutdown();
  return 0;
}
