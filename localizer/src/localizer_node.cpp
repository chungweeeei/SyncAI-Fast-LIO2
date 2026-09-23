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

#include <algorithm>
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
  // The [map] pcd from system.ini, passed in by the launch file as a parameter override.
  // initialpose uses it to loadMap automatically when no map is loaded yet (the localizer was
  // restarted and no relocalize has run since); an empty string means it is not set, in which
  // case initialpose can only be used after a relocalize.
  std::string map_path = "";
  // The [initial_pose] from system.ini: the robot's known start pose at boot (map frame). When
  // set, it is applied once as the ICP initial guess as soon as the first odom arrives, which is
  // equivalent to an automatic relocalize with no manual service call.
  // Only x / y / yaw: z / roll / pitch are always taken from the current estimate, for the
  // reasons given at applyPlanarGuess.
  bool set_initial_pose = false;
  double initial_pose_x = 0.0;
  double initial_pose_y = 0.0;
  double initial_pose_yaw = 0.0;
  double update_hz = 1.0;

  // Motion gate. update_hz is a *ceiling* on re-registration, not a schedule:
  // below these thresholds the odom pose has not moved enough to justify a new
  // scan match, so the previous offset is rebroadcast and GICP is skipped
  // entirely. For a parked robot the correct correction is a constant, and
  // re-solving it 5 times a second only resamples GICP's own noise floor
  // (measured on robot01 2026-09-21: 8.9 mm median / 24.9 mm max per update,
  // 193 of 241 updates > 5 mm, while Point-LIO itself moved 0.87 mm/frame).
  double min_update_trans = 0.05;
  double min_update_rot = 0.02;
  // Backstop: re-register this often even when parked, so a slow true drift is
  // still corrected. Kept short on purpose — the failure direction we want is
  // "jitter only partly suppressed", never "frozen on a stale pose".
  double max_update_interval = 2.0;
  // EMA weight applied to a backstop refresh (a re-registration that the motion
  // gate would otherwise have skipped). Motion-triggered updates always use 1.0,
  // so driving is bit-for-bit unchanged and picks up no lag. 1.0 here disables
  // blending and leaves only the gate.
  double static_blend_alpha = 0.1;
  // After a relocalize / initial-pose guess, register at full rate and full
  // alpha for this long. A relocalize returning success is a receipt, not a
  // result: the pose converges over the following rounds. Gating right after
  // the first successful align would leave a parked robot refining at one
  // alpha=0.1 step per max_update_interval, i.e. tens of seconds to converge.
  double post_reloc_settle = 3.0;
};

struct NodeState
{
  std::mutex message_mutex;
  std::mutex service_mutex;

  bool message_received = false;
  bool service_received = false;
  bool localize_success = false;
  // The start pose from config has not been applied yet. Only the timer thread touches it (set
  // during construction, then read and written only in timerCB once spin has started), so it
  // needs no lock.
  bool initial_pose_pending = false;
  rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
  builtin_interfaces::msg::Time last_message_time;
  CloudType::Ptr last_cloud = std::make_shared<CloudType>();
  M3D last_r;                           // localmap_body_r
  V3D last_t;                           // localmap_body_t
  M3D last_offset_r = M3D::Identity();  // map_localmap_r
  V3D last_offset_t = V3D::Zero();      // map_localmap_t
  M4F initial_guess = M4F::Identity();

  // Odom pose at the last *accepted* registration — the reference the motion
  // gate measures against. Only the timer thread touches these.
  M3D last_reg_r = M3D::Identity();
  V3D last_reg_t = V3D::Zero();
  bool has_reg = false;
  rclcpp::Time last_reg_time{0, 0, RCL_SYSTEM_TIME};
  // Full-rate window after a relocalize; see NodeConfig::post_reloc_settle.
  rclcpp::Time settle_until{0, 0, RCL_SYSTEM_TIME};
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

    // The services live in their own callback group (paired with the MultiThreadedExecutor in
    // main), so the slow loadMap inside relocCB cannot stall the TF rebroadcast running in the
    // timer/subscriber group.
    m_srv_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_reloc_srv = this->create_service<interface::srv::Relocalize>(
      "relocalize",
      std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, m_srv_cb_group);

    m_reloc_check_srv = this->create_service<interface::srv::IsValid>(
      "relocalize_check",
      std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, m_srv_cb_group);

    // Initial guesses from sources such as RViz's "2D Pose Estimate": they take the same
    // initial_guess path as relocalize (the next timerCB round feeds it to ICP). Normally the map
    // is already loaded during construction (loadInitialMap), so all that is left in the callback
    // is the fallback loadMap branch. That branch can take several seconds, which is why it shares
    // the services' callback group — so it cannot stall the TF rebroadcast in the timer group.
    rclcpp::SubscriptionOptions initialpose_options;
    initialpose_options.callback_group = m_srv_cb_group;
    m_initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10, std::bind(&LocalizerNode::initialPoseCB, this, std::placeholders::_1),
      initialpose_options);

    // The map is static: transient_local (latched) plus one publish after a successful loadMap, so
    // late-joining subscribers still receive it (the RViz side must set its QoS to transient_local
    // too).
    m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_cloud", rclcpp::QoS(1).transient_local());

    // Load the map during construction. Previously only relocalize called loadMap, so an
    // initialpose sent first after a restart hit an empty target (align() returned false
    // outright); and having every entry point do its own catch-up loadMap turned "is the map
    // loaded" into hidden state. The launch file now guarantees map_path exists (the node is not
    // started when the file is missing), so the map is loaded once here and relocalize /
    // initialpose only have to deal with the guess.
    // This must come after m_map_cloud_pub (the loaded map is published once, latched), and spin
    // has not started yet, so a synchronous load of a few seconds blocks no callback.
    loadInitialMap();

    // The start pose can only be applied after the first odom (applyPlanarGuess needs the current
    // estimate to fill in roll/pitch/z), so the constructor only raises the flag and the actual
    // application happens in timerCB.
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
    // Standard ROS 2 parameters (config/localizer.yaml is the /**/localizer_node params file; the
    // launch file layers the robot_id-prefixed topic / map_path overrides on top). The old version
    // parsed a flat YAML at config_path with yaml-cpp itself: `ros2 param` could only see
    // config_path, --params-file could not be used, and when config_path was not given
    // YAML::LoadFile threw, while a single missing required key raised InvalidNode — the node
    // died during construction either way. Now every parameter takes the struct's value as its
    // default, so a missing entry simply keeps that default.
    m_config.cloud_topic = declare_parameter<std::string>("cloud_topic", m_config.cloud_topic);
    m_config.odom_topic = declare_parameter<std::string>("odom_topic", m_config.odom_topic);
    m_config.map_frame = declare_parameter<std::string>("map_frame", m_config.map_frame);
    m_config.local_frame = declare_parameter<std::string>("local_frame", m_config.local_frame);
    m_config.map_path = declare_parameter<std::string>("map_path", m_config.map_path);
    m_config.update_hz = declare_parameter<double>("update_hz", m_config.update_hz);

    // Motion gate; see NodeConfig for what each one buys.
    m_config.min_update_trans =
      declare_parameter<double>("min_update_trans", m_config.min_update_trans);
    m_config.min_update_rot = declare_parameter<double>("min_update_rot", m_config.min_update_rot);
    m_config.max_update_interval =
      declare_parameter<double>("max_update_interval", m_config.max_update_interval);
    m_config.static_blend_alpha =
      declare_parameter<double>("static_blend_alpha", m_config.static_blend_alpha);
    m_config.post_reloc_settle =
      declare_parameter<double>("post_reloc_settle", m_config.post_reloc_settle);

    // The nested names (initial_pose.x) line up with syncai_amcl's set_initial_pose /
    // initial_pose.*; both are overridden by the launch file from the [initial_pose] section of
    // system.ini. z is deliberately not accepted here: see applyPlanarGuess.
    m_config.set_initial_pose =
      declare_parameter<bool>("set_initial_pose", m_config.set_initial_pose);
    m_config.initial_pose_x = declare_parameter<double>("initial_pose.x", m_config.initial_pose_x);
    m_config.initial_pose_y = declare_parameter<double>("initial_pose.y", m_config.initial_pose_y);
    m_config.initial_pose_yaw =
      declare_parameter<double>("initial_pose.yaw", m_config.initial_pose_yaw);

    // Common parameters of the small_gicp backend, shared by both registration stages.
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

    // The known start pose from config: only once the first odom has arrived is there a "current
    // estimate" to fill in roll/pitch/z, so it is applied here rather than in the constructor. It
    // gets exactly one chance — the flag is cleared first, so even if it is skipped because no
    // map is loaded, it does not linger until some later relocalize finishes loading a map and
    // then suddenly overwrite that call's guess.
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

    // Check the time since the last TF publish; if it is under 1 / update_hz, republish the
    // previous TF.
    bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

    if (!update_tf) {
      sendBroadCastTF(m_state.last_message_time);
      return;
    }

    m_state.last_send_tf_time = rclcpp::Clock().now();

    // Run rough + refine ICP and update the offset on success.
    M4F initial_guess = M4F::Identity();
    bool service_pending = false;
    {
      std::lock_guard<std::mutex> lock(m_state.service_mutex);
      service_pending = m_state.service_received;
      if (service_pending) initial_guess = m_state.initial_guess;
    }

    // Snapshot the odom pose before deciding whether to register: the motion
    // gate measures against it. setInput() stays on the align path — copying
    // the cloud into the localizer is the expensive part and a gated tick must
    // not pay it.
    M3D current_local_r;
    V3D current_local_t;
    builtin_interfaces::msg::Time current_time;
    {
      std::lock_guard<std::mutex> lock(m_state.message_mutex);
      current_local_r = m_state.last_r;
      current_local_t = m_state.last_t;
      current_time = m_state.last_message_time;
      if (!service_pending) {
        initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
        initial_guess.block<3, 1>(0, 3) =
          (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
      }
    }

    // A relocalize (or the config initial pose, which reaches us the same way)
    // must bypass the gate *and* the blend: it is the first step of a live map
    // switch, and easing 10% toward the requested pose would be worse than
    // useless. It also opens a full-rate settle window, because the pose only
    // converges over the rounds that follow.
    const rclcpp::Time now = rclcpp::Clock().now();
    if (service_pending) {
      m_state.settle_until = now + rclcpp::Duration::from_seconds(m_config.post_reloc_settle);
    }
    const bool settling = now < m_state.settle_until;

    double alpha = 1.0;
    if (!service_pending && !settling) {
      const double moved_trans = (current_local_t - m_state.last_reg_t).norm();
      // Rotation angle of last_reg_r -> current_local_r, via the trace.
      const M3D dr = current_local_r * m_state.last_reg_r.transpose();
      const double moved_rot = std::acos(std::clamp((dr.trace() - 1.0) * 0.5, -1.0, 1.0));
      const bool moved =
        moved_trans > m_config.min_update_trans || moved_rot > m_config.min_update_rot;
      const bool stale = (now - m_state.last_reg_time).seconds() > m_config.max_update_interval;

      if (m_state.has_reg && !moved && !stale) {
        // Parked and recently registered: the correct offset is whatever we
        // already hold. Rebroadcast it and skip GICP entirely.
        sendBroadCastTF(current_time);
        return;
      }
      if (!moved) alpha = m_config.static_blend_alpha;
    }

    {
      std::lock_guard<std::mutex> lock(m_state.message_mutex);
      m_localizer->setInput(m_state.last_cloud);
    }

    // With no map loaded the target cloud is empty and align() returns false outright, before
    // touching the guess. Normally loadInitialMap() has loaded it in the constructor; this only
    // happens when map_path was unset or unreadable and no relocalize / initialpose has loaded one
    // since.
    bool result = m_localizer->align(initial_guess);
    if (result) {
      M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
      V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
      // Re-orthonormalize: the ICP solution and the odom rotation are chained float products (and
      // are fed back into initial_guess on the next round), so accumulated error drifts R off
      // SO(3), and the norm of the quaternion it is converted to at broadcast time drifts with it
      // (tf2 denormalized-quaternion warning, and the skew/scale gets read as a spurious
      // displacement). Pull it back to a valid rotation. The blending path adds one more product
      // to the chain, which makes this step more important there; slerp itself returns a unit
      // quaternion.
      const M3D raw_offset_r = map_body_r * current_local_r.transpose();
      Eigen::Quaterniond cand_q(raw_offset_r);
      cand_q.normalize();
      M3D cand_r = cand_q.toRotationMatrix();
      // Use the same orthonormalized rotation for the translation so rotation and translation
      // stay consistent.
      V3D cand_t = -cand_r * current_local_t + map_body_t;

      // Build the candidate in locals and blend afterwards: writing
      // last_offset_r first and then deriving last_offset_t from it (as this
      // used to) would mix a blended rotation into an unblended translation.
      // Eigen's slerp picks the short arc itself, so no sign fix-up is needed.
      if (alpha < 1.0 && m_state.has_reg) {
        cand_q = Eigen::Quaterniond(m_state.last_offset_r).slerp(alpha, cand_q);
        cand_r = cand_q.toRotationMatrix();
        cand_t = (1.0 - alpha) * m_state.last_offset_t + alpha * cand_t;
      }
      m_state.last_offset_r = cand_r;
      m_state.last_offset_t = cand_t;

      m_state.last_reg_r = current_local_r;
      m_state.last_reg_t = current_local_t;
      m_state.last_reg_time = now;
      m_state.has_reg = true;

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

    // This is the only place the map changes; publish it once, latched, for RViz and any other
    // subscriber.
    builtin_interfaces::msg::Time stamp = this->now();
    publishMapCloud(stamp);

    response->success = true;
    response->message = "relocalize success";
    return;
  }

  void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // On the normal path the map was already loaded during construction (loadInitialMap); this
    // block is only reached when map_path was not set or that load failed. It is kept because,
    // with no map loaded, align() returns false outright on the empty target (the guess is not
    // even logged), so initialpose looks like it did nothing at all — which is exactly how it bit
    // us on the robot. Here we either load the map successfully or warn explicitly and ignore the
    // message, never leaving behind a stale guess that only takes effect once a map is loaded.
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

  // Build the ICP initial guess from a 2D start pose (x, y, yaw). Shared by two sources: the
  // initialpose from RViz / the console's gridmap, and the boot start pose from system.ini
  // [initial_pose].
  //
  // A 2D source can only yield x, y, yaw (z=0, roll=pitch=0). But with a tilted lidar mount and a
  // gravity-aligned map, map_T_body always carries the ~15° mount pitch: using the 2D pose
  // directly as the guess tilts the whole scan relative to the map, the error of far-away points
  // exceeds rough_max_corr_dist, and rough ICP never passes the score threshold — and while the
  // guess is pending, timerCB also skips the odom feedback path, so localization freezes on the
  // old offset and retries forever (measured on the robot: an initialpose at (5.0, 0.982) hung;
  // with the 14.6° pitch filled in, the same spot converged on the first round).
  // So only x, y, yaw are taken; roll/pitch/z are always filled in from the current estimate.
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
        // Keep the gravity tilt of the current pose and only turn the heading to the requested yaw.
        guess_r = Eigen::AngleAxisd(yaw - cur_yaw, V3D::UnitZ()).toRotationMatrix() * cur_r;
        guess_t.z() = cur_t.z();
      }
      // Early at boot, before any odom has arrived, there is no estimate to fill in from, so the
      // pure 2D pose is used (the config start pose waits for the first odom and never gets here).
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
    // Latched: publish even when there is no subscriber right now; DDS caches it for late joiners.
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
  // Two threads: one for the timer/subscriber group, one for the services, so the TF rebroadcast
  // keeps running while relocalize's loadMap is in progress.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
