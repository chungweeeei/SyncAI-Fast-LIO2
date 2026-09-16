#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <queue>
#include <thread>

#include "interface/srv/save_maps.hpp"
#include "interface/srv/reset_lio.hpp"
#include "interface/srv/reset_mapping.hpp"
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"

using namespace std::chrono_literals;

struct NodeConfig
{
  std::string cloud_topic = "/lio/body_cloud";
  std::string odom_topic = "/lio/odom";
  std::string map_frame = "map";
  std::string local_frame = "lidar";
  // The published "map so far" merge (see publishMapCloud). Node-level publish
  // behaviour, not PGO math, so they live here rather than in Config.
  double map_cloud_resolution = 0.2;
  double map_cloud_pub_period = 3.0;
  // pointlio's reset service. Absolute, and rewritten per robot_id by
  // pgo_launch.py for exactly the reason cloud_topic and odom_topic are: it
  // names something in pointlio's namespace, which a relative name from inside
  // /<robot_id>/pgo cannot reach. Keeping it in the config is what stops this
  // file from ever spelling a robot_id.
  std::string lio_reset_service = "/pointlio/reset";
};

struct NodeState
{
  std::mutex message_mutex;
  std::queue<CloudWithPose> cloud_buffer;
  // Was uninitialised, so the out-of-order guard in syncCB compared the very
  // first message against whatever was on the stack. -1.0 is the "no message
  // seen yet" value, and is also what resetMappingCB puts back.
  double last_message_time = -1.0;

  // The reset gate. Atomics rather than fields under message_mutex on purpose:
  // syncCB would otherwise have to take m_pgo_mutex *and* message_mutex in a
  // fixed order on its hot path, and this file has already proved it cannot be
  // trusted with one lock (see the lock_guard note below). Two relaxed loads
  // keep the gate lock-free and remove lock ordering from the design entirely.
  //
  // accepting is false for the duration of a reset -- nothing the front end
  // publishes can reach the graph while pointlio's state is changing, which is
  // what makes the reset ordering-proof rather than timing-dependent.
  // accept_after_time then discards the old run's tail: pairs already held by
  // the message_filters synchroniser, which surface after accepting goes true
  // again. Both sides of that comparison are the lidar header stamp, so it is
  // exact -- no clock conversion, no tolerance constant.
  std::atomic<bool> accepting{true};
  std::atomic<double> accept_after_time{-1.0};
};

class PGONode : public rclcpp::Node
{
public:
  PGONode() : Node("pgo_node")
  {
    RCLCPP_INFO(this->get_logger(), "PGO node started");
    loadParameters();
    m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
    rclcpp::QoS qos = rclcpp::QoS(10);
    m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
    m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
    // Relative name, was the absolute "/pgo/loop_markers": the launch runs this
    // node at /<robot_id>/pgo, and an absolute name ignores that namespace, so
    // two robots in one DDS domain would publish onto the same topic. The
    // inputs above stay absolute on purpose — they live in pointlio's
    // namespace, which a relative name cannot reach.
    m_loop_marker_pub =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("loop_markers", 10000);
    // Relative like loop_markers, so it lands on /<robot_id>/pgo/map_cloud.
    // Depth 1: each message is a multi-MB full-map merge and only the latest
    // matters — queueing old merges would just hold memory.
    m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_cloud", rclcpp::QoS(1));
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    m_sync = std::make_shared<
      message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
      m_cloud_sub, m_odom_sub);
    m_sync->setAgePenalty(0.1);
    m_sync->registerCallback(
      std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
    m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
    // Relative for the same reason as loop_markers — the service is now
    // /<robot_id>/pgo/save_maps.
    m_save_map_srv = this->create_service<interface::srv::SaveMaps>(
      "save_maps",
      std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));

    // Two dedicated MutuallyExclusive groups, same shape (and the same reason)
    // as localizer_node's: a handler that blocks must not sit on the group that
    // owns the 50 ms TF broadcast.
    //
    // The client's group is not a nicety, it is what stops a deadlock.
    // resetMappingCB blocks on the ResetLIO future, so the response has to be
    // delivered by a DIFFERENT executor thread, which means a different
    // callback group. (And never spin_until_future_complete from inside a
    // callback -- that re-enters the executor that is already busy running us.)
    m_srv_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cli_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_reset_srv = this->create_service<interface::srv::ResetMapping>(
      "reset_mapping",
      std::bind(&PGONode::resetMappingCB, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, m_srv_cb_group);

    m_lio_reset_cli = this->create_client<interface::srv::ResetLIO>(
      m_node_config.lio_reset_service, rmw_qos_profile_services_default, m_cli_cb_group);
  }

  ~PGONode()
  {
    // A merge may still be running on the worker; joining here keeps shutdown
    // from tearing the publisher down under it.
    if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();
  }

  void loadParameters()
  {
    this->declare_parameter("config_path", "");
    std::string config_path;
    this->get_parameter<std::string>("config_path", config_path);
    YAML::Node config = YAML::LoadFile(config_path);
    if (!config) {
      RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
    m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
    m_node_config.odom_topic = config["odom_topic"].as<std::string>();
    m_node_config.map_frame = config["map_frame"].as<std::string>();
    m_node_config.local_frame = config["local_frame"].as<std::string>();

    m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
    m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
    m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
    m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
    m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
    m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
    m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
    m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();

    // Defensive, unlike the keys above: a config generated by an older
    // pgo_launch.py (or a stale copy under /tmp/syncai_pgo/) simply lacks
    // these, and the node should fall back to the defaults rather than throw.
    if (config["map_cloud_resolution"]) {
      m_node_config.map_cloud_resolution = config["map_cloud_resolution"].as<double>();
    }
    if (config["map_cloud_pub_period"]) {
      m_node_config.map_cloud_pub_period = config["map_cloud_pub_period"].as<double>();
    }
    if (config["lio_reset_service"]) {
      m_node_config.lio_reset_service = config["lio_reset_service"].as<std::string>();
    }
  }

  void syncCB(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
  {
    /**
     * cloud_msg: lio odom frame 上的 pointcloud
     * odom_msg: lio odom -> robot pose
     */

    // Gate before the expensive part: a pair dropped here costs nothing, while
    // pcl::fromROSMsg below is a full copy of a lidar frame.
    //
    // accepting false means a reset is in flight -- see NodeState. Dropping
    // rather than buffering is the point: anything produced while the front end
    // is being reset belongs to neither run.
    if (!m_state.accepting.load()) return;

    CloudWithPose cp;
    cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);

    // The old run's tail. pointlio reported this boundary as the stamp of the
    // last odometry it published before resetting, so <= is the whole of the
    // old run and > is the whole of the new one. Without this, a single
    // straddling frame anchors a BetweenFactor carrying the entire accumulated
    // drift at 1e-6 variance, and the graph never recovers.
    if (cp.pose.second <= m_state.accept_after_time.load()) return;

    if (cp.pose.second < m_state.last_message_time) {
      RCLCPP_WARN(this->get_logger(), "Received out of order message");
      return;
    }

    // Was `std::lock_guard<std::mutex>(m_state.message_mutex);` -- with no
    // declarator-id that parses as a functional cast, so it built a temporary
    // and released the mutex at the end of that very statement. The buffer was
    // unguarded, and got away with it only because rclcpp::spin() serialised
    // syncCB against timerCB. main() now runs a MultiThreadedExecutor, so it
    // would not get away with it any more.
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    m_state.last_message_time = cp.pose.second;

    cp.pose.r = Eigen::Quaterniond(
                  odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                  odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)
                  .toRotationMatrix();
    cp.pose.t = V3D(
      odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z);
    cp.cloud = CloudType::Ptr(new CloudType);
    pcl::fromROSMsg(*cloud_msg, *cp.cloud);
    m_state.cloud_buffer.push(cp);
  }

  void sendBroadCastTF(builtin_interfaces::msg::Time & time)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = m_node_config.map_frame;
    transformStamped.child_frame_id = m_node_config.local_frame;
    transformStamped.header.stamp = time;
    Eigen::Quaterniond q(m_pgo->offsetR());
    V3D t = m_pgo->offsetT();
    transformStamped.transform.translation.x = t.x();
    transformStamped.transform.translation.y = t.y();
    transformStamped.transform.translation.z = t.z();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    m_tf_broadcaster->sendTransform(transformStamped);
  }

  void publishLoopMarkers(builtin_interfaces::msg::Time & time)
  {
    if (m_loop_marker_pub->get_subscription_count() == 0) return;
    if (m_pgo->historyPairs().size() == 0) return;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker nodes_marker;
    visualization_msgs::msg::Marker edges_marker;
    nodes_marker.header.frame_id = m_node_config.map_frame;
    nodes_marker.header.stamp = time;
    nodes_marker.ns = "pgo_nodes";
    nodes_marker.id = 0;
    nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodes_marker.action = visualization_msgs::msg::Marker::ADD;
    nodes_marker.pose.orientation.w = 1.0;
    nodes_marker.scale.x = 0.3;
    nodes_marker.scale.y = 0.3;
    nodes_marker.scale.z = 0.3;
    nodes_marker.color.r = 1.0;
    nodes_marker.color.g = 0.8;
    nodes_marker.color.b = 0.0;
    nodes_marker.color.a = 1.0;

    edges_marker.header.frame_id = m_node_config.map_frame;
    edges_marker.header.stamp = time;
    edges_marker.ns = "pgo_edges";
    edges_marker.id = 1;
    edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges_marker.action = visualization_msgs::msg::Marker::ADD;
    edges_marker.pose.orientation.w = 1.0;
    edges_marker.scale.x = 0.1;
    edges_marker.color.r = 0.0;
    edges_marker.color.g = 0.8;
    edges_marker.color.b = 0.0;
    edges_marker.color.a = 1.0;

    std::vector<KeyPoseWithCloud> & poses = m_pgo->keyPoses();
    std::vector<std::pair<size_t, size_t>> & pairs = m_pgo->historyPairs();
    for (size_t i = 0; i < pairs.size(); i++) {
      size_t i1 = pairs[i].first;
      size_t i2 = pairs[i].second;
      geometry_msgs::msg::Point p1, p2;
      p1.x = poses[i1].t_global.x();
      p1.y = poses[i1].t_global.y();
      p1.z = poses[i1].t_global.z();

      p2.x = poses[i2].t_global.x();
      p2.y = poses[i2].t_global.y();
      p2.z = poses[i2].t_global.z();

      nodes_marker.points.push_back(p1);
      nodes_marker.points.push_back(p2);
      edges_marker.points.push_back(p1);
      edges_marker.points.push_back(p2);
    }

    marker_array.markers.push_back(nodes_marker);
    marker_array.markers.push_back(edges_marker);
    m_loop_marker_pub->publish(marker_array);
  }

  void timerCB()
  {
    // Held for the whole body: everything below reads or mutates m_pgo, which
    // resetMappingCB replaces wholesale. See the m_pgo_mutex declaration for the
    // full discipline.
    std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);

    CloudWithPose cp;
    {
      // front() used to be read OUTSIDE this block, and the block itself used to
      // be `std::lock_guard<std::mutex>(m_state.message_mutex);` -- a temporary,
      // so nothing was ever locked. Both are fixed together: the read and the
      // drain are one critical section against syncCB, which now runs on a
      // different executor thread.
      std::lock_guard<std::mutex> lock(m_state.message_mutex);
      if (m_state.cloud_buffer.empty()) return;
      cp = m_state.cloud_buffer.front();  // 只拿最舊的一筆資料
      // 把整個 queue 清空
      while (!m_state.cloud_buffer.empty()) {
        m_state.cloud_buffer.pop();
      }
    }

    builtin_interfaces::msg::Time cur_time;
    cur_time.sec = cp.pose.sec;
    cur_time.nanosec = cp.pose.nsec;

    if (!m_pgo->addKeyPose(cp)) {  // 挑關鍵幀
      sendBroadCastTF(cur_time);   // 不是關鍵幀 -> 只發送 transform
      return;
    }

    // 找回環
    m_pgo->searchForLoopPairs();

    // 圖優化
    m_pgo->smoothAndUpdate();

    // 發transform
    sendBroadCastTF(cur_time);

    publishLoopMarkers(cur_time);

    // 只有 keyframe tick 會走到這裡 — 也正是合併地圖真的有變化的時刻。
    publishMapCloud(cur_time);
  }

  void publishMapCloud(builtin_interfaces::msg::Time & time)
  {
    // Same gate as publishLoopMarkers: with nobody listening (the operator
    // console's backend is the intended subscriber) the merge is pure waste.
    if (m_map_cloud_pub->get_subscription_count() == 0) return;
    if (m_pgo->keyPoses().empty()) return;

    // Keyframe-triggered AND rate-floored: keyframes land every ~0.5 m of
    // travel, which early in a run is faster than anyone needs a multi-MB
    // full-map merge. Timestamps are the keyframes' own (sensor time), so the
    // floor also behaves under sim/bag time.
    double now_s = m_pgo->keyPoses().back().time;
    if (now_s - m_last_map_cloud_time < m_node_config.map_cloud_pub_period) return;

    // One worker at a time. exchange() is the claim; the worker releases it as
    // its last act. If the previous merge is still running we simply skip this
    // keyframe — the next one re-triggers, and a merge that cannot keep up
    // with the period degrades to back-to-back merges, never to a queue.
    if (m_map_cloud_busy.exchange(true)) return;
    m_last_map_cloud_time = now_s;

    // busy was false, so a previous thread (if any) has finished executing;
    // join() only reclaims it and returns immediately.
    if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();

    // Snapshot by value, made HERE on the timer thread — the only thread that
    // ever mutates m_key_poses. Poses are copied (~200 B per keyframe);
    // body_cloud Ptrs are shared, which is safe because a keyframe's cloud is
    // never written after insertion (smoothAndUpdate rewrites poses only).
    // This is also why the merge does NOT go through SimplePGO::getSubMap():
    // that reads m_key_poses live and would race the next timer tick.
    std::vector<KeyPoseWithCloud> snapshot = m_pgo->keyPoses();
    m_map_cloud_thread =
      std::thread(&PGONode::mergeAndPublishMapCloud, this, std::move(snapshot), time);
  }

  // Runs on m_map_cloud_thread. The whole point of the thread: late in a run
  // this is a transform+concat over millions of points plus a VoxelGrid —
  // hundreds of ms — and timerCB owns the map->local_frame TF broadcast, which
  // must not gap for that long. rclcpp publishers are thread-safe.
  void mergeAndPublishMapCloud(
    std::vector<KeyPoseWithCloud> snapshot, builtin_interfaces::msg::Time time)
  {
    // 同 saveMapsCB 的合併: 逐 keyframe 用 (r_global, t_global) 轉到 map frame 疊加。
    CloudType::Ptr merged(new CloudType);
    for (const auto & kp : snapshot) {
      CloudType::Ptr world_cloud(new CloudType);
      pcl::transformPointCloud(
        *kp.body_cloud, *world_cloud, kp.t_global, Eigen::Quaterniond(kp.r_global));
      *merged += *world_cloud;
    }

    if (m_node_config.map_cloud_resolution > 0) {
      pcl::VoxelGrid<PointType> voxel_filter;
      voxel_filter.setLeafSize(
        m_node_config.map_cloud_resolution, m_node_config.map_cloud_resolution,
        m_node_config.map_cloud_resolution);
      voxel_filter.setInputCloud(merged);
      voxel_filter.filter(*merged);
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*merged, msg);
    // Already global: every point was placed with the snapshot's corrected
    // poses, so downstream needs no TF — and a re-publish after a loop closure
    // moves the whole map into its corrected shape.
    msg.header.frame_id = m_node_config.map_frame;
    msg.header.stamp = time;
    m_map_cloud_pub->publish(msg);

    // Last, deliberately: this is what lets publishMapCloud spawn the next
    // worker, and everything this thread does must be finished by then.
    m_map_cloud_busy.store(false);
  }

  // Tell every consumer the map is gone. Both are published UNGATED, unlike
  // their counterparts in the normal path: the subscriber-count check exists to
  // skip expensive merges nobody wants, but "the map is empty now" is two dozen
  // bytes and is precisely the message a late or idle subscriber must not miss.
  // Without these, the last thing rviz and the operator console hold is the map
  // the operator was just told had been discarded.
  void publishEmptyMapCloud(const builtin_interfaces::msg::Time & time)
  {
    CloudType empty;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(empty, msg);
    msg.header.frame_id = m_node_config.map_frame;
    msg.header.stamp = time;
    m_map_cloud_pub->publish(msg);
  }

  void publishLoopMarkerDeleteAll()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    m_loop_marker_pub->publish(marker_array);
  }

  // Throw the pose graph away and start a new map, with nothing restarted.
  //
  // Four phases, ordered so that the only step which can fail happens before
  // anything is destroyed. The single reachable partial state is "paused, graph
  // intact, LIO untouched", and every failure path exits through the resume --
  // so there is no half-reset for a caller to clean up, and no ordering exposed
  // to a caller to get wrong.
  void resetMappingCB(
    const std::shared_ptr<interface::srv::ResetMapping::Request> request,
    std::shared_ptr<interface::srv::ResetMapping::Response> response)
  {
    // ---- Phase 1: pause. Reversible, and the whole ordering fix. ----
    //
    // With accepting false, nothing the front end publishes can reach the graph
    // while pointlio's state changes underneath us. That is why this design
    // needs no sleep and no slack window: the odometry discontinuity has
    // nowhere to land, rather than landing somewhere we hope is harmless.
    if (m_resetting.exchange(true)) {
      response->success = false;
      response->message = "A reset is already running";
      return;
    }
    // RAII so every early return below -- including an exception out of the
    // client -- clears the flag and re-opens the gate.
    struct ResumeGuard
    {
      PGONode * self;
      bool committed = false;
      ~ResumeGuard()
      {
        if (!committed) self->m_state.accepting.store(true);
        self->m_resetting.store(false);
      }
    } resume_guard{this};

    m_state.accepting.store(false);

    // ---- Phase 2: reset the front end. The only fallible step, no mutex. ----
    //
    // Deliberately outside m_pgo_mutex: this blocks for up to five seconds, and
    // holding the lock would stall the 50 ms timer -- and with it the
    // map -> local_frame TF broadcast -- for that whole time.
    double last_odom_time = 0.0;
    if (request->reset_lio) {
      if (!m_lio_reset_cli->wait_for_service(std::chrono::seconds(2))) {
        response->success = false;
        response->message =
          "LIO reset service " + m_node_config.lio_reset_service + " is not available; map kept";
        RCLCPP_ERROR(this->get_logger(), "[PGONode][resetMappingCB] %s", response->message.c_str());
        return;
      }

      auto future = m_lio_reset_cli->async_send_request(
        std::make_shared<interface::srv::ResetLIO::Request>());
      if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        response->success = false;
        response->message = "Timed out waiting for the LIO reset; map kept";
        RCLCPP_ERROR(this->get_logger(), "[PGONode][resetMappingCB] %s", response->message.c_str());
        return;
      }

      auto lio_response = future.get();
      if (!lio_response->success) {
        response->success = false;
        response->message = "LIO refused the reset (" + lio_response->message + "); map kept";
        RCLCPP_ERROR(this->get_logger(), "[PGONode][resetMappingCB] %s", response->message.c_str());
        return;
      }
      last_odom_time = lio_response->last_odom_time;
    }

    // ---- Phase 3: reset the graph. Nothing below can fail. ----
    builtin_interfaces::msg::Time now = this->get_clock()->now();
    uint32_t dropped = 0;
    {
      std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);

      // NOT for safety -- mergeAndPublishMapCloud works on a by-value snapshot
      // and never touches m_pgo, so destroying SimplePGO under it was already
      // fine (the keyframe clouds are refcounted). The join is here so an
      // in-flight merge cannot publish the OLD map after we publish the empty
      // one, which would undo the only thing telling consumers the map is gone.
      if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();
      m_map_cloud_busy.store(false);
      m_last_map_cloud_time = 0.0;

      dropped = static_cast<uint32_t>(m_pgo->keyPoses().size());

      // The constructor IS the reset: fresh ISAM2, empty values and graph,
      // identity offsets, and the three keyframe vectors empty by virtue of
      // being a new object. Deliberately not a SimplePGO::reset() method --
      // gtsam::ISAM2 has no clear, so such a method would be a second
      // definition of "empty" that has to stay in sync with this one.
      m_pgo = std::make_shared<SimplePGO>(m_pgo_config);

      {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        // swap, not a pop() loop: pop() leaves the deque's capacity behind, and
        // every entry here drags a full body cloud with it.
        std::queue<CloudWithPose>().swap(m_state.cloud_buffer);
        m_state.last_message_time = -1.0;
      }
      m_state.accept_after_time.store(last_odom_time);

      publishEmptyMapCloud(now);
      publishLoopMarkerDeleteAll();
    }

    // ---- Phase 4: resume. ----
    m_state.accepting.store(true);
    resume_guard.committed = true;

    response->success = true;
    response->lio_last_odom_time = last_odom_time;
    response->dropped_key_poses = dropped;
    // Rendered verbatim by the operator console, so it is written as UI copy
    // rather than as a log line -- and it is the LAST place the stillness
    // warning can land: the dialog warns before the click, but the static IMU
    // initialisation itself happens in the seconds after this returns.
    response->message = request->reset_lio
                          ? "Map discarded. The new one starts building once the "
                            "lidar has re-levelled — keep the robot still until then."
                          : "Pose graph reset. The LIO front end was left running.";
    RCLCPP_WARN(
      this->get_logger(), "[PGONode][resetMappingCB] %s (dropped %u key poses)",
      response->message.c_str(), dropped);
  }

  void saveMapsCB(
    const std::shared_ptr<interface::srv::SaveMaps::Request> request,
    std::shared_ptr<interface::srv::SaveMaps::Response> response)
  {
    // Whole body, like timerCB: this iterates m_pgo->keyPoses() and writes a
    // multi-MB PCD out of it. A reset waiting behind a save is the correct
    // outcome -- the save is serialising exactly what the reset is about to
    // destroy, and letting them interleave would write a half-reset map.
    std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);

    if (!std::filesystem::exists(request->file_path)) {
      response->success = false;
      response->message = request->file_path + " IS NOT EXISTS!";
      return;
    }

    if (m_pgo->keyPoses().size() == 0) {
      response->success = false;
      response->message = "NO POSES!";
      return;
    }

    std::filesystem::path p_dir(request->file_path);
    std::filesystem::path patches_dir = p_dir / "patches";       // 分片點雲資料夾
    std::filesystem::path poses_txt_path = p_dir / "poses.txt";  // 每幀位姿清單
    std::filesystem::path map_path = p_dir / "map.pcd";          // 合併後完整地圖

    if (request->save_patches) {
      if (std::filesystem::exists(patches_dir)) {
        std::filesystem::remove_all(patches_dir);
      }

      std::filesystem::create_directories(patches_dir);

      if (std::filesystem::exists(poses_txt_path)) {
        std::filesystem::remove(poses_txt_path);
      }
      RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
    }
    RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

    std::ofstream txt_file(poses_txt_path);

    CloudType::Ptr ret(new CloudType);
    for (size_t i = 0; i < m_pgo->keyPoses().size(); i++) {
      CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
      if (request->save_patches) {
        std::string patch_name = std::to_string(i) + ".pcd";
        std::filesystem::path patch_path = patches_dir / patch_name;
        pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
        Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
        V3D t = m_pgo->keyPoses()[i].t_global;
        txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w()
                 << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
      }
      CloudType::Ptr world_cloud(new CloudType);
      pcl::transformPointCloud(
        *body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global,
        Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));

      // 疊近合併地圖
      *ret += *world_cloud;
    }
    txt_file.close();

    // 存完整地圖
    pcl::io::savePCDFileBinary(map_path.string(), *ret);
    response->success = true;
    response->message = "SAVE SUCCESS!";
  }

private:
  NodeConfig m_node_config;
  Config m_pgo_config;
  NodeState m_state;
  std::shared_ptr<SimplePGO> m_pgo;
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
  // The map-cloud worker. busy is claimed by publishMapCloud on the timer
  // thread and released by mergeAndPublishMapCloud as its last act; the thread
  // handle is only ever joined while busy is false (or in the destructor).
  std::atomic<bool> m_map_cloud_busy{false};
  double m_last_map_cloud_time = 0.0;
  std::thread m_map_cloud_thread;
  rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
  // The reset surface. m_resetting rejects a second concurrent call outright
  // rather than queueing it -- two resets in flight would have the second one
  // reading a boundary timestamp the first had already invalidated.
  rclcpp::CallbackGroup::SharedPtr m_srv_cb_group;
  rclcpp::CallbackGroup::SharedPtr m_cli_cb_group;
  rclcpp::Service<interface::srv::ResetMapping>::SharedPtr m_reset_srv;
  rclcpp::Client<interface::srv::ResetLIO>::SharedPtr m_lio_reset_cli;
  std::atomic<bool> m_resetting{false};
  // Guards m_pgo -- which resetMappingCB REPLACES rather than mutates, so every
  // reader needs to be excluded, not just every writer. Discipline:
  //
  //   timerCB          whole body (addKeyPose, searchForLoopPairs,
  //                    smoothAndUpdate, the TF broadcast, and the
  //                    m_map_cloud_busy / m_map_cloud_thread handshake)
  //   saveMapsCB       whole body (it serialises what a reset would destroy)
  //   resetMappingCB   phases 1, 3 and 4 -- NEVER while waiting on the LIO
  //                    future, which would stall the TF broadcast for seconds
  //
  // Needed only because main() runs a MultiThreadedExecutor now; under the old
  // rclcpp::spin() the executor itself provided this exclusion.
  std::mutex m_pgo_mutex;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>
    m_sync;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Named, NOT `add_node(std::make_shared<PGONode>())`. Executor::add_node
  // stores a weak_ptr, so a temporary shared_ptr dies at the end of that
  // statement and takes the node with it -- the process then spins forever over
  // an empty node set, publishing nothing and broadcasting no TF, while looking
  // alive in `ps`. (rclcpp::spin(make_shared<...>()), which this replaced, is
  // safe only because the argument outlives the call.) The symptom is a console
  // with no point cloud and a DDS "Finis." a third of a second after startup.
  auto node = std::make_shared<PGONode>();
  // Three threads for three groups, the same arrangement (and the same reason)
  // as localizer_node: the default group keeps the timer, both subscriptions
  // and save_maps serialised exactly as rclcpp::spin() did, so nothing about
  // the steady-state behaviour moves. The other two exist purely so
  // resetMappingCB can block on a client future without deadlocking itself --
  // the handler runs on one, its response arrives on the other.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}