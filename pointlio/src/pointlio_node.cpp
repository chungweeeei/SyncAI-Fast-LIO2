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
#include <yaml-cpp/yaml.h>

#include "map_builder/commons.h"
#include "map_builder/map_builder.h"
#include "tf2_ros/transform_broadcaster.h"
#include "utils.h"

using namespace std::chrono_literals;
struct NodeConfig
{
  std::string imu_topic = "/livox/imu";
  std::string lidar_topic = "/livox/lidar";
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
      m_node_config.imu_topic, 10, std::bind(&PointLIONode::imuCB, this, std::placeholders::_1));

    // register scan subscriber
    if (m_node_config.lidar_type == 1) {
      RCLCPP_INFO(
        this->get_logger(), "[PointLIONode][%s] Lidar input: sensor_msgs/PointCloud2 (%s)",
        __func__, m_node_config.lidar_topic.c_str());
      m_pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        m_node_config.lidar_topic, 10,
        std::bind(&PointLIONode::pc2CB, this, std::placeholders::_1));
    } else {
      RCLCPP_INFO(
        this->get_logger(), "Lidar input: livox_ros_driver2/CustomMsg (%s)",
        m_node_config.lidar_topic.c_str());
      m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        m_node_config.lidar_topic, 10,
        std::bind(&PointLIONode::lidarCB, this, std::placeholders::_1));
    }

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

  void loadParameters()
  {
    this->declare_parameter("config_path", "");
    std::string config_path;
    this->get_parameter<std::string>("config_path", config_path);

    YAML::Node config = YAML::LoadFile(config_path);
    if (!config) {
      RCLCPP_WARN(this->get_logger(), "[PointLIONode][%s] FAIL TO LOAD YAML FILE!", __func__);
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "[PointLIONode][%s] LOAD FROM YAML CONFIG PATH: %s", __func__,
      config_path.c_str());

    // config for pointlio node
    m_node_config.imu_topic = config["imu_topic"].as<std::string>();
    m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
    m_node_config.body_frame = config["body_frame"].as<std::string>();
    m_node_config.world_frame = config["world_frame"].as<std::string>();
    m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
    if (config["lidar_type"]) m_node_config.lidar_type = config["lidar_type"].as<int>();
    if (config["imu_acc_scale"]) m_node_config.imu_acc_scale = config["imu_acc_scale"].as<double>();

    // config for map builder
    m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
    m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
    m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
    m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
    m_builder_config.map_resolution = config["map_resolution"].as<double>();
    m_builder_config.cube_len = config["cube_len"].as<double>();
    m_builder_config.det_range = config["det_range"].as<double>();
    m_builder_config.move_thresh = config["move_thresh"].as<double>();

    // Point-LIO output model
    m_builder_config.gyr_cov_output = config["gyr_cov_output"].as<double>();
    m_builder_config.acc_cov_output = config["acc_cov_output"].as<double>();
    m_builder_config.b_gyr_cov = config["b_gyr_cov"].as<double>();
    m_builder_config.b_acc_cov = config["b_acc_cov"].as<double>();
    m_builder_config.imu_meas_omg_cov = config["imu_meas_omg_cov"].as<double>();
    m_builder_config.imu_meas_acc_cov = config["imu_meas_acc_cov"].as<double>();
    m_builder_config.satu_gyro = config["satu_gyro"].as<double>();
    m_builder_config.satu_acc = config["satu_acc"].as<double>();
    m_builder_config.lidar_meas_cov = config["lidar_meas_cov"].as<double>();
    m_builder_config.plane_thr = config["plane_thr"].as<double>();
    m_builder_config.batch_max_points = config["batch_max_points"].as<int>();

    m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
    m_builder_config.near_search_num = config["near_search_num"].as<int>();
    m_builder_config.gravity_align = config["gravity_align"].as<bool>();
    m_builder_config.esti_il = config["esti_il"].as<bool>();
    std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
    std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
    m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
    m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4],
      r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
  }

  void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
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
      // sorted
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
    // 將該幀需要用的 IMU 資料撈出來並把 imu_buffer 跟 lidar_buffer 清乾淨
    while (!m_state_data.imu_buffer.empty() &&
           m_state_data.imu_buffer.front().time < m_package.cloud_end_time) {
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
