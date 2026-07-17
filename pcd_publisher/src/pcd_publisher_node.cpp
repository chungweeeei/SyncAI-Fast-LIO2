#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// Loads a .pcd file once and publishes it a single time as a PointCloud2 for
// visualization (e.g. inspecting a FAST-LIO / PGO / HBA saved map in RViz).
// The publisher uses transient_local QoS, so late-joining subscribers still
// receive the last (only) message without needing a periodic re-publish.
class PCDPublisher : public rclcpp::Node
{
public:
  PCDPublisher() : Node("pcd_publisher")
  {
    m_pcd_path = this->declare_parameter<std::string>("pcd_path", "");
    m_frame_id = this->declare_parameter<std::string>("frame_id", "map");
    m_topic = this->declare_parameter<std::string>("topic", "/pcd_map");

    if (m_pcd_path.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'pcd_path' is empty; nothing to publish.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(m_pcd_path, cloud) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", m_pcd_path.c_str());
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = m_frame_id;
    cloud_msg.header.stamp = this->now();

    // Latched-style QoS so RViz picks up the map whenever it connects.
    rclcpp::QoS qos(1);
    qos.transient_local().reliable();
    m_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_topic, qos);
    m_pub->publish(cloud_msg);

    RCLCPP_INFO(this->get_logger(), "Loaded %s (%u points), published once on '%s' in frame '%s'.",
                m_pcd_path.c_str(), cloud_msg.width * cloud_msg.height,
                m_topic.c_str(), m_frame_id.c_str());
  }

private:
  std::string m_pcd_path;
  std::string m_frame_id;
  std::string m_topic;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCDPublisher>());
  rclcpp::shutdown();
  return 0;
}
