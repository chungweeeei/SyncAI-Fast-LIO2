#include "icp_localizer.h"

#include <rclcpp/logging.hpp>

#include <cmath>

namespace
{
// log 用：印出 4x4 位姿的平移 (m) 與 ZYX 尤拉角 (deg)，方便逐輪對照 ICP 解的跳動
void logPose(const char * tag, const M4F & T)
{
  const float rad2deg = 180.f / static_cast<float>(M_PI);
  const float roll = std::atan2(T(2, 1), T(2, 2)) * rad2deg;
  const float pitch = std::asin(-T(2, 0)) * rad2deg;
  const float yaw = std::atan2(T(1, 0), T(0, 0)) * rad2deg;
  RCLCPP_INFO(
    rclcpp::get_logger("icp_localizer"), "%s: t=(%.3f, %.3f, %.3f)m rpy=(%.2f, %.2f, %.2f)deg", tag,
    T(0, 3), T(1, 3), T(2, 3), roll, pitch, yaw);
}
}  // namespace

ICPLocalizer::ICPLocalizer(const ICPConfig & config) : m_config(config)
{
  m_refine_inp.reset(new CloudType);
  m_refine_tgt.reset(new CloudType);
  m_rough_inp.reset(new CloudType);
  m_rough_tgt.reset(new CloudType);

  // PCL 預設的 correspondence distance 是無上限（sqrt(DBL_MAX)）：
  // guess 偏差大時 scan 每個點都會硬配到地圖上某個點，遠處錯誤配對
  // 一起參與最佳化，有收斂到錯誤位姿的風險。rough 給大一點的收斂範圍
  // 吸收 relocalize 手動 guess 的誤差，refine 收緊做精配
  m_rough_icp.setMaxCorrespondenceDistance(m_config.rough_max_corr_dist);
  m_refine_icp.setMaxCorrespondenceDistance(m_config.refine_max_corr_dist);
}
bool ICPLocalizer::loadMap(const std::string & path)
{
  if (!std::filesystem::exists(path)) {
    std::cerr << "Map file not found: " << path << std::endl;
    return false;
  }
  pcl::PCDReader reader;
  CloudType::Ptr cloud(new CloudType);
  reader.read(path, *cloud);

  // The slow part (file IO + downsampling) works on local clouds with a
  // local filter, so align() keeps running on the previous map meanwhile;
  // only the pointer swap at the end takes the lock. m_voxel_filter is not
  // used here because setInput() may be using it on the timer thread.
  CloudType::Ptr refine_tgt(new CloudType);
  CloudType::Ptr rough_tgt(new CloudType);
  pcl::VoxelGrid<PointType> voxel_filter;
  if (m_config.refine_map_resolution > 0) {
    voxel_filter.setLeafSize(
      m_config.refine_map_resolution, m_config.refine_map_resolution,
      m_config.refine_map_resolution);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*refine_tgt);
  } else {
    pcl::copyPointCloud(*cloud, *refine_tgt);
  }

  if (m_config.rough_map_resolution > 0) {
    voxel_filter.setLeafSize(
      m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*rough_tgt);
  } else {
    pcl::copyPointCloud(*cloud, *rough_tgt);
  }

  if (refine_tgt->empty() || rough_tgt->empty()) {
    std::cerr << "Map file is empty: " << path << std::endl;
    return false;
  }

  std::lock_guard<std::mutex> lock(m_target_mutex);
  m_refine_tgt.swap(refine_tgt);
  m_rough_tgt.swap(rough_tgt);
  // target 只在這裡設一次：PCL 只有在 setInputTarget 之後的第一次 align 才會
  // 重建 target KD-tree（initCompute 檢查 target_cloud_updated_），
  // 若每次 align 都 setInputTarget，等於每個週期重建整張地圖的樹
  m_rough_icp.setInputTarget(m_rough_tgt);
  m_refine_icp.setInputTarget(m_refine_tgt);
  return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr & cloud)
{
  if (m_config.refine_scan_resolution > 0) {
    m_voxel_filter.setLeafSize(
      m_config.refine_scan_resolution, m_config.refine_scan_resolution,
      m_config.refine_scan_resolution);
    m_voxel_filter.setInputCloud(cloud);
    m_voxel_filter.filter(*m_refine_inp);
  } else {
    pcl::copyPointCloud(*cloud, *m_refine_inp);
  }

  if (m_config.rough_scan_resolution > 0) {
    m_voxel_filter.setLeafSize(
      m_config.rough_scan_resolution, m_config.rough_scan_resolution,
      m_config.rough_scan_resolution);
    m_voxel_filter.setInputCloud(cloud);
    m_voxel_filter.filter(*m_rough_inp);
  } else {
    pcl::copyPointCloud(*cloud, *m_rough_inp);
  }
}

bool ICPLocalizer::align(M4F & guess)
{
  // held for the whole alignment so loadMap() cannot swap the targets
  // mid-ICP; worst case a pending swap waits one align (~tens of ms)
  std::lock_guard<std::mutex> lock(m_target_mutex);
  CloudType::Ptr aligned_cloud(new CloudType);

  // coarse-to-fine strategy；target 已在 loadMap() 設好，這裡只換 source，
  // 避免每次 align 觸發 target KD-tree 重建
  if (m_refine_tgt->size() == 0 || m_rough_tgt->size() == 0) return false;

  logPose("guess ", guess);

  m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
  m_rough_icp.setInputSource(m_rough_inp);
  m_rough_icp.align(*aligned_cloud, guess);
  double rough_score = m_rough_icp.getFitnessScore();
  if (!m_rough_icp.hasConverged() || rough_score > m_config.rough_score_thresh) return false;
  m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
  m_refine_icp.setInputSource(m_refine_inp);
  m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
  double refine_score = m_refine_icp.getFitnessScore();
  if (!m_refine_icp.hasConverged() || refine_score > m_config.refine_score_thresh) return false;
  guess = m_refine_icp.getFinalTransformation();

  return true;
}