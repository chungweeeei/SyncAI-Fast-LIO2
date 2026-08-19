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

// log 用：印出這一段配準的收斂狀況。fitness 是 pcl::Registration 的
// getFitnessScore()（配準後 source 對 target 的最近鄰平均平方距離，單位 m²），
// 跟演算法無關，所以換成 GICP 之後 *_score_thresh 的物理意義沒變；error /
// num_inliers 則是 small_gicp 自己的 Mahalanobis 殘差與內點數，用來判斷門檻
// 卡住時到底是配不上還是根本沒有足夠內點
void logResult(const char * tag, const small_gicp::RegistrationResult & r, double fitness)
{
  RCLCPP_INFO(
    rclcpp::get_logger("icp_localizer"),
    "%s: converged=%s iters=%zu inliers=%zu error=%.4f fitness=%.4f", tag,
    r.converged ? "yes" : "no", r.iterations, r.num_inliers, r.error, fitness);
}
}  // namespace

ICPLocalizer::ICPLocalizer(const ICPConfig & config) : m_config(config)
{
  m_refine_inp.reset(new CloudType);
  m_refine_tgt.reset(new CloudType);
  m_rough_inp.reset(new CloudType);
  m_rough_tgt.reset(new CloudType);

  // setNumThreads() 一定要在第一次 setInputTarget()（loadMap）之前設定：
  // RegistrationPCL 是在 setInputTarget 當下就用當時的 num_threads_ 建
  // KdTreeBuilderOMP，之後再改不會重建那棵樹
  m_rough_icp.setNumThreads(m_config.num_threads);
  m_refine_icp.setNumThreads(m_config.num_threads);
  m_rough_icp.setCorrespondenceRandomness(m_config.num_neighbors);
  m_refine_icp.setCorrespondenceRandomness(m_config.num_neighbors);
  m_rough_icp.setRegistrationType(m_config.rough_registration_type);
  m_refine_icp.setRegistrationType(m_config.refine_registration_type);
  m_rough_icp.setVoxelResolution(m_config.rough_voxel_resolution);
  m_refine_icp.setVoxelResolution(m_config.refine_voxel_resolution);

  // 沒設的話 RegistrationPCL 的預設是 1000 m，等於沒有上限：guess 偏差大時
  // scan 每個點都會硬配到地圖上某個點，遠處錯誤配對一起參與最佳化，有收斂到
  // 錯誤位姿的風險。rough 給大一點的收斂範圍吸收 relocalize 手動 guess 的
  // 誤差，refine 收緊做精配
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
  // target 只在這裡設一次。整張地圖會被建成兩棵樹：RegistrationPCL 在
  // setInputTarget 當下建自己的 small_gicp KdTree（多執行緒，配準用），
  // pcl::Registration 則在之後第一次 align 建它的 FLANN 樹（initCompute 檢查
  // target_cloud_updated_，只有 getFitnessScore 會用到）。兩棵都只建一次，
  // 若每次 align 都 setInputTarget，等於每個週期重建整張地圖的樹兩遍。
  // 另外 target 的 GICP 協方差也是在第一次 align 對整張圖估一次後快取起來
  m_rough_icp.setInputTarget(m_rough_tgt);
  m_refine_icp.setInputTarget(m_refine_tgt);
  return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr & cloud)
{
  // 每一輪都配一份新的 cloud，不能沿用同一個 Ptr 把新 scan filter 進去：
  // RegistrationPCL::setInputSource() 第一行是 `if (input_ == cloud) return;`，
  // 指標相同就整個跳過——source 的 KdTree 不重建，上一輪的 source covariance
  // 也不會清掉（computeTransformation 只在 size 對不上時才重估），於是 GICP
  // 會拿舊 scan 的協方差去配新 scan。pcl::IterativeClosestPoint 沒有這個提前
  // 返回，所以原本重用 buffer 的寫法在它底下是安全的
  m_refine_inp.reset(new CloudType);
  m_rough_inp.reset(new CloudType);

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
  // 兩段都在門檻判斷前先印：換 GICP 之後 hasConverged() 從「幾乎恆真」變成
  // 真的會擋下來（見 ICPConfig::refine_max_iteration 的說明），定位卡住時要
  // 能一眼看出是 converged 沒過還是 score 沒過
  logResult("rough ", m_rough_icp.getRegistrationResult(), rough_score);
  if (!m_rough_icp.hasConverged() || rough_score > m_config.rough_score_thresh) return false;
  m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
  m_refine_icp.setInputSource(m_refine_inp);
  m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
  double refine_score = m_refine_icp.getFitnessScore();
  logResult("refine", m_refine_icp.getRegistrationResult(), refine_score);
  if (!m_refine_icp.hasConverged() || refine_score > m_config.refine_score_thresh) return false;
  guess = m_refine_icp.getFinalTransformation();

  return true;
}