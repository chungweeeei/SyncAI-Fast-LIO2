#include "simple_pgo.h"

SimplePGO::SimplePGO(const Config & config) : m_config(config)
{
  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeThreshold = 0.01;
  isam2_params.relinearizeSkip = 1;
  m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
  m_initial_values.clear();
  m_graph.resize(0);
  m_r_offset.setIdentity();
  m_t_offset.setZero();

  m_icp.setMaximumIterations(50);
  m_icp.setMaxCorrespondenceDistance(10);
  m_icp.setTransformationEpsilon(1e-6);
  m_icp.setEuclideanFitnessEpsilon(1e-6);
  m_icp.setRANSACIterations(0);
}

bool SimplePGO::isKeyPose(const PoseWithTime & pose)
{
  // 第一幀 pose 無條收
  if (m_key_poses.size() == 0) return true;

  const KeyPoseWithCloud & last_item = m_key_poses.back();

  // 計算位移差跟角度差
  double delta_trans = (pose.t - last_item.t_local).norm();
  double delta_deg =
    Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;

  // 任意條件超標就加到 key_poses
  if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
    return true;

  return false;
}

bool SimplePGO::addKeyPose(const CloudWithPose & cloud_with_pose)
{
  bool is_key_pose = isKeyPose(cloud_with_pose.pose);
  if (!is_key_pose) return false;

  size_t idx = m_key_poses.size();  // 新節點的編號

  /**
    * m_r_offset / m_t_offset 是 local -> global 的補償量 (上一輪優化後算出來的)
    * 為何需要添加 offset，主要是 graph 裡其他節點都是在 global 坐標系。如果新節點直接用 local 座標當初始值。
    */
  M3D init_r = m_r_offset * cloud_with_pose.pose.r;
  V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;

  // 添加初始值
  m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));

  // 加入 factor
  if (idx == 0) {
    // 第一幀 -> 先驗約束 PriorFactor
    // PriorFactor 是 「絕對約束」：直接宣告「第 0 號節點就應該在這個絕對位置」
    gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
    m_graph.add(
      gtsam::PriorFactor<gtsam::Pose3>(
        idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
  } else {
    // 之後每幀 ->  里程計約束 BetweenFactor
    // BetweenFactor 不說 idx 在哪，只說 從 idx-1 到 idx 的 相對位置應該是這個
    const KeyPoseWithCloud & last_item = m_key_poses.back();

    M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;  // 計算相對旋轉
    V3D t_between =
      last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);  // 計算相對平移
    gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
    m_graph.add(
      gtsam::BetweenFactor<gtsam::Pose3>(
        idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
  }

  // 最後把關鍵幀存起來
  KeyPoseWithCloud item;
  item.time = cloud_with_pose.pose.second;
  item.r_local = cloud_with_pose.pose.r;
  item.t_local = cloud_with_pose.pose.t;
  item.body_cloud = cloud_with_pose.cloud;
  item.r_global = init_r;
  item.t_global = init_t;
  m_key_poses.push_back(item);
  return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
  assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
  int min_idx = std::max(0, idx - half_range);
  int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

  CloudType::Ptr ret(new CloudType);
  for (int i = min_idx; i <= max_idx; i++) {
    CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
    CloudType::Ptr global_cloud(new CloudType);
    pcl::transformPointCloud(
      *body_cloud, *global_cloud, m_key_poses[i].t_global,
      Eigen::Quaterniond(m_key_poses[i].r_global));
    *ret += *global_cloud;
  }
  if (resolution > 0) {
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setLeafSize(resolution, resolution, resolution);
    voxel_grid.setInputCloud(ret);
    voxel_grid.filter(*ret);
  }
  return ret;
}

void SimplePGO::searchForLoopPairs()
{
  // 軌跡還太短，根本不可能「繞回來」。
  if (m_key_poses.size() < 10) return;

  // cool down time
  if (m_config.min_loop_detect_duration > 0.0) {
    if (m_history_pairs.size() > 0) {
      double current_time = m_key_poses.back().time;
      double last_time = m_key_poses[m_history_pairs.back().second].time;
      if (current_time - last_time < m_config.min_loop_detect_duration) return;
    }
  }

  // 找 candidate
  size_t cur_idx = m_key_poses.size() - 1;
  const KeyPoseWithCloud & last_item = m_key_poses.back();
  pcl::PointXYZ last_pose_pt;
  last_pose_pt.x = last_item.t_global(0);
  last_pose_pt.y = last_item.t_global(1);
  last_pose_pt.z = last_item.t_global(2);

  // 除了自己，全部舊幀
  pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < m_key_poses.size() - 1; i++) {
    pcl::PointXYZ pt;
    pt.x = m_key_poses[i].t_global(0);
    pt.y = m_key_poses[i].t_global(1);
    pt.z = m_key_poses[i].t_global(2);
    key_poses_cloud->push_back(pt);
  }

  // 半徑 search
  // {TODO} 這裡可能有效能點：每次呼叫都需要重建整顆 KD-Tree（要把所有歷史關鍵幀重新塞一遍）
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(key_poses_cloud);
  std::vector<int> ids;
  std::vector<float> sqdists;
  int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
  if (neighbors == 0) return;

  // 在「空間近」的候選裡，還要求「時間上相差 > 60 秒」才承認是 loop closure。
  int loop_idx = -1;
  for (size_t i = 0; i < ids.size(); i++) {
    int idx = ids[i];
    // 時間夠遠 → 選它
    if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_tresh) {
      loop_idx = idx;
      break;
    }
  }

  if (loop_idx == -1) return;

  /** 點雲驗證 - ICP 對齊並算出「結」
    * 到這裡只知道「位置近、時間遠」，但還不能信（兩條不同走廊也可能位置巧合地近）。
    * target端： 把舊關鍵幀前各 5 幀（共 11 幀）的點雲，各自用自己的 t_global / r_global 轉換到世界座標後、疊起來、降採樣。
    */
  CloudType::Ptr target_cloud =
    getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
  CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);
  CloudType::Ptr align_cloud(new CloudType);

  m_icp.setInputSource(source_cloud);
  m_icp.setInputTarget(target_cloud);
  m_icp.align(*align_cloud);

  if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_tresh) return;

  // 把 ICP 結果換算成 GTSAM 的 BetweenFactor
  M4F loop_transform = m_icp.getFinalTransformation();  // ICP 算出的修正變換 (在 global 座標下)

  LoopPair one_pair;
  one_pair.source_id = cur_idx;
  one_pair.target_id = loop_idx;
  one_pair.score = m_icp.getFitnessScore();

  // 先把「修正」套到當前幀的 global 位姿上，得到「修正後的當前位姿」
  M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
  V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global +
                  loop_transform.block<3, 1>(0, 3).cast<double>();

  // 再把它換算成「source 相對 target 的相對位姿」
  one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
  one_pair.t_offset =
    m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);

  // 存起來，等smoothAndUpdate加進圖中
  m_cache_pairs.push_back(one_pair);

  // 給 marker & cool down 使用
  m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
}

void SimplePGO::smoothAndUpdate()
{
  bool has_loop = !m_cache_pairs.empty();

  // 添加回環因子
  if (has_loop) {
    for (LoopPair & pair : m_cache_pairs) {
      m_graph.add(
        gtsam::BetweenFactor<gtsam::Pose3>(
          pair.target_id, pair.source_id,
          gtsam::Pose3(gtsam::Rot3(pair.r_offset), gtsam::Point3(pair.t_offset)),
          gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * pair.score)));
    }
    // 清空 cache_pairs
    std::vector<LoopPair>().swap(m_cache_pairs);
  }

  // smooth and mapping
  // iSAM2 -> Incremental Smoothing and Mapping 是增量式的：他內部用Bayes Tree的結構，只重新線性和求解。
  m_isam2->update(m_graph, m_initial_values);
  m_isam2->update();
  if (has_loop) {
    m_isam2->update();
    m_isam2->update();
    m_isam2->update();
    m_isam2->update();
  }
  m_graph.resize(0);
  m_initial_values.clear();

  // 把優化的結果寫回關鍵幀
  gtsam::Values estimate_values = m_isam2->calculateBestEstimate();  // 取最佳估計
  for (size_t i = 0; i < m_key_poses.size(); i++) {
    gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
    m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();  // 改寫 global
    m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
  }

  // 重算 offset 並同步更新 offset
  const KeyPoseWithCloud & last_item = m_key_poses.back();
  m_r_offset = last_item.r_global * last_item.r_local.transpose();
  m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}