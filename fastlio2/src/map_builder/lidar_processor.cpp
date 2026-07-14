#include "lidar_processor.h"

LidarProcessor::LidarProcessor(Config & config, std::shared_ptr<IESKF> kf)
: m_config(config), m_kf(kf)
{
  m_ikdtree = std::make_shared<KD_TREE<PointType>>();
  m_ikdtree->set_downsample_param(m_config.map_resolution);
  m_cloud_down_lidar.reset(new CloudType);
  m_cloud_down_world.reset(new CloudType(10000, 1));
  m_norm_vec.reset(new CloudType(10000, 1));
  m_effect_cloud_lidar.reset(new CloudType(10000, 1));
  m_effect_norm_vec.reset(new CloudType(10000, 1));
  m_nearest_points.resize(10000);
  m_point_selected_flag.resize(10000, false);

  if (m_config.scan_resolution > 0.0) {
    m_scan_filter.setLeafSize(
      m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
  }

  m_kf->setLossFunction([&](State & s, SharedState & d) { updateLossFunc(s, d); });
  m_kf->setStopFunction([&](const V21D & delta) -> bool {
    V3D rot_delta = delta.block<3, 1>(0, 0);
    V3D t_delta = delta.block<3, 1>(3, 0);
    return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015);
  });
}

void LidarProcessor::trimCloudMap()
{
  m_local_map.cub_to_rm.clear();
  const State & state = m_kf->x();
  // 計算雷達現在在「世界」的哪裡，局部地圖是以 lidar 為中心。
  Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

  if (!m_local_map.initialed) {
    for (int i = 0; i < 3; i++) {
      m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
      m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
    }
    m_local_map.initialed = true;
    return;  // ← 第一次只定位立方體,不刪任何點
  }

  // 以機器人為中心,往三個軸各撐 ±150m,建出 300³ 的立方體。
  float dist_to_map_edge[3][2];
  bool need_move = false;
  // 檢查是不是快出界了
  double det_thresh = m_config.move_thresh * m_config.det_range;
  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

    // 只要機器人離立方體的任何一面牆進到90m以內，就準備把整個立方體網那個方向推
    if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
      need_move = true;
  }
  if (!need_move) return;

  // 平移立方體,並標記要刪掉的「尾巴」
  BoxPointType new_corner, temp_corner;
  new_corner = m_local_map.local_map_corner;
  float mov_dist = std::max(
    (m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9,
    double(m_config.det_range * (m_config.move_thresh - 1)));

  for (int i = 0; i < 3; i++) {
    temp_corner = m_local_map.local_map_corner;
    if (dist_to_map_edge[i][0] <= det_thresh) {
      new_corner.vertex_max[i] -= mov_dist;
      new_corner.vertex_min[i] -= mov_dist;
      temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
      m_local_map.cub_to_rm.push_back(temp_corner);
    } else if (dist_to_map_edge[i][1] <= det_thresh) {
      new_corner.vertex_max[i] += mov_dist;
      new_corner.vertex_min[i] += mov_dist;
      temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
      m_local_map.cub_to_rm.push_back(temp_corner);
    }
  }

  // 真正執行刪除
  m_local_map.local_map_corner = new_corner;  // 立方體邊界更新

  PointVec points_history;
  // ikd-Tree 記帳用(取出之前被移除的點)
  m_ikdtree->acquire_removed_points(points_history);

  // 删除局部地圖之外的點雲
  if (m_local_map.cub_to_rm.size() > 0) m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);
  return;
}

void LidarProcessor::incrCloudMap()
{
  if (m_cloud_down_lidar->empty()) return;
  const State & state = m_kf->x();
  int size = m_cloud_down_lidar->size();
  PointVec point_to_add;
  PointVec point_no_need_downsample;
  for (int i = 0; i < size; i++) {
    const PointType & p = m_cloud_down_lidar->points[i];
    Eigen::Vector3d point(p.x, p.y, p.z);
    point = state.r_wi * (state.r_il * point + state.t_il) + state.t_wi;
    m_cloud_down_world->points[i].x = point(0);
    m_cloud_down_world->points[i].y = point(1);
    m_cloud_down_world->points[i].z = point(2);
    m_cloud_down_world->points[i].intensity = m_cloud_down_lidar->points[i].intensity;
    // 如果该点附近没有近邻点则需要添加到地图中
    if (m_nearest_points[i].empty()) {
      point_to_add.push_back(m_cloud_down_world->points[i]);
      continue;
    }

    const PointVec & points_near = m_nearest_points[i];
    bool need_add = true;
    PointType downsample_result, mid_point;
    mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) *
                    m_config.map_resolution +
                  0.5 * m_config.map_resolution;
    mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) *
                    m_config.map_resolution +
                  0.5 * m_config.map_resolution;
    mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) *
                    m_config.map_resolution +
                  0.5 * m_config.map_resolution;

    // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
    if (
      fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution &&
      fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution &&
      fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution) {
      point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
      continue;
    }
    float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

    for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++) {
      // 如果该点的近邻点较少，则需要加入到地图中
      if (points_near.size() < static_cast<size_t>(m_config.near_search_num)) break;
      // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
      if (sq_dist(points_near[readd_i], mid_point) < dist) {
        need_add = false;
        break;
      }
    }
    if (need_add) point_to_add.push_back(m_cloud_down_world->points[i]);
  }
  m_ikdtree->Add_Points(point_to_add, true);
  m_ikdtree->Add_Points(point_no_need_downsample, false);
}

void LidarProcessor::initCloudMap(PointVec & point_vec)
{
  m_ikdtree->Build(point_vec);
}

void LidarProcessor::process(SyncPackage & package)
{
  // m_kf->setLossFunction([&](State &s, SharedState &d)
  //                       { updateLossFunc(s, d); });
  // m_kf->setStopFunction([&](const V21D &delta) -> bool
  //                       { V3D rot_delta = delta.block<3, 1>(0, 0);
  //                         V3D t_delta = delta.block<3, 1>(3, 0);
  //                         return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
  if (m_config.scan_resolution > 0.0) {
    // 將該幀點雲進行 downsample
    m_scan_filter.setInputCloud(package.cloud);
    m_scan_filter.filter(
      *m_cloud_down_lidar);  // 進行 voxel filter (變成 scarce matrix)，把空間切成 scan_resolution 的小方格，每格只留一個代表點。
  } else {
    pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
  }

  // 移動「局部地圖window」，把太遠的舊地圖丟掉
  // 主要問題是： 地圖不能無限成長，不然 ikd-Tree 會越查越慢、記憶體爆掉。
  // fast-lio2 的解法是只維護機器人週為一個大立方體(邊長 cube_len)的「局部地圖」。
  trimCloudMap();
  // scan-to-map 迭代優化，把 IMU 預測的姿態用 lidar 進行修正。
  // update function 收斂後得到 1. 修正後的最佳姿態估計 2. 收斂後的後線不確定度
  m_kf->update();
  // 用新姿態投到世界 → 判斷這點值不值得加 → 加進 ikd-Tree
  incrCloudMap();
}

void LidarProcessor::updateLossFunc(State & state, SharedState & share_data)
{
  int size = m_cloud_down_lidar->size();
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  for (int i = 0; i < size; i++) {
    PointType & point_body = m_cloud_down_lidar->points[i];
    PointType & point_world = m_cloud_down_world->points[i];
    Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
    Eigen::Vector3d point_world_vec =
      state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
    point_world.x = point_world_vec(0);
    point_world.y = point_world_vec(1);
    point_world.z = point_world_vec(2);
    point_world.intensity = point_body.intensity;
    std::vector<float> point_sq_dist(m_config.near_search_num);
    auto & points_near = m_nearest_points[i];
    m_ikdtree->Nearest_Search(point_world, m_config.near_search_num, points_near, point_sq_dist);
    if (
      points_near.size() >= static_cast<size_t>(m_config.near_search_num) &&
      point_sq_dist[m_config.near_search_num - 1] <= 5)
      m_point_selected_flag[i] = true;
    else
      m_point_selected_flag[i] = false;
    if (!m_point_selected_flag[i]) continue;

    Eigen::Vector4d pabcd;
    m_point_selected_flag[i] = false;
    if (esti_plane(points_near, 0.1, pabcd)) {
      double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) +
                   pabcd(2) * point_world_vec(2) + pabcd(3);
      double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
      if (s > 0.9) {
        m_point_selected_flag[i] = true;
        m_norm_vec->points[i].x = pabcd(0);
        m_norm_vec->points[i].y = pabcd(1);
        m_norm_vec->points[i].z = pabcd(2);
        m_norm_vec->points[i].intensity = pd2;
      }
    }
  }

  int effect_feat_num = 0;
  for (int i = 0; i < size; i++) {
    if (!m_point_selected_flag[i]) continue;
    m_effect_cloud_lidar->points[effect_feat_num] = m_cloud_down_lidar->points[i];
    m_effect_norm_vec->points[effect_feat_num] = m_norm_vec->points[i];
    effect_feat_num++;
  }
  if (effect_feat_num < 1) {
    share_data.valid = false;
    std::cerr << "NO Effective Points!" << std::endl;
    return;
  }
  share_data.valid = true;
  share_data.H.setZero();
  share_data.b.setZero();
  Eigen::Matrix<double, 1, 12> J;
  for (int i = 0; i < effect_feat_num; i++) {
    J.setZero();
    const PointType & laser_p = m_effect_cloud_lidar->points[i];
    const PointType & norm_p = m_effect_norm_vec->points[i];
    Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
    Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
    Eigen::Matrix<double, 1, 3> B =
      -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_wi);
    J.block<1, 3>(0, 0) = B;
    J.block<1, 3>(0, 3) = norm_vec.transpose();
    if (m_config.esti_il) {
      Eigen::Matrix<double, 1, 3> C =
        -norm_vec.transpose() * state.r_wi * state.r_il * Sophus::SO3d::hat(laser_p_vec);
      Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.r_wi;
      J.block<1, 3>(0, 6) = C;
      J.block<1, 3>(0, 9) = D;
    }
    share_data.H += J.transpose() * m_config.lidar_cov_inv * J;
    share_data.b += J.transpose() * m_config.lidar_cov_inv * norm_p.intensity;
  }
}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D & r, const V3D & t)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = r.cast<float>();
  transform.block<3, 1>(0, 3) = t.cast<float>();
  CloudType::Ptr ret(new CloudType);
  pcl::transformPointCloud(*inp, *ret, transform);
  return ret;
}