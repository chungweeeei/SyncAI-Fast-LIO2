#include "map_builder.h"

MapBuilder::MapBuilder(Config & config, std::shared_ptr<PointEKF> kf) : m_config(config), m_kf(kf)
{
  m_kf->setConfig(config);
  m_imu_initializer = std::make_shared<IMUInitializer>(config, kf);
  m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
  m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::processIMU(const IMUData & imu)
{
  // 處理一筆 IMU 事件的完整邏輯
  m_kf->predictState(imu.time - m_last_state_time);   // 在 EKF 中, predictState 用的是狀態自己的 omg/acc 透過傳進去的 dt 去預測那個時間下的 omg/acc
                                                      // delta omg => m_x.omg * dt 
                                                      // delta acc => (m_x.r_wi * m_x.acc + m_x.g) * dt;
  m_kf->predictCov(imu.time - m_last_cov_time);       // Covariance matrix 也是
  m_kf->updateIMU(imu.gyro, imu.acc);                 // 把 IMU 讀值當量測值進行估值修正
  m_last_state_time = std::max(imu.time, m_last_state_time);
  m_last_cov_time = std::max(imu.time, m_last_cov_time);
}

void MapBuilder::predictTo(double time)
{
  m_kf->predictState(time - m_last_state_time);
  m_last_state_time = std::max(time, m_last_state_time);
}

void MapBuilder::process(SyncPackage & package)
{
  if (m_status == BuilderStatus::IMU_INIT) {
    if (m_imu_initializer->initialize(package)) {
      // propagation 時鐘從這一幀結束時刻起算
      m_last_state_time = package.cloud_end_time;
      m_last_cov_time = package.cloud_end_time;
      m_status = BuilderStatus::MAP_INIT;
    }
    return;
  }

  if (m_status == BuilderStatus::MAP_INIT) {
    // 這一幀只推進狀態並用結束時刻的 pose 建立初始地圖 (同 fastlio2 的流程)
    for (const auto & imu : package.imus) processIMU(imu);

    // 如果維持現在估計的 omg / acc，那在 cloud_end_time後會在哪、朝哪、跑多快
    predictTo(package.cloud_end_time);

    // 把第一幀點雲投到世界座標也就是 map frame 上
    CloudType::Ptr cloud_world = LidarProcessor::transformCloud(
      package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
    
    // 那這筆第一幀點雲作為地圖的地基
    m_lidar_processor->initCloudMap(cloud_world->points);

    // 切換到正常 mapping 狀態
    m_status = BuilderStatus::MAPPING;
    return;
  }

  // MAPPING: point-by-point 事件迴圈。IMU 與點群兩個序列都已依時間排序，
  // 雙游標依時間先後處理: IMU -> predict + IMU 量測更新; 點群 -> predict +
  // point-to-plane 更新 (Point-LIO output model, 不做 scan 去畸變)。
  m_lidar_processor->preprocess(package, m_groups);
  m_lidar_processor->trimCloudMap();

  // 底下就是兩個已排序按時間合併執行的迴圈
  // pointlio 不需要進行修正主要原因在於：
  // pointlio 這邊進行邊投影邊修正
  // group1 投影 -> update -> 狀態變準了
  // group2 投影 (用修正過的狀態) -> update -> 更準
  // group3 投影 -> ..
  // 每個 group 的位姿都受益於前面所有 group 的修正。誤差不會在一幀的 pointcloud 內累積 
  size_t imu_idx = 0;
  for (const auto & group : m_groups) {
    // 判斷 IMU 的時間有沒有小於點群的時間，如果沒有就直接 predict 到點群的時間，如果有先 processIMU 到 IMU 的時間上，然後再 predict 到點群時間
    while (imu_idx < package.imus.size() && package.imus[imu_idx].time <= group.time) {
      processIMU(package.imus[imu_idx]);
      imu_idx++;
    }
    predictTo(group.time);
    m_lidar_processor->processGroup(group);
  }

  // 最後一筆點群處理完後，可能還有 IMU 還沒被消化完
  while (imu_idx < package.imus.size()) {
    processIMU(package.imus[imu_idx]);
    imu_idx++;
  }
  predictTo(package.cloud_end_time);

  // 降採樣後每個地圖只保留「最靠近voxel中心」的那個點
  // 用增量的方式 - 不重建整張地圖，只判斷新來的點該不該進去
  m_lidar_processor->incrCloudMap();
}
