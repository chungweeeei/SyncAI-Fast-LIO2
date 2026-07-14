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
  m_kf->predictState(imu.time - m_last_state_time);
  m_kf->predictCov(imu.time - m_last_cov_time);
  m_kf->updateIMU(imu.gyro, imu.acc);
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
    predictTo(package.cloud_end_time);
    CloudType::Ptr cloud_world = LidarProcessor::transformCloud(
      package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
    m_lidar_processor->initCloudMap(cloud_world->points);
    m_status = BuilderStatus::MAPPING;
    return;
  }

  // MAPPING: point-by-point 事件迴圈。IMU 與點群兩個序列都已依時間排序，
  // 雙游標依時間先後處理: IMU -> predict + IMU 量測更新; 點群 -> predict +
  // point-to-plane 更新 (Point-LIO output model, 不做 scan 去畸變)。
  m_lidar_processor->preprocess(package, m_groups);
  m_lidar_processor->trimCloudMap();

  size_t imu_idx = 0;
  for (const auto & group : m_groups) {
    while (imu_idx < package.imus.size() && package.imus[imu_idx].time <= group.time) {
      processIMU(package.imus[imu_idx]);
      imu_idx++;
    }
    predictTo(group.time);
    m_lidar_processor->processGroup(group);
  }
  while (imu_idx < package.imus.size()) {
    processIMU(package.imus[imu_idx]);
    imu_idx++;
  }
  predictTo(package.cloud_end_time);

  m_lidar_processor->incrCloudMap();
}
