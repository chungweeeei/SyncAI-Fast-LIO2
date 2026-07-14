#include "map_builder.h"
MapBuilder::MapBuilder(Config & config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
  m_imu_processor = std::make_shared<IMUProcessor>(config, kf);
  m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
  m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::process(SyncPackage & package)
{
  if (m_status == BuilderStatus::IMU_INIT) {
    // 初始化所有 IMU 需要的參數
    if (m_imu_processor->initialize(package)) m_status = BuilderStatus::MAP_INIT;
    return;
  }

  m_imu_processor->undistort(package);

  // 第一幀並沒有地圖可以進行 matching
  if (m_status == BuilderStatus::MAP_INIT) {
    CloudType::Ptr cloud_world = LidarProcessor::transformCloud(
      package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
    m_lidar_processor->initCloudMap(cloud_world->points);
    m_status = BuilderStatus::MAPPING;
    return;
  }

  // 正常運行MAPPING時, lidar_processor->process 做的事 scan-to-map: 拿當前這幀undistorted後的點雲，去跟 ikd-Tree 裡已存在的地圖做 point-to-plan 匹配。
  m_lidar_processor->process(package);
}