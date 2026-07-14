#pragma once
#include "commons.h"
#include "point_ekf.h"

// Static IMU initialization only. Unlike fastlio2's IMUProcessor there is no
// undistort(): Point-LIO handles motion by predicting/updating point by point.
class IMUInitializer
{
public:
  IMUInitializer(Config & config, std::shared_ptr<PointEKF> kf);

  bool initialize(SyncPackage & package);

private:
  Config m_config;
  std::shared_ptr<PointEKF> m_kf;
  Vec<IMUData> m_imu_cache;
};
