#include "imu_initializer.h"

IMUInitializer::IMUInitializer(Config & config, std::shared_ptr<PointEKF> kf)
: m_config(config), m_kf(kf)
{
  m_imu_cache.clear();
}

bool IMUInitializer::initialize(SyncPackage & package)
{
  m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());

  // 累積足夠的 IMU sample
  if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num)) return false;

  // 算平均
  V3D acc_mean = V3D::Zero();
  V3D gyro_mean = V3D::Zero();
  for (const auto & imu : m_imu_cache) {
    acc_mean += imu.acc;
    gyro_mean += imu.gyro;
  }
  acc_mean /= static_cast<double>(m_imu_cache.size());
  gyro_mean /= static_cast<double>(m_imu_cache.size());

  // 設定外參
  m_kf->x().r_il = m_config.r_il;  // imu -> lidar 的旋轉
  m_kf->x().t_il = m_config.t_il;  // imu -> lidar 的平移
  m_kf->x().bg = gyro_mean;
  if (m_config.gravity_align) {
    m_kf->x().r_wi =
      (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
    m_kf->x().initGravityDir(V3D(0, 0, -1.0));
  } else
    m_kf->x().initGravityDir(-acc_mean);

  // output model 的 omg/acc 狀態: 靜置時 omg ~ 0, acc(比力) = -R^T g
  m_kf->x().omg = gyro_mean - m_kf->x().bg;
  m_kf->x().acc = -m_kf->x().r_wi.transpose() * m_kf->x().g;

  m_kf->P().setIdentity();
  m_kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
  m_kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
  m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
  m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;
  m_kf->P().block<3, 3>(21, 21) = M3D::Identity() * 0.0001;
  m_kf->P().block<3, 3>(24, 24) = M3D::Identity() * 0.01;
  m_kf->P().block<3, 3>(27, 27) = M3D::Identity() * 0.01;
  return true;
}
