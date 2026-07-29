#pragma once
#include "imu_initializer.h"
#include "lidar_processor.h"

enum BuilderStatus
{
    IMU_INIT,       // 累積夠 IMU
    MAP_INIT,       // 建立完初始地圖
    MAPPING         
};

class MapBuilder
{
public:
    MapBuilder(Config &config, std::shared_ptr<PointEKF> kf);

    void process(SyncPackage &package);
    BuilderStatus status() { return m_status; }
    std::shared_ptr<LidarProcessor> lidar_processor() { return m_lidar_processor; }

private:
    // 一筆 IMU 事件: predict 到該時刻 + 以 IMU 作為量測更新 (output model)
    void processIMU(const IMUData &imu);
    // 把 nominal state 往前推到 time (covariance 維持 IMU 頻率傳播)
    void predictTo(double time);

    Config m_config;
    BuilderStatus m_status;
    std::shared_ptr<PointEKF> m_kf;
    std::shared_ptr<IMUInitializer> m_imu_initializer;
    std::shared_ptr<LidarProcessor> m_lidar_processor;
    Vec<PointGroup> m_groups;
    double m_last_state_time = -1.0;
    double m_last_cov_time = -1.0;
};
