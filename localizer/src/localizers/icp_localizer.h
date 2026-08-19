#pragma once
#include "commons.h"
#include <filesystem>
#include <mutex>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

// 配準從 pcl::IterativeClosestPoint（point-to-point）換成 small_gicp 的 GICP。
// RegistrationPCL 繼承自 pcl::Registration，setInputTarget / setInputSource /
// setMaximumIterations / setMaxCorrespondenceDistance / align / hasConverged /
// getFinalTransformation / getFitnessScore 都在，所以 align() 的流程沒有改動。
// 換掉的理由是收斂品質而不是速度（update_hz 只有 1）：MID360 的 scan 相對地圖
// 稀疏且分布不均，point-to-point 在長走廊 / 大面牆這種幾何退化方向容易沿著牆滑，
// GICP 的 distribution-to-distribution 用局部協方差把那個方向壓住；對 scan 與
// map 兩邊 voxel 解析度不一致（0.25/0.1）的容忍度也高很多。
// 標頭全 header-only（pcl_registration.hpp 末尾自己 include impl），連結
// small_gicp::small_gicp 只是為了拿 include path 與 OpenMP 旗標。
#include <small_gicp/pcl/pcl_registration.hpp>

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    // small_gicp 的 converged 是「更新量小於 eps」才成立，撞到 max_iteration
    // 就停算 not converged（PCL 的 ICP 是撞到上限也回報 converged）；而
    // align() 把 hasConverged() 當硬性條件，迭代數給太少會變成分數明明夠好卻
    // 一路 return false、TF 永遠不更新。所以這裡對齊 small_gicp 自己的預設 20
    int refine_max_iteration = 20;
    double refine_max_corr_dist = 0.5;
    // "GICP" 或 "VGICP"。VGICP 對 target 建 voxelmap 而不是 KD-tree，收斂盆地
    // 更大，適合 rough 這段吸收 relocalize 手動 guess 的誤差；預設兩段都先維持
    // GICP，一次只換一個變數，確認 GICP 在場地裡的表現後再考慮 rough 開 VGICP
    std::string refine_registration_type = "GICP";
    // 只有 registration_type 是 VGICP 時才會用到
    double refine_voxel_resolution = 0.5;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 20;
    double rough_max_corr_dist = 2.0;
    std::string rough_registration_type = "GICP";
    double rough_voxel_resolution = 1.0;

    // GICP 的協方差估計與 reduction 都吃這個執行緒數。localizer 跟 pointlio
    // 共用同一顆 CPU，不要開滿
    int num_threads = 4;
    // 估每個點的局部協方差時取的鄰居數（等同 pcl::GICP 的 correspondence
    // randomness）。小於 5 會被 small_gicp 夾回 5
    int num_neighbors = 20;
};

class ICPLocalizer
{
public:
    ICPLocalizer(const ICPConfig &config);

    bool loadMap(const std::string &path);

    void setInput(const CloudType::Ptr &cloud);

    bool align(M4F &guess);
    ICPConfig &config() { return m_config; }
    CloudType::Ptr roughMap()
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        return m_rough_tgt;
    }
    CloudType::Ptr refineMap()
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        return m_refine_tgt;
    }
    // 地圖只會透過 relocalize 的 loadMap() 進來；載入前 align() 一律失敗，
    // initialpose 等來源可先用這個判斷，避免無聲吞掉 guess
    bool isMapLoaded()
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        return !m_refine_tgt->empty() && !m_rough_tgt->empty();
    }


private:
    ICPConfig m_config;
    // guards m_rough_tgt / m_refine_tgt: loadMap() may run on the service
    // thread while align() runs on the timer thread
    std::mutex m_target_mutex;
    pcl::VoxelGrid<PointType> m_voxel_filter;
    small_gicp::RegistrationPCL<PointType, PointType> m_refine_icp;
    small_gicp::RegistrationPCL<PointType, PointType> m_rough_icp;
    CloudType::Ptr m_refine_inp;
    CloudType::Ptr m_rough_inp;
    CloudType::Ptr m_refine_tgt;
    CloudType::Ptr m_rough_tgt;
    std::string m_pcd_path;
};
