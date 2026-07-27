#pragma once
#include "commons.h"
#include <filesystem>
#include <mutex>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    int refine_max_iteration = 10;
    double refine_max_corr_dist = 0.5;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 5;
    double rough_max_corr_dist = 2.0;
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
    pcl::IterativeClosestPoint<PointType, PointType> m_refine_icp;
    pcl::IterativeClosestPoint<PointType, PointType> m_rough_icp;
    CloudType::Ptr m_refine_inp;
    CloudType::Ptr m_rough_inp;
    CloudType::Ptr m_refine_tgt;
    CloudType::Ptr m_rough_tgt;
    std::string m_pcd_path;
};