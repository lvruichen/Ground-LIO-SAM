// input rawcloud and output feature cloud

#ifndef __FEATURE_EXTRACTOR__
#define __FEATURE_EXTRACTOR__
#define PCL_NO_PRECOMPILE 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <memory>
// logger
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// 定义点云的曲率
struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)


// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

class FeatureExtractor{
public:
    FeatureExtractor();

    ~FeatureExtractor();

    void allocateMemory();

    void resetParameters();

    void featureExtract(sensor_msgs::PointCloud2 &msgIn, pcl::PointCloud<pcl::PointXYZI>::Ptr 
    cornerCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr surfCloud);
    
    void calculateSmoothness();
    
    void detectFeaturePoint(pcl::PointCloud<PointType>::Ptr& cloud,
                            std::vector<int>& pointSharp,
                            std::vector<int>& pointFlat);

    void projectCloud();
    
    float pointDistance(PointType p)
    {
        return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    }

    float pointDistance(PointType p1, PointType p2)
    {
        return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
    }

private:
    pcl::PointCloud<PointXYZIRT>::Ptr rawCloud;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    std::vector<pcl::PointCloud<PointType>::Ptr> vlines;
    std::vector<std::vector<int>> vcorner;
    std::vector<std::vector<int>> vsurf;

    std::shared_ptr<spdlog::logger> logger;

    cv::Mat rangeMat;

    int ringFlag{-1};
    float lidarMinRange = 1;
    float lidarMaxRange = 100;
    int N_SCAN = 16;
    int Horizon_SCAN = 1800;

    float edgeThreshold{1.0};
    float surfThreshold{0.1};
};
#endif