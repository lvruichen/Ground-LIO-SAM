// 可视化线程，发布全局点云和局部子图点云
#ifndef __VISUALIZOR_HH__
#define __VISUALIZOR_HH__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <mutex>

#include "mapManager/mapManager.h"
#include "mapManager/keyFrame.hpp"
#include "utils/slam_utils.hpp"

namespace lio_sam{

void publishGlobalMap(MapManager& _mapManager, ros::Publisher& _globalMapPublisher) {
    if(_mapManager.cloudKeyPoses3D->empty())
        return;
    
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;

    std::unique_lock<std::mutex> visLock(_mapManager.mapMutex);
    kdtreeGlobalMap->setInputCloud(_mapManager.cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(_mapManager.cloudKeyPoses3D->back(),
    50, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
    visLock.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(_mapManager.cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);

    // downsample near selected key frames
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setLeafSize(0.2, 0.2, 0.2); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    for(auto& pt : globalMapKeyPosesDS->points) {
        kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
        pt.intensity = _mapManager.cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
    }

    // extract visualized and downsampled key frames
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i) {
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        float x = _mapManager.cloudKeyPoses6D->points[thisKeyInd].x;
        float y = _mapManager.cloudKeyPoses6D->points[thisKeyInd].y;
        float z = _mapManager.cloudKeyPoses6D->points[thisKeyInd].z;
        float roll = _mapManager.cloudKeyPoses6D->points[thisKeyInd].roll;
        float pitch = _mapManager.cloudKeyPoses6D->points[thisKeyInd].pitch;
        float yaw = _mapManager.cloudKeyPoses6D->points[thisKeyInd].yaw;
        Eigen::Affine3f transCur = pcl::getTransformation(x, y, z, roll, pitch, yaw);
        pcl::transformPointCloud(*_mapManager.rawCloudKeyFrames[thisKeyInd], *cloudOut, transCur);
        *globalMapKeyFrames += *cloudOut; 
    }

    // downsample visualized points
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(0.2, 0.2, 0.2); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(_globalMapPublisher, globalMapKeyFrames, ros::Time::now(), "map");
}

}


#endif