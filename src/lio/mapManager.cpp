#include "mapManager/mapManager.h"

MapManager::MapManager() {
    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterICP.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurroundingKeyPoses.setLeafSize(1, 1, 1);

    allocateMemory();
}

MapManager::~MapManager() {

}

void MapManager::allocateMemory() {
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
}

// save current cloud and pose, including feature points and 6d pose
void MapManager::saveKeyFrame(KeyFrame& _keyFrame) {
    PointType thisPose3D;
    PointTypePose thisPose6D;
    thisPose3D.x = _keyFrame.x;
    thisPose3D.y = _keyFrame.y;
    thisPose3D.z = _keyFrame.z;
    thisPose3D.intensity = cloudKeyPoses3D->size();
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose6D.intensity;
    thisPose6D.roll = _keyFrame.roll;
    thisPose6D.pitch = _keyFrame.pitch;
    thisPose6D.yaw = _keyFrame.yaw;
    thisPose6D.time = _keyFrame.time;
    cloudKeyPoses6D->push_back(thisPose6D);

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    
    pcl::copyPointCloud(*_keyFrame.cornerCloudDS, *thisCornerKeyFrame);
    pcl::copyPointCloud(*_keyFrame.surfCloudDS, *thisSurfKeyFrame);

    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    cornerCloudKeyFrames.push_back(thisSurfKeyFrame);
}

bool MapManager::extractSurroundingKeyFrames(KeyFrame& _keyFrame) {
    if (cloudKeyPoses3D->empty())
        return false;
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), 50.0, pointSearchInd, pointSearchSqDis);
    for (int i = 0; i < (int)pointSearchInd.size(); ++i){
        int id = pointSearchInd[i];
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
    }

    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    for (int i = 0; i < surroundingKeyPoses->size(); ++i) {
        if (pointDistance(surroundingKeyPoses->points[i], cloudKeyPoses3D->back()) > 50.0)
            continue;
        int thisKeyInd = (int)surroundingKeyPoses->points[i].intensity;
        // 这里每次都重新加入现有的点云会是一个比较耗时的操作
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) {
            *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
            *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
        }
        else {
            // transformed cloud not available
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
            *laserCloudCornerFromMap += laserCloudCornerTemp;
            *laserCloudSurfFromMap   += laserCloudSurfTemp;
            laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            
            downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
            downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);

            // Downsample the surrounding surf key frames (or map)
            downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
            downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);

            // clear map cache if too large
            if (laserCloudMapContainer.size() > 1000)
                laserCloudMapContainer.clear();
        }
    }
}
