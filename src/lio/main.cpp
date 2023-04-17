// 负责对接ros，接受ros的原始消息，然后自身维护多个线程的类

#include "featureExtractor/featureExtractor.h"
#include "imuIntegrator/imuIntegrator.h"
#include "poseEstimator/poseEstimator.h"
#include "utils/tictoc.hpp"
#include "utils/slam_utils.hpp"
#include <sensor_msgs/Imu.h>
#include <queue>
#include <mutex>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using std::cout;
using std::endl;

ros::Publisher pubCornerCloud;
ros::Publisher pubSurfCloud;
ros::Publisher pubLaserCloudInfo;

FeatureExtractor* featureExtractorPtr;
IMUPreIntegrator* imuIntegratorPtr;
PoseEstimator* poseEstimatorPtr;

std::shared_ptr<spdlog::logger> logger;

std::mutex lidarQueMutex;
std::mutex imuQueMutex;
std::mutex imuInitMutex;
std::mutex odomIncreMutex;
std::mutex keyFrameMutex;

std::queue<sensor_msgs::PointCloud2ConstPtr> lidarMsgQueue;
std::queue<sensor_msgs::Imu> imuMsgQueue;
std::queue<sensor_msgs::Imu> imuInitQueue;
std::queue<nav_msgs::Odometry> odomIncreQueue;
std::queue<std::shared_ptr<KeyFrame>> keyFrameQueue;

sensor_msgs::PointCloud2 curCloudMsg;
std_msgs::Header curCloudHeader;

nav_msgs::Odometry lidarIncreOdom;
nav_msgs::Odometry imuOdom;

pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;

bool newLidarCloud{false};
double curLidarTime = -1;

void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg) {
    std::unique_lock<std::mutex> lidarLock(lidarQueMutex);
    lidarMsgQueue.push(laserCloudMsg);
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg) {
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
    std::unique_lock<std::mutex> imuLock(imuQueMutex);
    std::unique_lock<std::mutex> imuInitLock(imuInitMutex);
    imuInitQueue.push(thisImu);
    imuMsgQueue.push(thisImu);
}

void preIntegrationThread() {
    while (ros::ok()) {
        std::unique_lock<std::mutex> imuLock(imuQueMutex);
        if (!imuMsgQueue.empty()) {
            sensor_msgs::Imu thisImu = imuMsgQueue.front();
            // 输入一个imu的消息，然后返回高频的雷达增量里程计和imu里程计
            imuIntegratorPtr->pushImuMsg(thisImu, lidarIncreOdom, imuOdom); 
            std::unique_lock<std::mutex> odomLock(odomIncreMutex);
            odomIncreQueue.push(lidarIncreOdom);
            odomLock.unlock();
            imuMsgQueue.pop();
            static int count = 0;
            count++;
            if (count % 200 == 0) {
                logger->info("imu_odom: {}, {}, {}", imuOdom.pose.pose.position.x, imuOdom.pose.pose.position.y, imuOdom.pose.pose.position.z);
                logger->info("laserOdometry: {}, {}, {}", lidarIncreOdom.pose.pose.position.x, lidarIncreOdom.pose.pose.position.y, lidarIncreOdom.pose.pose.position.z);
                count = 0;
            }   
        }
    }
}

void mapOptimizationThread() {
    while (ros::ok()) {
        common::TicToc t2;
        std::unique_lock<std::mutex> keyFrameLock(keyFrameMutex);
        if (!keyFrameQueue.empty()) {
            std::shared_ptr<KeyFrame> thisKeyFrame = keyFrameQueue.front();
            keyFrameQueue.pop();
            keyFrameLock.unlock();
            poseEstimatorPtr->estimate(*thisKeyFrame);
            logger->info("mapOptimization cost: {}", t2.toc());
            if (poseEstimatorPtr->propagateIMUFlag == true) {
                poseEstimatorPtr->propagateIMU(imuIntegratorPtr, *thisKeyFrame);
            }            
        }        
    }
}

void featureExtractionThread() {
    while (ros::ok()) {
        // common::TicToc t1;
        newLidarCloud = false;
        std::unique_lock<std::mutex> lidarLock(lidarQueMutex);
        if (!lidarMsgQueue.empty()) {
            curCloudMsg = *lidarMsgQueue.front();
            curLidarTime = curCloudMsg.header.stamp.toSec();
            curCloudHeader = curCloudMsg.header;
            lidarMsgQueue.pop();
            newLidarCloud = true;
        }
        lidarLock.unlock();

        if (newLidarCloud) {
        CloudType::Ptr cornerCloud;
        CloudType::Ptr surfCloud;

        cornerCloud.reset(new CloudType());
        surfCloud.reset(new CloudType());
        
        featureExtractorPtr->featureExtract(curCloudMsg, cornerCloud, surfCloud);
        publishCloud(pubCornerCloud, cornerCloud, curCloudHeader.stamp, curCloudHeader.frame_id);
        publishCloud(pubSurfCloud, surfCloud, curCloudHeader.stamp, curCloudHeader.frame_id);
        
        // judge if odom is available for initial guess
        bool odomAvailable = false;
        nav_msgs::Odometry currentImuOdom;
        std::unique_lock<std::mutex> odomLock(odomIncreMutex);
        while (!odomIncreQueue.empty() && odomIncreQueue.front().header.stamp.toSec() < curLidarTime) {
            odomIncreQueue.pop();
        }
        if (!odomIncreQueue.empty()) {
            odomAvailable = true;
            currentImuOdom = odomIncreQueue.front();
        }
        odomLock.unlock();

        // judge if imu is available for initial guess
        bool imuAvailable = false;
        std::unique_lock<std::mutex> imuInitLock(imuInitMutex);
        sensor_msgs::Imu currentImuMsg;
        while (!imuInitQueue.empty() && imuInitQueue.front().header.stamp.toSec() < curLidarTime) {
            imuInitQueue.pop();
        }
        if (!imuInitQueue.empty()) {
            imuAvailable = true;
            currentImuMsg = imuInitQueue.front();
        }
        imuInitLock.unlock();

        // for mapOptimization
        std::shared_ptr<KeyFrame> keyFrame(new KeyFrame());
        downSizeFilterCorner.setInputCloud(cornerCloud);
        downSizeFilterCorner.filter(*keyFrame->cornerCloudDS);
        downSizeFilterSurf.setInputCloud(surfCloud);
        downSizeFilterSurf.filter(*keyFrame->surfCloudDS);
        keyFrame->time = curCloudHeader.stamp.toSec();

        if(odomAvailable) {
            tf::Quaternion orientation;
            tf::quaternionMsgToTF(currentImuOdom.pose.pose.orientation, orientation);
            double roll, pitch, yaw;
            tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
            keyFrame->odomAvailable = true;
            keyFrame->x = currentImuOdom.pose.pose.position.x;
            keyFrame->y = currentImuOdom.pose.pose.position.y;
            keyFrame->z = currentImuOdom.pose.pose.position.z;
            keyFrame->roll = roll;
            keyFrame->pitch = pitch;
            keyFrame->roll = roll;
        }
        if (imuAvailable) {
            keyFrame->imuAvailable = true;
            imuRPY2rosRPY(&currentImuMsg, &keyFrame->imuRollInit, &keyFrame->imuPitchInit, &keyFrame->imuYawInit);
        }
        std::unique_lock<std::mutex> keyFrameLock(keyFrameMutex);
        keyFrameQueue.push(keyFrame);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ScanRegistration");
    ros::NodeHandle nh;
    ros::Subscriber subImu;
    ros::Subscriber subLidarCloud;
    
    logger = spdlog::stdout_color_mt("console"); 

    subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_points", 5, cloudHandler, ros::TransportHints().tcpNoDelay());
    subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 2000, imuHandler, ros::TransportHints().tcpNoDelay());

    pubCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/corner_cloud", 1);
    pubSurfCloud   = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/surf_cloud", 1);

    featureExtractorPtr = new FeatureExtractor(logger);
    imuIntegratorPtr    = new IMUPreIntegrator(logger);
    poseEstimatorPtr    = new PoseEstimator(logger);

    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.2, 0.2, 0.2);

    std::thread thread_process{featureExtractionThread};
    std::thread thread_imuIntegration{preIntegrationThread};
    std::thread thread_mapOptimization{mapOptimizationThread};

    ros::spin();
    return 0; 
}