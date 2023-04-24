// 负责对接ros，接受ros的原始消息，然后自身维护多个线程的类
#include <sensor_msgs/Imu.h>
#include <deque>
#include <mutex>
#include <thread>

#include "featureExtractor/featureExtractor.h"
#include "imuIntegrator/imuIntegrator.h"
#include "utils/tictoc.hpp"
#include "utils/slam_utils.hpp"
#include "utils/visualizor.hpp"

// test
#include "lio_sam/keyFrame.h"


using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using std::cout;
using std::endl;

ros::Publisher pubCornerCloud;
ros::Publisher pubSurfCloud;
ros::Publisher pubKeyFrameInfo;
ros::Publisher pubImuOdometry;


FeatureExtractor* featureExtractorPtr;
IMUPreIntegrator* imuIntegratorPtr;

std::shared_ptr<spdlog::logger> logger;

std::mutex lidarQueMutex;
std::mutex imuQueMutex;
std::mutex imuInitMutex;
std::mutex odomIncreMutex;
std::mutex keyFrameMutex;

std::deque<sensor_msgs::PointCloud2ConstPtr> lidarMsgQueue;
std::deque<sensor_msgs::Imu> imuMsgQueue;
std::deque<sensor_msgs::Imu> imuInitQueue;
std::deque<nav_msgs::Odometry> odomIncreQueue;
std::deque<std::shared_ptr<KeyFrame>> keyFrameQueue;

sensor_msgs::PointCloud2 curCloudMsg;
std_msgs::Header curCloudHeader;

nav_msgs::Odometry lidarIncreOdom;
nav_msgs::Odometry imuOdom;

pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterRaw;

bool firstLidarScan{true};
bool newLidarCloud{false};

void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg) {
    std::unique_lock<std::mutex> lidarLock(lidarQueMutex);
    lidarMsgQueue.push_back(laserCloudMsg);
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg) {
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
    std::unique_lock<std::mutex> imuLock(imuQueMutex);
    std::unique_lock<std::mutex> imuInitLock(imuInitMutex);
    imuInitQueue.push_back(thisImu);
    imuMsgQueue.push_back(thisImu);
}

void checkImuAvailable(std::shared_ptr<KeyFrame> keyFrame) {
    keyFrame->imuAvailable = false;
    while (!imuInitQueue.empty() && (imuInitQueue.front().header.stamp.toSec() < keyFrame->time - 0.015)) {
        imuInitQueue.pop_front();
    }
    if (imuInitQueue.empty())
        return;
    for (int i = 0; i < (int)imuInitQueue.size(); ++i) {
        sensor_msgs::Imu thisImuMsg = imuInitQueue[i];
        double currentImuTime = thisImuMsg.header.stamp.toSec();
        if (currentImuTime <= keyFrame->time)
            imuRPY2rosRPY(&thisImuMsg, &keyFrame->imuRollInit, &keyFrame->imuPitchInit, &keyFrame->imuYawInit);
        if(currentImuTime > keyFrame->time + 0.01)
            break; 
    }
    keyFrame->imuAvailable = true;
    return;
}

void checkOdomAvailable(std::shared_ptr<KeyFrame> keyFrame) {
    keyFrame->odomAvailable = false;
    while (!odomIncreQueue.empty() && odomIncreQueue.front().header.stamp.toSec() < keyFrame->time - 0.01) {
        odomIncreQueue.pop_front();
    }
    if(odomIncreQueue.empty())
        return;
    if(odomIncreQueue.front().header.stamp.toSec() > keyFrame->time)
        return;
    nav_msgs::Odometry startOdomMsg;
    for (int i = 0; i < (int)odomIncreQueue.size(); ++i) {
        startOdomMsg = odomIncreQueue[i];
        if (ROS_TIME(&startOdomMsg) < keyFrame->time)
            continue;
        else
            break;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        keyFrame->odomAvailable = true;
        keyFrame->x = startOdomMsg.pose.pose.position.x;
        keyFrame->y = startOdomMsg.pose.pose.position.y;
        keyFrame->z = startOdomMsg.pose.pose.position.z;
        keyFrame->roll = roll;
        keyFrame->pitch = pitch;
        keyFrame->roll = roll;
    }
    keyFrame->odomAvailable = true;
}

void preIntegrationThread() {
    while (ros::ok()) {
        std::unique_lock<std::mutex> imuLock(imuQueMutex);
        while (!imuMsgQueue.empty()) {
            sensor_msgs::Imu thisImu = imuMsgQueue.front();
            // 输入一个imu的消息，然后返回高频的雷达增量里程计和imu里程计
            std::unique_lock<std::mutex> odomLock(odomIncreMutex);
            imuIntegratorPtr->pushImuMsg(thisImu, odomIncreQueue, pubImuOdometry); 
            odomLock.unlock();
            imuMsgQueue.pop_front();
            static int count = 0;
            count++;
            if (count % 500 == 0 && !odomIncreQueue.empty()) {
                lidarIncreOdom = odomIncreQueue.back();
                logger->info("laserOdometry: {}, {}, {}", lidarIncreOdom.pose.pose.position.x, lidarIncreOdom.pose.pose.position.y, lidarIncreOdom.pose.pose.position.z);
                count = 0;
            }   
        }
    }
}

void lidarIncreOdomHandler(const nav_msgs::Odometry::ConstPtr& msgIn) {
    imuIntegratorPtr->pushOdomIncreMsg(*msgIn);
}

void featureExtractionThread() {
    while (ros::ok()) {
        newLidarCloud = false;
        std::unique_lock<std::mutex> lidarLock(lidarQueMutex);
        if (lidarMsgQueue.size() > 2) {
            curCloudMsg = *lidarMsgQueue.front();
            curCloudHeader = curCloudMsg.header;
            lidarMsgQueue.pop_front();
            newLidarCloud = true;
        }
        lidarLock.unlock();
        if (firstLidarScan) {
            if(!imuInitQueue.empty() && curCloudHeader.stamp.toSec() > imuInitQueue.front().header.stamp.toSec())
                firstLidarScan = false;
        }
        else if (newLidarCloud) {
        CloudType::Ptr cornerCloud;
        CloudType::Ptr surfCloud;
        CloudType::Ptr rawCloud;

        cornerCloud.reset(new CloudType());
        surfCloud.reset(new CloudType());
        rawCloud.reset(new CloudType());

        std::shared_ptr<KeyFrame> keyFrame(new KeyFrame());

        keyFrame->time = curCloudHeader.stamp.toSec();

        pcl::fromROSMsg(curCloudMsg, *rawCloud);
        downSizeFilterRaw.setInputCloud(rawCloud);
        downSizeFilterRaw.filter(*keyFrame->rawCloudDS);
        
        // move函数，后续curCloudMsg为空
        featureExtractorPtr->featureExtract(curCloudMsg, cornerCloud, surfCloud);
        publishCloud(pubCornerCloud, cornerCloud, curCloudHeader.stamp, curCloudHeader.frame_id);
        publishCloud(pubSurfCloud, surfCloud, curCloudHeader.stamp, curCloudHeader.frame_id);
        
        // judge if odom is available for initial guess
        std::unique_lock<std::mutex> odomLock(odomIncreMutex);
        checkOdomAvailable(keyFrame);
        odomLock.unlock();

        // judge if imu is available for initial guess
        std::unique_lock<std::mutex> imuInitLock(imuInitMutex);
        checkImuAvailable(keyFrame);
        imuInitLock.unlock();

        // for mapOptimization
        downSizeFilterCorner.setInputCloud(cornerCloud);
        downSizeFilterCorner.filter(*keyFrame->cornerCloudDS);
        downSizeFilterSurf.setInputCloud(surfCloud);
        downSizeFilterSurf.filter(*keyFrame->surfCloudDS);

        if (keyFrame->imuAvailable == false && keyFrame->odomAvailable == false) {
            logger->warn("no initial msg available !");
        } 

        lio_sam::keyFrame keyFrameMsg;
        sensor_msgs::PointCloud2 cloud_corner;
        sensor_msgs::PointCloud2 cloud_surf;
        sensor_msgs::PointCloud2 cloud_raw;
        pcl::toROSMsg(*keyFrame->cornerCloudDS, cloud_corner);
        pcl::toROSMsg(*keyFrame->surfCloudDS, cloud_surf);
        pcl::toROSMsg(*keyFrame->rawCloudDS, cloud_raw);
        keyFrameMsg.header = curCloudHeader;
        keyFrameMsg.cloud_corner = cloud_corner;
        keyFrameMsg.cloud_surface = cloud_surf;
        keyFrameMsg.cloud_raw = cloud_raw;
        keyFrameMsg.imuAvailable = keyFrame->imuAvailable;
        keyFrameMsg.odomAvailable = keyFrame->odomAvailable;
        keyFrameMsg.imuPitchInit = keyFrame->imuPitchInit;
        keyFrameMsg.imuRollInit = keyFrame->imuRollInit;
        keyFrameMsg.imuYawInit = keyFrame->imuYawInit;
        keyFrameMsg.initialGuessX = keyFrame->x;
        keyFrameMsg.initialGuessY = keyFrame->y;
        keyFrameMsg.initialGuessZ = keyFrame->z;
        keyFrameMsg.initialGuessRoll = keyFrame->roll;
        keyFrameMsg.initialGuessPitch = keyFrame->pitch;
        keyFrameMsg.initialGuessYaw = keyFrame->yaw;
        pubKeyFrameInfo.publish(keyFrameMsg);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ScanRegistration");
    ros::NodeHandle nh;
    ros::Subscriber subImu;
    ros::Subscriber subLidarCloud;
    ros::Subscriber subLidarIncreOdom;
    
    logger = spdlog::stdout_color_mt("console"); 
    logger->info("lio sam front end start !");

    subLidarCloud     = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_points", 5, cloudHandler, ros::TransportHints().tcpNoDelay());
    subImu            = nh.subscribe<sensor_msgs::Imu>("/imu/data", 2000, imuHandler, ros::TransportHints().tcpNoDelay());
    subLidarIncreOdom = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 500, lidarIncreOdomHandler, ros::TransportHints().tcpNoDelay());

    pubCornerCloud    = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/corner_cloud", 1);
    pubSurfCloud      = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/surf_cloud", 1);
    pubKeyFrameInfo   = nh.advertise<lio_sam::keyFrame>("lio_sam/feature/key_frame", 1);
    pubImuOdometry    = nh.advertise<nav_msgs::Odometry>("odometry/imu", 1);

    featureExtractorPtr = new FeatureExtractor(logger);
    imuIntegratorPtr    = new IMUPreIntegrator(logger);
    TransformFusion trans;

    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterRaw.setLeafSize(0.2, 0.2, 0.2);

    std::thread thread_featureExtraction{featureExtractionThread};
    std::thread thread_imuIntegration{preIntegrationThread};
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    thread_featureExtraction.join();
    thread_imuIntegration.join();
    return 0; 
}