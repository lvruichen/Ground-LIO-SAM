// 负责对接ros，接受ros的原始消息，然后自身维护多个线程的类

#include "featureExtractor/featureExtractor.h"
#include "imuIntegrator/imuIntegrator.h"
#include "poseEstimator/poseEstimator.h"
#include <sensor_msgs/Imu.h>
#include <queue>
#include <mutex>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using std::cout;
using std::endl;

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}   

template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

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

bool newLidarCloud{false};
double curLidarTime = -1;

sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in) {
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Matrix3d extRot;
    extRot << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    Eigen::Quaterniond extQRPY = Eigen::Quaterniond(extRot);

    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;   
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }
    return imu_out;
}

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
            imuIntegratorPtr->pushImuMsg(thisImu, lidarIncreOdom, imuOdom); 
            std::unique_lock<std::mutex> odomLock(odomIncreMutex);
            odomIncreQueue.push(lidarIncreOdom);
            odomLock.unlock();
            imuMsgQueue.pop();
            static int count = 0;
            count++;
            if (count % 200 == 0) {
                // logger->info("imu_odom: {}, {}, {}", imuOdom.pose.pose.position.x, imuOdom.pose.pose.position.y, imuOdom.pose.pose.position.z);
                // logger->info("laserOdometry: {}, {}, {}", lidarIncreOdom.pose.pose.position.x, lidarIncreOdom.pose.pose.position.y, lidarIncreOdom.pose.pose.position.z);
                count = 0;
            }   
        }
    }
}

void mapOptimizationThread() {
    while (ros::ok()) {
        std::unique_lock<std::mutex> keyFrameLock(keyFrameMutex);
        if (!keyFrameQueue.empty()) {
            std::shared_ptr<KeyFrame> thisKeyFrame = keyFrameQueue.front();
            keyFrameQueue.pop();
            keyFrameLock.unlock();
            poseEstimatorPtr->estimate(*thisKeyFrame);
            if (poseEstimatorPtr->propagateIMUFlag == true) {
                poseEstimatorPtr->propagateIMU(imuIntegratorPtr, *thisKeyFrame);
            }            
            // logger->info("transformTobeMapped: {}, {}, {}, {}, {}, {}", 
            // poseEstimatorPtr->transformTobeMapped[0], 
            // poseEstimatorPtr->transformTobeMapped[1], 
            // poseEstimatorPtr->transformTobeMapped[2],
            // poseEstimatorPtr->transformTobeMapped[3], 
            // poseEstimatorPtr->transformTobeMapped[4], 
            // poseEstimatorPtr->transformTobeMapped[5]);
        }        
    }
}

void process() {
    while (ros::ok()) {
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
        keyFrame->cornerCloudDS = cornerCloud;
        keyFrame->surfCloudDS   = surfCloud;
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

    std::thread thread_process{process};
    std::thread thread_imuIntegration{preIntegrationThread};
    std::thread thread_mapOptimization{mapOptimizationThread};

    ros::spin();
    return 0; 
}