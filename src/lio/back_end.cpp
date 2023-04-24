#include "poseEstimator/poseEstimator.h"
#include "utils/visualizor.hpp"
#include "lio_sam/keyFrame.h"

ros::Publisher pubLaserOdometryIncremental;
ros::Publisher pubGlobalCloud;
ros::Publisher pubGlobalpath;
PoseEstimator* poseEstimatorPtr;
MapManager* mapManagerPtr;
std::shared_ptr<spdlog::logger> logger;

void laserKeyFrameHandler(const lio_sam::keyFrame::ConstPtr& keyFrameMsg) {
    // logger->info("received one frame");
    std::shared_ptr<KeyFrame> thisKeyFrame(new KeyFrame());
    thisKeyFrame->time = keyFrameMsg->header.stamp.toSec();
    thisKeyFrame->x = keyFrameMsg->initialGuessX;
    thisKeyFrame->y = keyFrameMsg->initialGuessY;
    thisKeyFrame->z = keyFrameMsg->initialGuessZ;
    thisKeyFrame->roll = keyFrameMsg->initialGuessRoll;
    thisKeyFrame->pitch = keyFrameMsg->initialGuessPitch;
    thisKeyFrame->yaw = keyFrameMsg->initialGuessYaw;
    thisKeyFrame->imuAvailable = keyFrameMsg->imuAvailable;
    thisKeyFrame->odomAvailable = keyFrameMsg->odomAvailable;
    thisKeyFrame->imuRollInit = keyFrameMsg->imuRollInit;
    thisKeyFrame->imuPitchInit = keyFrameMsg->imuPitchInit;
    thisKeyFrame->imuYawInit = keyFrameMsg->imuYawInit;
    pcl::fromROSMsg(keyFrameMsg->cloud_corner, *thisKeyFrame->cornerCloudDS);
    pcl::fromROSMsg(keyFrameMsg->cloud_surface, *thisKeyFrame->surfCloudDS);
    pcl::fromROSMsg(keyFrameMsg->cloud_raw, *thisKeyFrame->rawCloudDS);

    poseEstimatorPtr->estimate(*thisKeyFrame);
    if(poseEstimatorPtr->propagateIMUFlag) {
        nav_msgs::Odometry odomIncre = poseEstimatorPtr->propagateIMU(*thisKeyFrame);
        pubLaserOdometryIncremental.publish(odomIncre);
    }    
    // correctPoses (loop closure thread)

    // pubOdom and frames
    poseEstimatorPtr->publishOdometry(pubGlobalpath);
}

void visualizationThread() {
    ros::Rate rate(2);
    while (ros::ok()) {
        lio_sam::publishGlobalMap(*mapManagerPtr, pubGlobalCloud);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapOptimization");
    ros::NodeHandle nh;
    ros::Subscriber subCloudInfo;
    logger = spdlog::stdout_color_mt("console1"); 
    logger->info("lio sam back end start !");
    poseEstimatorPtr = new PoseEstimator(logger);
    mapManagerPtr = poseEstimatorPtr->mapManager;
    subCloudInfo = nh.subscribe<lio_sam::keyFrame>("lio_sam/feature/key_frame", 1, &laserKeyFrameHandler, ros::TransportHints().tcpNoDelay());
    pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 1);
    pubGlobalCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/global_cloud", 1);
    pubGlobalpath = nh.advertise<nav_msgs::Path>("lio_sam/global_path", 1);

    std::thread thread_visualizor{visualizationThread};
    ros::spin();
    thread_visualizor.join();
    return 0;
}