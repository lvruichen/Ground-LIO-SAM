#include "poseEstimator/poseEstimator.h"

PoseEstimator::PoseEstimator(std::shared_ptr<spdlog::logger> _logger) {
    this->logger = _logger;
    mapManager = new MapManager();
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    allocateMemory();
}

PoseEstimator::~PoseEstimator() {

}

void PoseEstimator::allocateMemory() {
    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());

    localCornerCloud.reset(new pcl::PointCloud<PointType>());
    localSurfCloud.reset(new pcl::PointCloud<PointType>());
}

void PoseEstimator::estimate(KeyFrame& keyFrame) {
    timeLaserInfoCur = keyFrame.time;
    laserCloudCornerLastDS = keyFrame.cornerCloudDS;
    laserCloudCornerLastDS = keyFrame.surfCloudDS;

    static double timeLastProcessing = -1;
    if(timeLaserInfoCur - timeLastProcessing >= 0.15) {
        timeLastProcessing = timeLaserInfoCur;
        // update initial guess
        updateInitialGuess(keyFrame);
        // logger->info("transformTobeMapped: {}, {}, {}, {}, {}, {}", transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2],
                                                                    // transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
        // extract surrounding keyFrames
        extractSurroundKeyFrames(keyFrame);
        // downSampleCurrentScan

        // scan2MapOptimization

        // saveKeyFramesAndFactor

        // correctPoses
    }
}

void PoseEstimator::updateInitialGuess(KeyFrame& keyFrame) {
    incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);
    static Eigen::Affine3f lastImuTransformation;
    if (mapManager->cloudKeyPoses3D->empty()) {
        transformTobeMapped[0] = keyFrame.imuRollInit;
        transformTobeMapped[1] = keyFrame.imuPitchInit;
        transformTobeMapped[2] = keyFrame.imuYawInit;

        lastImuTransformation = pcl::getTransformation(0, 0, 0, 
                            keyFrame.imuRollInit, keyFrame.imuPitchInit, keyFrame.imuYawInit); // save imu before return;
        return;
    }

    static bool lastImuPreTransAvailable = false;
    static Eigen::Affine3f lastImuPreTransformation;
    if (keyFrame.odomAvailable == true) {
        Eigen::Affine3f transBack = pcl::getTransformation(keyFrame.x, keyFrame.y, keyFrame.z, 
                                                           keyFrame.roll, keyFrame.pitch, keyFrame.yaw);
        if (lastImuPreTransAvailable == false)
        {
            lastImuPreTransformation = transBack;
            lastImuPreTransAvailable = true;
        } else {
            Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuPreTransformation = transBack;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, keyFrame.imuRollInit, keyFrame.imuPitchInit, keyFrame.imuYawInit); // save imu before return;
            return;
        }

        if (keyFrame.imuAvailable == true) {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, keyFrame.imuRollInit, keyFrame.imuPitchInit, keyFrame.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, keyFrame.imuRollInit, keyFrame.imuPitchInit, keyFrame.imuYawInit); // save imu before return;
            return;
        }
    }


}

void PoseEstimator::extractSurroundKeyFrames(KeyFrame& keyFrame) {
    // 调用mapManager
    mapManager->extractSurroundingKeyFrames(keyFrame);
    localCornerCloud = mapManager->laserCloudCornerFromMap;
    localSurfCloud = mapManager->laserCloudSurfFromMap;
}

void PoseEstimator::downsampleCurrentScan(KeyFrame& keyFrame) {
    
}

void PoseEstimator::scan2MapOptimization() {
    
}


