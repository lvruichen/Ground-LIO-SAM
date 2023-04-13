#include "imuIntegrator/imuIntegrator.h"

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

IMUPreIntegrator::IMUPreIntegrator(std::shared_ptr<spdlog::logger> _logger) {
    this->logger = _logger;
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(9.80511);
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(0.01, 2);
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(0.001, 2); // gyro white noise in continuous
    p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

    priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
    priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
    correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
    noiseModelBetweenBias = (gtsam::Vector(6) << 0.0002, 0.0002, 0.0002, 0.00003, 0.00003, 0.00003).finished();
    
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
}

IMUPreIntegrator::~IMUPreIntegrator() {

}

Eigen::Affine3f odom2affine(const nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

void IMUPreIntegrator::resetOptimization(){
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph newGraphFactors;
    graphFactors = newGraphFactors;

    gtsam::Values NewGraphValues;
    graphValues = NewGraphValues;
}

void IMUPreIntegrator::resetParams() {
    doneFirstOpt = false;
    systemInitialized = false;
}

bool IMUPreIntegrator::failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur) {
    Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
}

void IMUPreIntegrator::getImuBias() const{

}

void IMUPreIntegrator::pushImuMsg(const sensor_msgs::Imu& imuMsg, nav_msgs::Odometry& lidarIncreOdom, nav_msgs::Odometry& imuOdom){
    std::lock_guard<std::mutex> lock1(queImuMutex);
    std::lock_guard<std::mutex> lock2(queOptMutex);
    imuQueImu.push_back(imuMsg);
    imuQueOpt.push_back(imuMsg);
    if (doneFirstOpt == false)
        return;
    // 返回一个当前的odom
    gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
    // publish odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp = imuMsg.header.stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "odom_imu";

    // transform imu pose to ldiar
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
    gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

    odometry.pose.pose.position.x = lidarPose.translation().x();
    odometry.pose.pose.position.y = lidarPose.translation().y();
    odometry.pose.pose.position.z = lidarPose.translation().z();
    odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
    
    odometry.twist.twist.linear.x = currentState.velocity().x();
    odometry.twist.twist.linear.y = currentState.velocity().y();
    odometry.twist.twist.linear.z = currentState.velocity().z();
    odometry.twist.twist.angular.x = imuMsg.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y = imuMsg.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z = imuMsg.angular_velocity.z + prevBiasOdom.gyroscope().z();

    this->imuOdomQueue.push_back(odometry);

    std::lock_guard<mutex> odomLock(odomMutex);
    // 发布增量式的odom
    if (lidarOdomTime == -1)
        return;
    while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
    Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
    Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
    Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
    Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
    
    // publish latest odometry利用上一帧的雷达里程计坐标加上imuOdom的增量，得到的imu频率的雷达里程计
    nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
    laserOdometry.pose.pose.position.x = x;
    laserOdometry.pose.pose.position.y = y;
    laserOdometry.pose.pose.position.z = z;
    laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    lidarIncreOdom = laserOdometry;
    imuOdom = odometry;
}

void IMUPreIntegrator::setOdomMsg(const nav_msgs::Odometry& odomMsg) {
    std::lock_guard<mutex> odomLock(odomMutex);
    lidarOdomAffine = odom2affine(odomMsg);
    lidarOdomTime = odomMsg.header.stamp.toSec();
}

void IMUPreIntegrator::pushOdomIncreMsg(const nav_msgs::Odometry& odomMsg){
    // 优化操作
    std::lock_guard<std::mutex> lock2(queOptMutex);
    std::lock_guard<std::mutex> lock1(queImuMutex);
    double currentCorrectionTime = ROS_TIME(&odomMsg);

    if (imuQueOpt.empty())
        return;
    float p_x = odomMsg.pose.pose.position.x;
    float p_y = odomMsg.pose.pose.position.y;
    float p_z = odomMsg.pose.pose.position.z;
    float r_x = odomMsg.pose.pose.orientation.x;
    float r_y = odomMsg.pose.pose.orientation.y;
    float r_z = odomMsg.pose.pose.orientation.z;
    float r_w = odomMsg.pose.pose.orientation.w;
    bool degenerate = (int)odomMsg.pose.covariance[0] == 1 ? true : false;
        // lidarPose是因子图初始的一个定住的点
    gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));
    if (systemInitialized == false) {
        resetOptimization();

        // pop old IMU message
        while (!imuQueOpt.empty())
        {
            if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime)
            {
                lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        prevPose_ = lidarPose;
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
        graphFactors.add(priorPose);
        // initial velocity
        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
        graphFactors.add(priorVel);
        // initial bias
        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        
        key = 1;
        systemInitialized = true;
        return;
        }

    if(key == 100) {
        // get updated noise before reset
        gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
        gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
        gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
        // reset graph
        resetOptimization();
        // add pose
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
        graphFactors.add(priorPose);
        // add velocity
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
        graphFactors.add(priorVel);
        // add bias
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();
        key = 1;
        logger->info("imu msg get 100 times, size: {}", imuQueOpt.size());
    }

    while (!imuQueOpt.empty()) {
        // pop and integrate imu data that is between two optimizations
        sensor_msgs::Imu *thisImu = &imuQueOpt.front();
        double imuTime = ROS_TIME(thisImu);
        if (imuTime < currentCorrectionTime)
        {
            double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
            imuIntegratorOpt_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
            
            lastImuT_opt = imuTime;
            imuQueOpt.pop_front();
        }
        else
            break;
    }
    // 将当前时刻的imu的测量值加入到因子图中，执行优化
    // add imu factor to graph
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
    graphFactors.add(imu_factor);
    // add imu bias between factor
    graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                        gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
    // add pose factor
    gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
    graphFactors.add(pose_factor);
    // insert predicted values
    gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    graphValues.insert(X(key), propState_.pose());
    graphValues.insert(V(key), propState_.v());
    graphValues.insert(B(key), prevBias_);
    // optimize
    optimizer.update(graphFactors, graphValues);
    optimizer.update();
    graphFactors.resize(0);
    graphValues.clear();
    // Overwrite the beginning of the preintegration for the next step.
    gtsam::Values result = optimizer.calculateEstimate();
    prevPose_  = result.at<gtsam::Pose3>(X(key));
    prevVel_   = result.at<gtsam::Vector3>(V(key));
    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
    // Reset the optimization preintegration object.
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    // check optimization
    if (failureDetection(prevVel_, prevBias_)){
        resetParams();
        return;
    }

    // 2. after optiization, re-propagate imu odometry preintegration
    // 在因子图执行优化完毕后，将imu优化得到的bias重新送入imu预积分
    prevStateOdom = prevState_;
    prevBiasOdom  = prevBias_;
    // first pop imu message older than current correction data
    double lastImuQT = -1;
    while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime)
    {
        lastImuQT = ROS_TIME(&imuQueImu.front());
        imuQueImu.pop_front();
    }
    // repropogate
    if (!imuQueImu.empty())
    {
        // reset bias use the newly optimized bias
        imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
        // integrate imu message from the beginning of this optimization
        for (int i = 0; i < (int)imuQueImu.size(); ++i)
        {
            sensor_msgs::Imu *thisImu = &imuQueImu[i];
            double imuTime = ROS_TIME(thisImu);
            double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

            imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                    gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
            lastImuQT = imuTime;
        }
    }
    ++key;
    doneFirstOpt = true;
}
