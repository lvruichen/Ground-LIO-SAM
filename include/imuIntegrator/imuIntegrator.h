// 需要处理两个imu数据的队列，一个是用来构建因子图的队列。一个是用来拿到当前频率的100hz的位姿
// 当一个激光里程计位姿到来时，当前的imu队列可以分为两个部分，一个是激光帧之前的，一个是激光帧之后的
// 获得当前时刻的位姿
// 获取当前时刻的imu的偏置
// 类中保持两个imuIntegrator，一个用来优化，一个用来传播
#ifndef __IMUINTEGRATOR_HH__
#define __IMUINTEGRATOR_HH__

#include <mutex>
#include <queue>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// logger
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/common/transforms.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

// 用来输入计算两帧点云之间的imu的预积分，输入为lio-sam优化得到的10hz的雷达里程计和增量里程计以及原始的imu数据
class TransformFusion {
    public:
    std::mutex mtx;

    ros::NodeHandle nh;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubImuPath;

    Eigen::Affine3f transOdom2Map;
    Eigen::Affine3f transBase2Odom;
    Eigen::Affine3f transBase2Map;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion();
    ~TransformFusion();
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);

};

class IMUPreIntegrator {
private:
   // for propagation
    std::deque<sensor_msgs::Imu> imuQueImu;
    // for optimization 
    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<nav_msgs::Odometry> imuOdomQueue;

    std::mutex odomMutex;
    std::mutex queImuMutex;
    std::mutex queOptMutex;

    bool systemInitialized{false};
    bool doneFirstOpt{false};
    double lidarOdomTime = -1;
    double lastImuT_imu{-1};
    double lastImuT_opt{-1};
    int key{1};

    std::shared_ptr<spdlog::logger> logger;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::PreintegratedImuMeasurements* imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements* imuIntegratorImu_;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0, 0, 0));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0, 0, 0));

public:
    IMUPreIntegrator(std::shared_ptr<spdlog::logger> _logger);
    ~IMUPreIntegrator();
    void resetOptimization();
    void resetParams();
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);
    void getImuBias() const;
    // 可以获得一个imu里程计和一个雷达里程计
    void pushImuMsg(const sensor_msgs::Imu& imuMsg, std::deque<nav_msgs::Odometry>& odomIncreQueue, ros::Publisher& pubImuOdometry);
    void pushOdomIncreMsg(const nav_msgs::Odometry& odomMsg);
};

#endif