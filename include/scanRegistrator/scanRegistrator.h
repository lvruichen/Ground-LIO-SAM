#include "featureExtractor/featureExtractor.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// test 
#include "lio_sam/utility.h"
#include "lio_sam/cloud_info.h"

using std::cout;
using std::endl;
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

class scanRegistrator {
public:
    scanRegistrator() {
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/extracted_cloud", 1);
        pubCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/corner_cloud", 1);
        pubSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/planner_cloud", 1);
        subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_points", 5, 
                            &scanRegistrator::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);

        // subImu        = nh.subscribe<sensor_msgs::Imu>("/imu/data", 2000, &scanRegistrator::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>("/lio_sam/odometry/imu_incremental", 2000, &scanRegistrator::odometryHandler, this, ros::TransportHints().tcpNoDelay());


        logger = spdlog::stdout_color_mt("console");  

        nh.getParam("configFile", configFile);
        
        logger->info("config file path: {}",configFile);
        cv::FileStorage fs(configFile, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            cout << "wrong config file path" << endl;
            ros::shutdown();
            return;
        }
        N_SCAN = static_cast<int>(fs["N_SCAN"]);
        Horizon_SCAN = static_cast<int>(fs["Horizon_SCAN"]);

        featureExtractorPtr = new FeatureExtractor(logger);

        allocateMemory();

    };

    ~scanRegistrator() {

    };

    void allocateMemory() {
        laserCloudIn.reset(new CloudType());
        cornerCloud.reset(new CloudType());
        surfCloud.reset(new CloudType());
        resetParameters();
    }

    void resetParameters() {
        laserCloudIn->clear();
        cornerCloud->clear();
        surfCloud->clear();
    }

    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg) {

        curCloudMsg = *laserCloudMsg;
        featureExtractorPtr->featureExtract(curCloudMsg, cornerCloud, surfCloud);
        std_msgs::Header cloudHeader = laserCloudMsg->header;
        publishCloud(pubCornerCloud, cornerCloud, cloudHeader.stamp, cloudHeader.frame_id);
        publishCloud(pubSurfCloud, surfCloud, cloudHeader.stamp, cloudHeader.frame_id);

        // test
        sensor_msgs::PointCloud2 tempCloud1;
        tempCloud1.header = laserCloudMsg->header;
        pcl::toROSMsg(*cornerCloud, tempCloud1);
        cloudInfo.cloud_corner = tempCloud1;

        sensor_msgs::PointCloud2 tempCloud2;
        pcl::toROSMsg(*surfCloud, tempCloud2);
        cloudInfo.cloud_surface = tempCloud2;

        cloudInfo.odomAvailable = false;
        cloudInfo.imuAvailable = true;
        cloudInfo.header = laserCloudMsg->header;

        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);
        sensor_msgs::Imu thisImuMsg = imuQueue.back();
        imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
        pubLaserCloudInfo.publish(cloudInfo);
        
        imuQueue.clear();
        odomQueue.clear();
        resetParameters();
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg) {
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(*imuMsg);
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg) {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    

private:
    ros::NodeHandle nh;
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerCloud;
    ros::Publisher pubSurfCloud;

    ros::Subscriber subLidarCloud;
    ros::Subscriber subImu;
    ros::Subscriber subOdom;

    CloudType::Ptr laserCloudIn;
    CloudType::Ptr cornerCloud;
    CloudType::Ptr surfCloud; 

    std::string configFile;

    int N_SCAN;
    int Horizon_SCAN;

    FeatureExtractor* featureExtractorPtr;
    std::shared_ptr<spdlog::logger> logger;
    sensor_msgs::PointCloud2 curCloudMsg;

    // test
    std::deque<sensor_msgs::Imu> imuQueue;
    std::deque<nav_msgs::Odometry> odomQueue;
    lio_sam::cloud_info cloudInfo;
    std::mutex imuLock;
    std::mutex odoLock;


};