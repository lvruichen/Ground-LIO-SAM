#include "featureExtractor/featureExtractor.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>


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

        featureExtractorPtr = new FeatureExtractor();

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


        resetParameters();
    }



private:
    ros::NodeHandle nh;
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerCloud;
    ros::Publisher pubSurfCloud;
    ros::Subscriber subLidarCloud;

    CloudType::Ptr laserCloudIn;
    CloudType::Ptr cornerCloud;
    CloudType::Ptr surfCloud; 

    std::string configFile;

    int N_SCAN;
    int Horizon_SCAN;

    FeatureExtractor* featureExtractorPtr;
    std::shared_ptr<spdlog::logger> logger;
    sensor_msgs::PointCloud2 curCloudMsg;

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "ScanRegistration");

    scanRegistrator SR;

    ROS_INFO("\033[1;32m----> ScanRegistration Started.\033[0m");

    ros::spin();
    return 0; 
}