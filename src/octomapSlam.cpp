#include "lio_sam/utility.h"
#include "lio_sam/cloud_info.h"
#include "common/nlohmann/json.hpp"
#include "common/pcl_utils/pcl_utils.h"
#include "common/ros_utils/transform.hpp"
#include <octomap/octomap.h>


using json = nlohmann::json;
using namespace ros_utils;

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

using PointXYZIRT = VelodynePointXYZIRT;

class OctomapSlam : public ParamServer
{
private:
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subImuOdom;

    ros::Publisher pubGloablCloud;

    std::deque<nav_msgs::Odometry> odomQueue;
    std::shared_ptr<spdlog::logger> logger;
    std::unique_ptr<octomap::OcTree> tree;
    std::vector<pcl::PointCloud<PointXYZIRT>::Ptr> RawSubmapSet;
    std::vector<pcl::PointCloud<PointXYZIRT>::Ptr> StaticSubmapSet;
    std::vector<pcl::PointCloud<PointXYZIRT>::Ptr> DynamicSubmapSet;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    
    pcl::PointCloud<PointXYZIRT>::Ptr submapRawCloud;
    pcl::PointCloud<PointXYZIRT>::Ptr submapStaticCloud;
    pcl::PointCloud<PointXYZIRT>::Ptr submapDynamicCloud;

    json data;
    double timeScanCur;
    sensor_msgs::PointCloud2 currentCloudMsg;
    Eigen::Matrix4f currentPose;

    int submapCapacity = data["submapCapacity"].get<int>();
    int laserInterval = 1;
    int laserCount = 0;
    
public:
    OctomapSlam() {
        logger = spdlog::stdout_color_mt("console");
        string jsonFile = "/home/eric/a_ros_ws/lio_sam_lrc/src/dynamic-removal/config/octomapConfig.json";
        fstream f(jsonFile);
        data = json::parse(f);
        subImuOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, 
        &OctomapSlam::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1,
        &OctomapSlam::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        tree = std::make_unique<octomap::OcTree>(new octomap::OcTree(data["resolution"].get<float>()));
        tree->setProbHit(data["probability_hit"].get<float>());
        tree->setProbMiss(data["probability_miss"].get<float>());
        tree->setClampingThresMin(data["threshold_min"].get<float>());
        tree->setClampingThresMax(data["threshold_max"].get<float>());
        tree->setOccupancyThres(data["threshold_occupancy"].get<float>());

        allocateMemory();
    }   

    void allocateMemory() {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        submapRawCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        submapStaticCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        submapDynamicCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        resetParameters();
    }

    void resetParameters() {
        tree->clear();
        submapRawCloud->clear();
        submapStaticCloud->clear();
        submapDynamicCloud->clear();
    }

    ~OctomapSlam() {}

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg) {
        std::lock_guard<std::mutex> lock1(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
        // 如果需要添加间隔，则在间隔内return
        laserCount++;
        if(laserCount == laserInterval) {
            laserCount = 0;
            currentCloudMsg = *laserCloudMsg;
            pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

            // cloud转化到map坐标系下
            if(!transCurCloud())
                return;

            // cloud插入到tree里面
            insertCloudtoTree();

            // cloud插入到submapRawCloud
            insertCloudtoSubmap();

            // 当submap的容量达到上限时，开始利用octree分离动态点云和静态点云

            // 得到的submapRaw, submapStatic, submapDynami插入到set里面 
        }

        

    }

    bool transCurCloud() {
        timeScanCur = currentCloudMsg.header.stamp.toSec();
        std::lock_guard<std::mutex> lock1(odoLock);
        if (odomQueue.empty() || odomQueue.front().header.stamp.toSec() > timeScanCur) {
            logger->info("Waiting for odom data ...");
            return false;
        }
        while(!odomQueue.empty()) {
            if(odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }
        if(odomQueue.empty())
            return false;
        nav_msgs::Odometry thisOdom = odomQueue.front();
        geometry_msgs::Transform thisTrans = OdomToTransform(thisOdom);
        currentPose = TransformToMatrix(thisTrans);
        pcl::transformPointCloud(*laserCloudIn, *laserCloudIn, currentPose);
        return true;
    }

    void insertCloudtoTree() {
        octomap::Pointcloud cloudOcto;
        for (const auto &p : laserCloudIn->points) {
            cloudOcto.push_back(p.x, p.y, p.z);
        }
        tree->insertPointCloud(cloudOcto, octomap::point3d(currentPose(0, 3), currentPose(1, 3), currentPose(2, 3)));
    }

    void insertCloudtoSubmap() {
        *submapRawCloud += *laserCloudIn;
    }

    void splitRawSubmap() {
        for (const auto &p : *submapRawCloud) {
            
        }
    }

    // 以一定的频率可视化动态点云和静态点云
    void visualizeGlobalMapThread() {

    }

    void publishGlobalMap() {

    }


};