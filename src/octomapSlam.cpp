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
using Point = pcl::PointXYZI;

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
    ros::Time timeScanCurStamp;
    sensor_msgs::PointCloud2 currentCloudMsg;
    Eigen::Matrix4f currentPose;

    int submapCapacity = 10;
    int laserInterval = 2;

    int laserCount = 0;
    int curSubmapSzie = 0;
    
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

        pubGloablCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/Octomap/global_map", 1);

        submapCapacity = data["submapCapacity"].get<int>();
        laserInterval = data["laserInterval"].get<int>();
        unique_ptr<octomap::OcTree> ptr(new octomap::OcTree(data["resolution"].get<double>()));
        tree = move(ptr);
        // tree = make_unique<octomap::OcTree>(new oc   tomap::OcTree(treeResolution));
        tree->setProbHit(data["probability_hit"].get<float>());
        tree->setProbMiss(data["probability_miss"].get<float>());
        tree->setClampingThresMin(data["threshold_min"].get<float>());
        tree->setClampingThresMax(data["threshold_max"].get<float>());
        tree->setOccupancyThres(data["threshold_occupancy"].get<float>());
        allocateMemory();
        logger->info("initialize succeed !");
    }   

    void allocateMemory() {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        submapRawCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        submapStaticCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        submapDynamicCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        resetParameters();
    }

    void resetParameters() {
        laserCloudIn->clear();
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
            curSubmapSzie++;
            laserCount = 0;
            currentCloudMsg = *laserCloudMsg;
            laserCloudIn->clear();
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);

            // cloud转化到map坐标系下
            if(!this->transCurCloud())
                return;

            // cloud插入到tree里面
            this->insertCloudtoTree();

            // cloud插入到submapRawCloud
            this->insertCloudtoSubmap();

            if(curSubmapSzie < submapCapacity) {
                return;
            }
            else {
                // 当submap的容量达到上限时，开始利用octree分离动态点云和静态点云
                this->splitRawSubmap();
                // 得到的submapRaw, submapStatic, submapDynami插入到set里面
                this->saveSubmap();
                curSubmapSzie = 0;
                resetParameters();
            }

        }
    }

    bool transCurCloud() {
        timeScanCur = currentCloudMsg.header.stamp.toSec();
        timeScanCurStamp = currentCloudMsg.header.stamp;
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
        tree->updateInnerOccupancy();
        for (const auto &p : *submapRawCloud) {
            octomap::OcTreeNode* node = tree->search(p.x, p.y, p.z);
            if (node == nullptr) {
                continue;
            }
            if (tree->isNodeOccupied(node) || p.z < -1) {
                submapStaticCloud->points.push_back(p);
            }
            else if (!tree->isNodeOccupied(node) && p.z > -1) {
                submapDynamicCloud->points.push_back(p);
            }
            else {
                submapStaticCloud->points.push_back(p);
            }
        }
    }

    void saveSubmap() {
        pcl::PointCloud<PointXYZIRT>::Ptr submapRawCloud_(new pcl::PointCloud<PointXYZIRT>());
        pcl::PointCloud<PointXYZIRT>::Ptr submapStaticCloud_(new pcl::PointCloud<PointXYZIRT>());
        pcl::PointCloud<PointXYZIRT>::Ptr submapDynamicCloud_(new pcl::PointCloud<PointXYZIRT>());
        pcl::copyPointCloud(*submapRawCloud, *submapRawCloud_);
        pcl::copyPointCloud(*submapStaticCloud, *submapStaticCloud_);
        pcl::copyPointCloud(*submapDynamicCloud, *submapDynamicCloud_);
        RawSubmapSet.push_back(submapRawCloud_);
        StaticSubmapSet.push_back(submapStaticCloud_);
        DynamicSubmapSet.push_back(submapDynamicCloud_);
    }

    // 以一定的频率可视化动态点云和静态点云
    void visualizeGlobalMapThread() {
        ros::Rate rate(0.5);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();
        }
    }

    void publishGlobalMap() {
        logger->info("current global submap set szie: {}", RawSubmapSet.size());
        pcl::PointCloud<PointXYZIRT>::Ptr globalMapFrames(new pcl::PointCloud<PointXYZIRT>);
        pcl::PointCloud<PointXYZIRT>::Ptr globalMapFramesDS(new pcl::PointCloud<PointXYZIRT>);
        for (int i = 0; i < StaticSubmapSet.size(); ++i) {
            *globalMapFrames += *StaticSubmapSet[i];
        }
        pcl::VoxelGrid<PointXYZIRT> downSizeFilterGloablMap;
        downSizeFilterGloablMap.setLeafSize(0.1, 0.1, 0.1);
        downSizeFilterGloablMap.setInputCloud(globalMapFrames);
        downSizeFilterGloablMap.filter(*globalMapFramesDS);
        logger->info("global map DS size: {}", globalMapFramesDS->points.size());
        publishCloud(pubGloablCloud, globalMapFramesDS, timeScanCurStamp, "map");
    }

    void visibilityCheckThread() {
        ros::Rate rate(5);
        while(ros::ok()) {
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_slam");

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);    

    OctomapSlam os;

    ROS_INFO("\033[1;32m----> Octomap Slam Started.\033[0m");

    std::thread visualizeMapThread(&OctomapSlam::visualizeGlobalMapThread, &os);

    ros::spin();

    visualizeMapThread.join();

    return 0;

}