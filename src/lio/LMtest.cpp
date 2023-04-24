#include <string>

#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/imgproc.hpp>



using PointType = pcl::PointXYZI;
using namespace std;

Eigen::Matrix4d initialTrans;
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
std::vector<PointType> coeffSelSurfVec;
std::vector<bool> laserCloudOriSurfFlag;
pcl::PointCloud<PointType>::Ptr laserCloudOri;
pcl::PointCloud<PointType>::Ptr coeffSel;
float transformTobeMapped[6]{0, 0, 0, 0, 0, 0};
cv::Mat matP;
bool isDegenerate{false};

void pointAssociateToMap(const PointType* pi, PointType* po, Eigen::Matrix4d& transPointAssociateToMap) {
    po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
    po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
    po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
    po->intensity = pi->intensity;
}

void surfOptimization(pcl::PointCloud<PointType>::Ptr currentScan, pcl::PointCloud<PointType>::Ptr localSubmap) {
    #pragma omp parallel for num_threads(4)
    for (int i = 0; i < currentScan->size(); i++) {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        pointOri = currentScan->points[i];
        pointAssociateToMap(&pointOri, &pointSel, initialTrans); 
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0(j, 0) = localSubmap->points[pointSearchInd[j]].x;
                matA0(j, 1) = localSubmap->points[pointSearchInd[j]].y;
                matA0(j, 2) = localSubmap->points[pointSearchInd[j]].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * localSubmap->points[pointSearchInd[j]].x +
                            pb * localSubmap->points[pointSearchInd[j]].y +
                            pc * localSubmap->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid) {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                        + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }
}

void combineOptimizationCoeffs(pcl::PointCloud<PointType>::Ptr currentScan) {
    for (int i = 0; i < currentScan->size(); ++i){
        if (laserCloudOriSurfFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    // reset flag for next iteration
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

bool LMOptimization(int _iterCount) {
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    float srx = sin(transformTobeMapped[1]);
    float crx = cos(transformTobeMapped[1]);
    float sry = sin(transformTobeMapped[2]);
    float cry = cos(transformTobeMapped[2]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

    int laserCloudSelNum = laserCloudOri->size();
    if (laserCloudSelNum < 50) {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++) {
        // lidar -> camera
        pointOri.x = laserCloudOri->points[i].y;
        pointOri.y = laserCloudOri->points[i].z;
        pointOri.z = laserCloudOri->points[i].x;
        // lidar -> camera
        coeff.x = coeffSel->points[i].y;
        coeff.y = coeffSel->points[i].z;
        coeff.z = coeffSel->points[i].x;
        coeff.intensity = coeffSel->points[i].intensity;
        // in camera
        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                    + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                    + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                    + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                    + ((-cry*crz - srx*sry*srz)*pointOri.x 
                    + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                    + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                    + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
        // camera -> lidar
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (_iterCount == 0) {

        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate)
    {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(
                        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
                        pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
        return true; // converged
    }
    return false; // keep optimizing
}


int main(int argc, char const *argv[])
{
    string scanPath = "/home/eric/Downloads/hgy/scan.pcd";
    string submapPath = "/home/eric/Downloads/hgy/submap.pcd";
    string initialPosePath = "/home/eric/Downloads/hgy/init_pose.txt";
    pcl::PointCloud<PointType>::Ptr scanCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr submapCloud(new pcl::PointCloud<PointType>());
    // read cloud
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (scanPath, *scanCloud) == -1) {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (submapPath, *submapCloud) == -1) {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    // initial guess;
    ifstream inFile;
    inFile.open(initialPosePath);
    double tmp;
    int count = 0;
    while(inFile >> tmp) {
        initialTrans(count / 4, count % 4) = tmp;
        count++;
    }
    // cout << initialTrans << endl;
    inFile.close();
    Eigen::Affine3d rotation(initialTrans.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    pcl::getEulerAngles(rotation, roll, pitch, yaw);
    pcl::PointCloud<PointType>::Ptr originScan(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*scanCloud, *originScan, initialTrans);

    transformTobeMapped[0] = roll;
    transformTobeMapped[1] = pitch;
    transformTobeMapped[2] = yaw;
    transformTobeMapped[3] = initialTrans(0, 3);
    transformTobeMapped[4] = initialTrans(1, 3);
    transformTobeMapped[5] = initialTrans(2, 3);


    cout << "Before LM Optimization: " << endl;
    cout << "x: " << transformTobeMapped[3] << endl;
    cout << "y: " << transformTobeMapped[4] << endl;
    cout << "z: " << transformTobeMapped[5] << endl;
    cout << "roll: " << transformTobeMapped[0] << endl;
    cout << "pitch: " << transformTobeMapped[1] << endl;
    cout << "yaw: " << transformTobeMapped[2] << endl;

    // initialization
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    laserCloudOriSurfVec.resize(16 * 1800);
    coeffSelSurfVec.resize(16 * 1800);
    laserCloudOriSurfFlag.resize(16 * 1800);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());
    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    // scan2Map
    kdtreeSurfFromMap->setInputCloud(submapCloud);

    for (int iterCount = 0; iterCount < 30; ++iterCount) {
        laserCloudOri->clear();
        coeffSel->clear();
        surfOptimization(scanCloud, submapCloud);
        combineOptimizationCoeffs(scanCloud);
        if (LMOptimization(iterCount) == true) {
            break;    
        }  
    }
    cout << "After LM Optimization: " << endl;
    cout << "x: " << transformTobeMapped[3] << endl;
    cout << "y: " << transformTobeMapped[4] << endl;
    cout << "z: " << transformTobeMapped[5] << endl;
    cout << "roll: " << transformTobeMapped[0] << endl;
    cout << "pitch: " << transformTobeMapped[1] << endl;
    cout << "yaw: " << transformTobeMapped[2] << endl;

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(transformTobeMapped[0],Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(transformTobeMapped[1],Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(transformTobeMapped[2],Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=yawAngle*pitchAngle*rollAngle;
    Eigen::Matrix4d trans;
    trans.block<3, 3>(0, 0) = rotation_matrix;
    trans(0, 3) = transformTobeMapped[3];
    trans(1, 3) = transformTobeMapped[4];
    trans(2, 3) = transformTobeMapped[5];

    pcl::transformPointCloud(*scanCloud, *scanCloud, trans);

    // visualize
    pcl::visualization::PCLVisualizer vis("vis");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> target_handler(submapCloud, 255.0, 0.0, 0.0);  // r
    pcl::visualization::PointCloudColorHandlerCustom<PointType> source_handler(originScan, 0.0, 255.0, 0.0);  // g
    pcl::visualization::PointCloudColorHandlerCustom<PointType> aligned_handler(scanCloud, 0.0, 0.0, 255.0);      // b
    vis.addPointCloud(submapCloud, target_handler, "target");
    vis.addPointCloud(originScan, source_handler, "source");
    vis.addPointCloud(scanCloud, aligned_handler, "aligned");
    vis.spin();
    return 0;
}
