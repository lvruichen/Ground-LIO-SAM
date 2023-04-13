#include "featureExtractor/featureExtractor.h"

FeatureExtractor::FeatureExtractor(std::shared_ptr<spdlog::logger> _logger) {
    logger = _logger;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    allocateMemory();
}

FeatureExtractor::~FeatureExtractor() {

}

void FeatureExtractor::allocateMemory() {
    rawCloud.reset(new pcl::PointCloud<PointXYZIRT>());
    // cornerCloud.reset(new pcl::PointCloud<PointType>());
    // surfCloud.reset(new pcl::PointCloud<PointType>());

    vlines.resize(N_SCAN);
    for (auto& ptr : vlines) {
        ptr.reset(new pcl::PointCloud<PointType>());
    }
    vcorner.resize(N_SCAN);
    vsurf.resize(N_SCAN);

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
}

void FeatureExtractor::resetParameters() {
    rawCloud->clear();
    for (auto& ptr : vlines) {
        ptr->clear();
    }
    for (auto& v : vcorner) {
        v.clear();
    }
    for (auto& v : vsurf) {
        v.clear();
    }
    // cornerCloud->clear();
    // surfCloud->clear();
}

// 对每个激光束操作，提取其中的角点和平面点
void FeatureExtractor::detectFeaturePoint(pcl::PointCloud<PointType>::Ptr& cloud,
                            std::vector<int>& pointSharp,
                            std::vector<int>& pointFlat) {
    // filter to close and too far points
    pcl::PointCloud<PointType>::Ptr _laserCloud(new pcl::PointCloud<PointType>());
    std::vector<float> _cloudRange;
    std::vector<float> _cloudCurvature;
    std::vector<int> _cloudNeighborPicked;
    std::vector<int> _cloudLabel;
    std::vector<int> _cloudColumn;
    std::vector<int> _cloudIdxOri;
    std::vector<smoothness_t> _cloudSmoothness;
    pointSharp.clear();
    pointFlat.clear();

    _laserCloud->points.reserve(cloud->size());
    _cloudRange.reserve(cloud->size());
    _cloudColumn.reserve(cloud->size());
    _cloudIdxOri.reserve(cloud->size());

    // filter cloud and calculate columun idx;
    for (int i = 0; i < cloud->size(); ++i) {
        PointType& p = cloud->points[i];
        float _range = pointDistance(p);
        if (_range < lidarMinRange || _range > lidarMaxRange)
            continue;
        float _horizonAngle = atan2(p.x, p.y) * 180 / M_PI;
        static float _ang_res_x = 360.0 / float(Horizon_SCAN);
        int _colIdx = -round((_horizonAngle - 90.0) / _ang_res_x) + Horizon_SCAN/2;
        if (_colIdx >= Horizon_SCAN)
            _colIdx -= Horizon_SCAN;
        if (_colIdx < 0 || _colIdx >= Horizon_SCAN)
            continue;
        
        _laserCloud->points.push_back(p);
        _cloudRange.push_back(_range);
        _cloudColumn.push_back(_colIdx);
        _cloudIdxOri.push_back(i);
    }

    // calculate roughness
    int _cloudSize = _laserCloud->size();
    _cloudCurvature.resize(_cloudSize);
    _cloudNeighborPicked.resize(_cloudSize);
    _cloudLabel.resize(_cloudSize);
    _cloudSmoothness.resize(_cloudSize);

    for (int i = 5; i < _cloudSize - 5; ++i) {
        float _diffRange = _cloudRange[i-5] + _cloudRange[i-4]
                        + _cloudRange[i-3] + _cloudRange[i-2]
                        + _cloudRange[i-1] - 10 *  _cloudRange[i]
                        + _cloudRange[i+1] + _cloudRange[i+2]
                        + _cloudRange[i+3] + _cloudRange[i+4]
                        + _cloudRange[i+5];
        _cloudCurvature[i] = _diffRange * _diffRange;
        _cloudNeighborPicked[i] = 0;
        _cloudLabel[i] = 0;
        _cloudSmoothness[i].value = _cloudCurvature[i];
        _cloudSmoothness[i].ind = i;
    }

    // mark occulusion
    for (int i = 5; i < _cloudSize - 6; ++i) {
        float depth1 = _cloudRange[i];
        float depth2 = _cloudRange[i+1];
        int columnDiff = std::abs(int(_cloudColumn[i] - _cloudColumn[i+1]));
        if (columnDiff < 10) {
            // 10 pixel diff in range image
            if (depth1 - depth2 > 0.3){
                _cloudNeighborPicked[i - 5] = 1;
                _cloudNeighborPicked[i - 4] = 1;
                _cloudNeighborPicked[i - 3] = 1;
                _cloudNeighborPicked[i - 2] = 1;
                _cloudNeighborPicked[i - 1] = 1;
                _cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                _cloudNeighborPicked[i + 1] = 1;
                _cloudNeighborPicked[i + 2] = 1;
                _cloudNeighborPicked[i + 3] = 1;
                _cloudNeighborPicked[i + 4] = 1;
                _cloudNeighborPicked[i + 5] = 1;
                _cloudNeighborPicked[i + 6] = 1;
            }
        }
        // parallel beam
        float diff1 = std::abs(float(_cloudRange[i-1] - _cloudRange[i]));
        float diff2 = std::abs(float(_cloudRange[i+1] - _cloudRange[i]));

        if (diff1 > 0.02 * _cloudRange[i] && diff2 > 0.02 * _cloudRange[i])
            _cloudNeighborPicked[i] = 1;
    }

    // detect corner and surf
    for (int part = 0; part < 6; ++part) {
        int sp = 5 + (_cloudSize - 10) * part / 6;
        int ep = 5 + (_cloudSize - 10) * (part + 1) / 6 - 1;
        if (sp >= ep)
            continue;
        std::sort(_cloudSmoothness.begin() + sp, _cloudSmoothness.begin() + ep, by_value());
        int _largestPickedNum = 0;
        // corner points
        for (int k = ep; k >= sp; --k) {
            int ind = _cloudSmoothness[k].ind;
            if (_cloudNeighborPicked[ind] == 0 && _cloudCurvature[ind] > edgeThreshold) {
                _largestPickedNum++;
                if (_largestPickedNum <= 20) {
                    _cloudLabel[ind] = 1;
                    pointSharp.push_back(_cloudIdxOri[ind]);
                }
                else
                    break;
            }
            _cloudNeighborPicked[ind] = 1;

            // in case corner cloud too dense
            for (int l = 1; l <= 5; ++l) {
                int colDiff = std::abs(int(_cloudColumn[ind + l] = _cloudColumn[ind + l - 1]));
                if (colDiff > 10)
                    break;
                _cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; --l) {
                int colDiff = std::abs(int(_cloudColumn[ind + l] = _cloudColumn[ind + l + 1]));
                if (colDiff > 10)
                    break;
                _cloudNeighborPicked[ind + l] = 1;
            }        
        }

        // surf points
        for (int k = sp; k <= ep; k++) {
            int ind = _cloudSmoothness[k].ind;
            if (_cloudNeighborPicked[ind] == 0 && _cloudCurvature[ind] < surfThreshold) {
                _cloudLabel[ind] = -1;
                _cloudNeighborPicked[ind] = 1;

            }
            // in case surf cloud too dense
            for (int l = 1; l <= 5; ++l) {
                int colDiff = std::abs(int(_cloudColumn[ind + l] = _cloudColumn[ind + l - 1]));
                if (colDiff > 10)
                    break;
                _cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; --l) {
                int colDiff = std::abs(int(_cloudColumn[ind + l] = _cloudColumn[ind + l + 1]));
                if (colDiff > 10)
                    break;
                _cloudNeighborPicked[ind + l] = 1;
            }      
        }

        // 默认将非角点的点都当作平面点来处理
        for (int k = sp; k <= ep; ++k) {
            int ind = _cloudSmoothness[k].ind;
            if(_cloudLabel[k] <= 0) {
                pointFlat.push_back(_cloudIdxOri[ind]);
            }
        }
    }
}

void FeatureExtractor::featureExtract(sensor_msgs::PointCloud2 &msgIn, pcl::PointCloud<pcl::PointXYZI>::Ptr 
                        cornerCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr surfCloud) {
    auto msg1 = msgIn;
    pcl::moveFromROSMsg(msg1, *rawCloud);

    // check dense flag
    if (rawCloud->is_dense == false) {
        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }
    // check ring flag
    if (ringFlag == -1) {
        ringFlag = 0;
        for (auto &field : msgIn.fields) {
            if (field.name == "r" || field.name == "ring") {
                ringFlag = 1;
                break;
            }
        }
    }

    if (ringFlag == 0) {
        //generate ring channel
    }

    // logger->info("ring channel flag {}", ringFlag);
    // separate cloud to lines
    int _cloudNum = rawCloud->size();
    for (int i = 0; i < _cloudNum; ++i) {
        int _ring = rawCloud->points[i].ring;
        PointType _thisPoint;
        _thisPoint.x = rawCloud->points[i].x;
        _thisPoint.y = rawCloud->points[i].y;
        _thisPoint.z = rawCloud->points[i].z;
        _thisPoint.intensity = rawCloud->points[i].intensity;
        vlines[_ring]->push_back(_thisPoint);
    }

    std::thread thread[N_SCAN];
    for (int i = 0; i < N_SCAN; ++i) {
        thread[i] = std::thread(&FeatureExtractor::detectFeaturePoint, this, 
        std::ref(vlines[i]), std::ref(vcorner[i]), std::ref(vsurf[i]));
    }
    for (int i = 0; i < N_SCAN; ++i) {
        thread[i].join();
    }

    cornerCloud->clear();
    surfCloud->clear();
    pcl::PointCloud<PointType>::Ptr surfCloudOri(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCAN; ++i) {
        // the corner points and surf points in ring i
        for (int cornerIdx = 0; cornerIdx < vcorner[i].size(); ++cornerIdx) {
            int id = vcorner[i][cornerIdx];
            cornerCloud->push_back(vlines[i]->points[id]);
        }
        for (int surfIdx = 0; surfIdx < vsurf[i].size(); ++surfIdx) {
            int id = vsurf[i][surfIdx];
            surfCloudOri->push_back(vlines[i]->points[id]);
        }
    }    

    
    downSizeFilter.setInputCloud(surfCloudOri);
    downSizeFilter.filter(*surfCloud);
    // logger->info("detected {} corner points", cornerCloud->size());
    // logger->info("detected {} surf points", surfCloud->size());    

    resetParameters();
}
