cmake_minimum_required(VERSION 3.0.2)
find_package(GTSAM REQUIRED QUIET)

add_library(featureExtractor SHARED featureExtractor.cpp)
target_link_libraries(featureExtractor ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(imuIntegrator SHARED imuIntegrator.cpp)
target_link_libraries(imuIntegrator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_library(poseEstimator SHARED poseEstimator.cpp mapManager.cpp)
target_link_libraries(poseEstimator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(${PROJECT_NAME}_lio main.cpp)
target_link_libraries(${PROJECT_NAME}_lio featureExtractor imuIntegrator poseEstimator)
target_link_libraries(${PROJECT_NAME}_lio ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
