cmake_minimum_required(VERSION 3.0.2)
find_package(GTSAM REQUIRED QUIET)

add_library(featureExtractor featureExtractor.cpp)
target_link_libraries(featureExtractor PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Threads::Threads)

add_library(imuIntegrator imuIntegrator.cpp)
target_link_libraries(imuIntegrator PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Threads::Threads)

add_library(poseEstimator poseEstimator.cpp mapManager.cpp)
target_link_libraries(poseEstimator PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Threads::Threads)

add_executable(${PROJECT_NAME}_lio main.cpp)
target_link_libraries(${PROJECT_NAME}_lio featureExtractor imuIntegrator poseEstimator)
target_link_libraries(${PROJECT_NAME}_lio ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Threads::Threads)
