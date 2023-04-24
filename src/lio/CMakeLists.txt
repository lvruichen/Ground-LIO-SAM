cmake_minimum_required(VERSION 3.0.2)
find_package(GTSAM REQUIRED QUIET)

add_library(featureExtractor featureExtractor.cpp)
target_link_libraries(featureExtractor PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(imuIntegrator imuIntegrator.cpp)
target_link_libraries(imuIntegrator PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_library(poseEstimator poseEstimator.cpp mapManager.cpp)
target_compile_options(poseEstimator PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(poseEstimator PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(${PROJECT_NAME}_lio_front_end front_end.cpp)
target_link_libraries(${PROJECT_NAME}_lio_front_end featureExtractor imuIntegrator)
target_link_libraries(${PROJECT_NAME}_lio_front_end ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_lio_back_end back_end.cpp)
target_compile_options(${PROJECT_NAME}_lio_back_end PRIVATE)
target_link_libraries(${PROJECT_NAME}_lio_back_end ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam poseEstimator)


add_executable(LMtest LMtest.cpp)
target_link_libraries(LMtest ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})