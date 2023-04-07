cmake_minimum_required(VERSION 3.0.2)

add_executable(${PROJECT_NAME}_scanRegistration scanRegistration.cpp featureExtractor.cpp)
target_link_libraries(${PROJECT_NAME}_scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})