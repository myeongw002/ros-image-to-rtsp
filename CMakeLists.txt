cmake_minimum_required(VERSION 3.0.2)
project(ros_image_to_rtsp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_to_rtsp_cpp src/image_to_rtsp_cpp.cpp)
target_link_libraries(image_to_rtsp_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

