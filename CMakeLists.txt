cmake_minimum_required(VERSION 3.0.2)
project(rtsp_image_sender)

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

add_executable(compressedimage_to_rtsp_cpp src/compressedimage_to_rtsp_cpp.cpp)
target_link_libraries(compressedimage_to_rtsp_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

