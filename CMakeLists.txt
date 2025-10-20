cmake_minimum_required(VERSION 3.0.2)
project(fisheye2panorama)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
)
find_package(OpenCV REQUIRED COMPONENTS 
    core
    imgproc
)
find_package(yaml-cpp REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

add_executable(fisheye2panorama_node
  src/fisheye2panorama_node.cpp
)
target_link_libraries(fisheye2panorama_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)