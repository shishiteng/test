cmake_minimum_required(VERSION 2.8.3)
project(openni2_recorder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARIES})


add_executable(openni2_convert src/openni2_convert.cpp)
target_link_libraries(openni2_convert ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

