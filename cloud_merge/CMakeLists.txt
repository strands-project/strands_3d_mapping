cmake_minimum_required(VERSION 2.8.3)
project(cloud_merge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs tf pcl_ros semantic_map image_geometry image_transport qt_build cloud_register ros_datacentre_cpp_client)

set(CMAKE_CXX_FLAGS "-mno-avx -O4 -fPIC -std=c++0x -fpermissive")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


set(HDRS
    include/cloud_merge.h
    include/cloud_merge_node.h)

set(SRCS
    src/cloud_merge.cpp
    src/cloud_merge_node.cpp
    src/main.cpp)

add_executable(cloud_merge ${HDRS} ${SRCS})

 target_link_libraries(cloud_merge
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
   mongoclient
   message_store
 )
