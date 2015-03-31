cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_launcher)

find_package(catkin REQUIRED COMPONENTS roscpp semantic_map calibrate_sweeps cloud_merge object_manager semantic_map_publisher)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   CATKIN_DEPENDS roscpp semantic_map calibrate_sweeps cloud_merge object_manager semantic_map_publisher
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


############################# INSTALL TARGETS

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
