cmake_minimum_required(VERSION 2.8.3)
project(cloud_register)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive")

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_msgs tf pcl_ros)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
)

set(HDRS
    include/cloud_register.h
    include/ndt.h
    include/ndtRegistration.h
    include/voxel_grid_covariance.h
    include/voxel_grid_covariance.hpp
    include/ndt.hpp
    include/boost.h)
set(SRCS
    src/cloud_register.cpp
    src/voxel_grid_covariance.cpp
    src/ndtRegistration.cpp)


add_executable(cloud_register_ndt src/ndt_main.cpp)

add_library(cloud_register_lib ${HDRS} ${SRCS})

 target_link_libraries(cloud_register_ndt
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

 target_link_libraries(cloud_register_lib
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )


