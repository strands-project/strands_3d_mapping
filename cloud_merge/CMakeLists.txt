cmake_minimum_required(VERSION 2.8.3)
project(cloud_merge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS mongodb_store rospy roscpp actionlib actionlib_msgs std_msgs sensor_msgs tf pcl_ros semantic_map image_geometry image_transport qt_build)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

add_action_files(
    DIRECTORY
        action
    FILES
        Sweep.action
    )

generate_messages(DEPENDENCIES actionlib_msgs sensor_msgs)


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS mongodb_store semantic_map std_msgs sensor_msgs tf pcl_ros image_geometry image_transport qt_build actionlib actionlib_msgs rospy
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


set(HDRS
    include/cloud_merge/cloud_merge.h
    include/cloud_merge/cloud_merge_node.h)

set(SRCS
    src/cloud_merge.cpp
    src/cloud_merge_node.cpp)

add_executable(cloud_merge ${HDRS} ${SRCS} src/main.cpp)

add_dependencies(cloud_merge ${catkin_EXPORTED_TARGETS})

 target_link_libraries(cloud_merge
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
 )

############################# INSTALL TARGETS

install(TARGETS cloud_merge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
        scripts/do_sweep.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/cloud_merge/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
