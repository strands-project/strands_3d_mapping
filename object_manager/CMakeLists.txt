cmake_minimum_required(VERSION 2.8.3)
project(object_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs pcl_ros message_generation qt_build cv_bridge semantic_map metaroom_xml_parser)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")
set(CMAKE_PREFIX_PATH /usr/share/pcl-1.7/ ${CMAKE_PREFIX_PATH})
set(PCL_DIR /usr/share/pcl-1.7/)
find_package(PCL 1.7 REQUIRED NO_DEFAULT_PATH)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

rosbuild_prepare_qt4(QtCore QtXml)

add_service_files(
   FILES
   DynamicObjectsService.srv
   GetDynamicObjectService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES semantic_map
   CATKIN_DEPENDS message_runtime sensor_msgs std_msgs cv_bridge semantic_map metaroom_xml_parser qt_build geometry_msgs pcl_ros
   DEPENDS PCL
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_manager_node include/object_manager.h src/object_manager.cpp src/object_manager_main.cpp)

add_dependencies(object_manager_node object_manager_node_generate_messages_cpp)


 target_link_libraries(object_manager_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
  )

############################# INSTALL TARGETS

install(TARGETS object_manager_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


