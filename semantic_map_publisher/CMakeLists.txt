cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_publisher)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs pcl_ros message_generation qt_build semantic_map metaroom_xml_parser octomap_msgs octomap_ros)
find_package(octomap REQUIRED)

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
   MetaroomService.srv
   ObservationService.srv
   ObservationOctomapService.srv
   WaypointInfoService.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  octomap_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS message_runtime sensor_msgs std_msgs octomap_msgs octomap_ros  semantic_map metaroom_xml_parser
   DEPENDS PCL octomap
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${cloud_register_INCLUDE_DIRS}
)

add_executable(semantic_map_publisher include/semantic_map_publisher.h src/semantic_map_publisher.cpp src/semantic_map_publisher_main.cpp)
add_dependencies(semantic_map_publisher semantic_map_generate_messages_cpp primitive_extraction_generate_messages_cpp semantic_map_publisher_generate_messages_cpp)


 target_link_libraries(semantic_map_publisher
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
  )

############################# INSTALL TARGETS

install(TARGETS semantic_map_publisher 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


