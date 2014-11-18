cmake_minimum_required(VERSION 2.8.3)
project(semantic_map)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS mongodb_store roscpp rospy std_msgs sensor_msgs pcl_ros message_generation qt_build image_geometry)

find_package(Boost REQUIRED COMPONENTS system thread program_options filesystem)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

set(CMAKE_PREFIX_PATH /usr/share/pcl-1.7/ ${CMAKE_PREFIX_PATH})

set(PCL_DIR /usr/share/pcl-1.7/)
find_package(PCL 1.7 REQUIRED NO_DEFAULT_PATH)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

add_message_files(
  FILES
  RoomObservation.msg
)

add_service_files(
   FILES
   MetaroomService.srv
   DynamicClusterService.srv
   ObservationService.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime sensor_msgs std_msgs 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${cloud_register_INCLUDE_DIRS}
)



set(HDRS
    include/semantic_map/constants.h
    include/semantic_map/room.h
    include/semantic_map/metaroom.h
    include/semantic_map/roombase.h
    include/semantic_map/room_xml_parser.h
    include/semantic_map/metaroom_xml_parser.h
    include/semantic_map/semantic_map_summary_parser.h
    include/semantic_map/occlusion_checker.h
    include/semantic_map/ndt_registration.h
)

set(SRCS
    src/room.cpp
    src/metaroom.cpp
    src/roombase.cpp
    src/room_xml_parser.cpp
    src/metaroom_xml_parser.cpp
    src/semantic_map_summary_parser.cpp
    src/occlusion_checker.cpp
    src/ndt_registration.cpp
)

add_library(semantic_map ${HDRS} ${SRCS})
add_executable(semantic_map_node src/semantic_map_node.cpp src/semantic_map_main.cpp)

add_executable(load_from_mongo src/load_from_mongo.cpp)

add_dependencies(semantic_map primitive_extraction_generate_messages_cpp strands_perception_msgs_generate_messages_cpp)
add_dependencies(semantic_map_node primitive_extraction_generate_messages_cpp strands_perception_msgs_generate_messages_cpp)

 target_link_libraries(semantic_map_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
  )

 target_link_libraries(load_from_mongo
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
  )


############################# INSTALL TARGETS

install(TARGETS semantic_map semantic_map_node load_from_mongo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/semantic_map/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


