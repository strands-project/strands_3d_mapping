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
    include/constants.h
    include/room.h
    include/metaroom.h
    include/roombase.h
    include/roomXMLparser.h
    include/metaroomXMLparser.h
    include/semanticMapSummaryParser.h
    include/occlusionChecker.h
    include/ndtRegistration.h
)

set(SRCS
    src/room.cpp
    src/metaroom.cpp
    src/roombase.cpp
    src/roomXMLparser.cpp
    src/metaroomXMLparser.cpp
    src/semanticMapSummaryParser.cpp
    src/occlusionChecker.cpp
    src/ndtRegistration.cpp
)

add_library(semantic_map ${HDRS} ${SRCS})
add_executable(semantic_map_node include/semantic_map_node.h src/semantic_map_node.cpp src/semantic_map_main.cpp)

add_dependencies(semantic_map primitive_extraction_generate_messages_cpp strands_perception_msgs_generate_messages_cpp)
add_dependencies(semantic_map_node primitive_extraction_generate_messages_cpp strands_perception_msgs_generate_messages_cpp)

 target_link_libraries(semantic_map_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
  )

############################# INSTALL TARGETS

install(TARGETS semantic_map semantic_map_node 
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


