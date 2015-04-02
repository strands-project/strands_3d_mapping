cmake_minimum_required(VERSION 2.8.3)
project(semantic_map)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS mongodb_store roscpp rospy std_msgs sensor_msgs pcl_ros message_generation qt_build image_geometry cv_bridge tf_conversions strands_sweep_registration metaroom_xml_parser)
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

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES semantic_map
   CATKIN_DEPENDS message_runtime sensor_msgs std_msgs cv_bridge mongodb_store tf_conversions strands_sweep_registration pcl_ros image_geometry qt_build
   DEPENDS PCL qt_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${cloud_register_INCLUDE_DIRS}
)



set(HDRS
    include/semantic_map/constants.h
    include/semantic_map/room.h    
    include/semantic_map/metaroom.h
    include/semantic_map/metaroom_update_iteration.h
    include/semantic_map/roombase.h
    include/semantic_map/room_xml_parser.h
    include/semantic_map/metaroom_xml_parser.h
    include/semantic_map/semantic_map_summary_parser.h
    include/semantic_map/occlusion_checker.h
    include/semantic_map/ndt_registration.h
    include/semantic_map/reg_features.h
    include/semantic_map/reg_transforms.h
    include/semantic_map/room_utilities.h
    include/semantic_map/mongodb_interface.h
)

set(HDRS_IMPL
    include/semantic_map/room.hpp
    include/semantic_map/room_xml_parser.hpp
    include/semantic_map/metaroom_update_iteration.hpp
    include/semantic_map/metaroom.hpp
    include/semantic_map/metaroom_xml_parser.hpp
    include/semantic_map/roombase.hpp
    )

set(SRCS
    src/room.cpp
    src/metaroom.cpp
    src/metaroom_update_iteration.cpp
    src/roombase.cpp
    src/room_xml_parser.cpp
    src/metaroom_xml_parser.cpp
    src/semantic_map_summary_parser.cpp
    src/occlusion_checker.cpp
    src/ndt_registration.cpp
    src/reg_features.cpp
    src/reg_transforms.cpp
    src/room_utilities.cpp
    src/mongodb_interface.cpp
)

add_library(semantic_map ${HDRS} ${HDRS_IMPL} ${SRCS})
add_executable(semantic_map_node include/semantic_map/semantic_map_node.h src/semantic_map_node.cpp src/semantic_map_main.cpp)
add_executable(load_from_mongo src/load_from_mongo.cpp)
add_executable(add_to_mongo src/add_to_mongo.cpp)

add_dependencies(semantic_map semantic_map_generate_messages_cpp primitive_extraction_generate_messages_cpp strands_perception_msgs_generate_messages_cpp)
add_dependencies(semantic_map_node semantic_map_generate_messages_cpp primitive_extraction_generate_messages_cpp strands_perception_msgs_generate_messages_cpp)

 target_link_libraries(semantic_map
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
 )

 target_link_libraries(semantic_map_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
   semantic_map
  )

 target_link_libraries(load_from_mongo
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
   semantic_map
  )

 target_link_libraries(add_to_mongo
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${Boost_LIBRARIES}
   semantic_map
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


