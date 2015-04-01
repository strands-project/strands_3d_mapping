cmake_minimum_required(VERSION 2.8.3)
project(object_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs pcl_ros message_generation qt_build cv_bridge semantic_map metaroom_xml_parser)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
message (STATUS “OpenMP found”)
set(CMAKE_CXX_FLAGS "${OpenMP_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${OpenMP_C_FLAGS} ${CMAKE_C_FLAGS}")
else(OPENMP_FOUND)
message (STATUS “OpenMP not found”)
endif()

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

set(HDRS

    include/dynamic_object.h
    include/dynamic_object_utilities.h
    include/dynamic_object_xml_parser.h
    include/dynamic_object_mongodb_interface.h
)

set(SRCS

    src/dynamic_object.cpp
    src/dynamic_object_utilities.cpp
    src/dynamic_object_xml_parser.cpp
    src/dynamic_object_mongodb_interface.cpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_manager_node   ${HDRS} ${SRCS} include/object_manager.h src/object_manager.cpp src/object_manager_main.cpp)
add_dependencies(object_manager_node object_manager_node_generate_messages_cpp)

add_executable(load_objects_from_mongo ${HDRS} ${SRCS} src/load_objects_from_mongo.cpp)
add_dependencies(load_objects_from_mongo load_objects_from_mongo_generate_messages_cpp)

 target_link_libraries(object_manager_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
  )

 target_link_libraries(load_objects_from_mongo
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
  )

############################# INSTALL TARGETS

install(TARGETS object_manager_node load_objects_from_mongo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

