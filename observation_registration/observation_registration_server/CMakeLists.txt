cmake_minimum_required(VERSION 2.8.3)
project(observation_registration_server)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs observation_registration_services metaroom_xml_parser semantic_map tf pcl_ros qt_build siftgpu)


catkin_package(
   INCLUDE_DIRS 
   CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs observation_registration_services metaroom_xml_parser semantic_map qt_build libqt4-dev pcl_ros siftgpu
   DEPENDS libceres-dev suitesparse
   LIBRARIES observation_registration_optimizer
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS}
                    include)


add_library(observation_registration_optimizer include/observation_registration_server/observation_registration_optimizer.h
                                               include/observation_registration_server/observation_residual.h
                                               include/observation_registration_server/sift_wrapper.h
                                               src/observation_registration_optimizer.cpp
                                               src/sift_wrapper.cpp)
add_dependencies(observation_registration_optimizer observation_registration_services_gencpp semantic_map_gencpp)
target_link_libraries(observation_registration_optimizer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${CERES_LIBRARIES})

add_executable(observation_registration_server src/observation_registration_main.cpp)
target_link_libraries(observation_registration_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${CERES_LIBRARIES} observation_registration_optimizer)
add_dependencies(observation_registration_server observation_registration_services_gencpp semantic_map_gencpp)


add_executable(test_observation_registration test/test_observation_registration.cpp)
target_link_libraries(test_observation_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${CERES_LIBRARIES} observation_registration_optimizer)
add_dependencies(observation_registration_server observation_registration_services_gencpp semantic_map_gencpp)

