cmake_minimum_required(VERSION 2.8.3)
project(additional_view_registration_server)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs observation_registration_services metaroom_xml_parser semantic_map tf pcl_ros qt_build siftgpu)


catkin_package(
   INCLUDE_DIRS 
   CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs observation_registration_services metaroom_xml_parser semantic_map qt_build libqt4-dev pcl_ros siftgpu
   DEPENDS libceres-dev suitesparse
   LIBRARIES #observation_registration_optimizer
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


add_library(additional_view_registration_optimizer include/additional_view_registration_server/additional_view_registration_optimizer.h
                                               include/additional_view_registration_server/additional_view_registration_residual.h
                                               include/additional_view_registration_server/sift_wrapper.h
                                               src/additional_view_registration_optimizer.cpp
                                               src/sift_wrapper.cpp)

add_dependencies(additional_view_registration_optimizer observation_registration_services_gencpp semantic_map_gencpp)
target_link_libraries(additional_view_registration_optimizer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${CERES_LIBRARIES})

add_executable(additional_view_registration_server src/additional_view_registration_main.cpp)
target_link_libraries(additional_view_registration_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${CERES_LIBRARIES} additional_view_registration_optimizer)
add_dependencies(additional_view_registration_server observation_registration_services_gencpp semantic_map_gencpp)


add_executable(test_additional_view_registration test/test_additional_view_registration.cpp)
target_link_libraries(test_additional_view_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${CERES_LIBRARIES} additional_view_registration_optimizer)
add_dependencies(test_additional_view_registration observation_registration_services_gencpp semantic_map_gencpp)

