cmake_minimum_required(VERSION 2.8.3)
project(observation_registration_launcher)

find_package(catkin REQUIRED COMPONENTS roscpp metaroom_xml_parser qt_build additional_view_registration_server observation_registration_server observation_registration_services siftgpu)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   CATKIN_DEPENDS roscpp metaroom_xml_parser additional_view_registration_server observation_registration_server observation_registration_services siftgpu
   DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


############################# INSTALL TARGETS

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
