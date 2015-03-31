cmake_minimum_required(VERSION 2.8.3)
project(calibrate_sweeps)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation qt_build semantic_map metaroom_xml_parser strands_sweep_registration)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")
set(CMAKE_PREFIX_PATH /usr/share/pcl-1.7/ ${CMAKE_PREFIX_PATH})
set(PCL_DIR /usr/share/pcl-1.7/)
find_package(PCL 1.7 REQUIRED NO_DEFAULT_PATH)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


rosbuild_prepare_qt4(QtCore QtXml)

add_action_files(
DIRECTORY
    action
FILES
    CalibrateSweeps.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)


catkin_package(
   INCLUDE_DIRS
   LIBRARIES 
   CATKIN_DEPENDS semantic_map metaroom_xml_parser strands_sweep_registration
   DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calibrate_sweep_as src/calibrate_sweep_action_server.cpp )
add_dependencies(calibrate_sweep_as calibrate_sweeps_generate_messages_cpp)

 target_link_libraries(calibrate_sweep_as
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
  )

############################# INSTALL TARGETS

install(TARGETS calibrate_sweep_as 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
