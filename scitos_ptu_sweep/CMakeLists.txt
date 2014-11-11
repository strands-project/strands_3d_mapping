cmake_minimum_required(VERSION 2.8.3)
project(scitos_ptu_sweep)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs message_generation rospy sensor_msgs pcl_ros tf message_filters)
find_package(PCL 1.6 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_action_files(
    DIRECTORY action
    FILES PTUSweep.action
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs#   sensor_msgs
# )

generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs sensor_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
# INCLUDE_DIRS include
# LIBRARIES strands_human_aware_cmd_vel
# CATKIN_DEPENDS geometry_msgs message_filters roscpp strands_perception_people_msgs
  DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(scitos_PTU_sweep
#   src/${PROJECT_NAME}/scitos_PTU_sweep.cpp
# )

## Declare a cpp executable
add_executable(transform_pc2 src/transform_pc2.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(scitos_PTU_sweep_node scitos_PTU_sweep_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(transform_pc2
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS transform_pc2
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 install(PROGRAMS
   scripts/PTU_sweep.py scripts/PTU_test.py scripts/sweep_save.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

