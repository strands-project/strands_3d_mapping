cmake_minimum_required(VERSION 2.8.3)
project(object_view_generator)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    message_generation
    nav_msgs 
    rospy
    tf
    std_msgs
)

add_service_files(
  FILES
    GetTrajectoryPoints.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs
    message_generation
    nav_msgs 
    rospy
    tf
    std_msgs  
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(
    PROGRAMS scripts/view_points_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
