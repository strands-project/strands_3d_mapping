cmake_minimum_required(VERSION 2.8)
project(semantic_map_to_2d)

set(PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  semantic_map_publisher
)

find_package(catkin REQUIRED COMPONENTS message_generation ${PACKAGE_DEPENDENCIES})

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)


add_service_files(
  DIRECTORY srv
  FILES ChangeWaypoint.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS message_runtime octomap
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
include_directories(
  include/octomap_server
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} src/SemanticMap2DServer.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_executable(semantic_map_2d_server src/semantic_map_2d_server.cpp)
target_link_libraries(semantic_map_2d_server ${PROJECT_NAME} ${LINK_LIBS})

# install targets:
install(TARGETS ${PROJECT_NAME}
  semantic_map_2d_server

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)


