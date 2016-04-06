cmake_minimum_required(VERSION 2.8)
project(nbv_planning)

set(PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  tf_conversions
  eigen_conversions
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS cmake_modules ${PACKAGE_DEPENDENCIES} message_generation)

find_package(cmake_modules REQUIRED)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

find_package(Eigen REQUIRED)

find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_service_files(
  FILES
  SetTarget.srv
  SelectNextView.srv
  SetViews.srv
  Update.srv
)
generate_messages( DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime
        DEPENDS octomap eigen
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)
include_directories(
  include/octomap_server
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} src/SensorModel.cpp include/nbv_planning/SensorModel.h src/TargetVolume.cpp include/nbv_planning/TargetVolume.h src/NBVFinder.cpp include/nbv_planning/NBVFinder.h src/NBVFinderROS.cpp include/nbv_planning/NBVFinderROS.h include/nbv_planning/conversions.h src/Ray.cpp include/nbv_planning/Ray.h)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

add_executable(nbv_server src/nbv_planner_server.cpp )
target_link_libraries(nbv_server ${PROJECT_NAME} ${LINK_LIBS})

add_executable(nbv_pcds src/nbv_planner_pcds.cpp )
target_link_libraries(nbv_pcds ${PROJECT_NAME} ${LINK_LIBS} yaml-cpp )

# install targets:
install(TARGETS ${PROJECT_NAME}
        nbv_server
        nbv_pcds

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)


