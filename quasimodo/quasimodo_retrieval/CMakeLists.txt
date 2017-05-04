cmake_minimum_required(VERSION 2.8.3)
project(quasimodo_retrieval)

add_definitions(-std=c++11 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf sensor_msgs std_msgs image_geometry metaroom_xml_parser quasimodo_msgs message_runtime
                                        message_generation tf_conversions pcl_ros image_geometry cv_bridge tf_conversions eigen_conversions
                                        k_means_tree dynamic_object_retrieval object_3d_benchmark dynamic_reconfigure soma_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(${catkin_INCLUDE_DIRS})

# This is the root dir of https://github.com/nilsbore/dynamic_object_retrieval
#set(retrieval_project_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../dynamic_object_retrieval)
#set(retrieval_project_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../../../object_3d_retrieval/)

# This is the main project, used for retrieval
#set(retrieval_dir ${retrieval_project_dir}/dynamic_object_retrieval)
#set(retrieval_libraries grouped_vocabulary_tree vocabulary_tree k_means_tree
#    dynamic_retrieval
#    dynamic_visualize register_objects extract_sift sift pfhrgb_estimation)
#include_directories(${retrieval_dir}/include)
#link_directories(${retrieval_dir}/build)

# This is the base vocabulary tree representation
#set(k_means_tree_dir ${retrieval_project_dir}/k_means_tree)
#set(k_means_tree_library_dir ${k_means_tree_dir}/build)
#include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
#link_directories(${k_means_tree_library_dir})

# This is the stopwatch by Whelan, also included in the base dir
#set(stopwatch_dir ${retrieval_dir}/stopwatch)
#set(stopwatch_include_dir ${stopwatch_dir}/src)
#include_directories(${stopwatch_include_dir})

find_package(OpenGL)

# The benchmark project is also in the base dir
#set(benchmark_dir ${retrieval_project_dir}/benchmark)
#set(benchmark_libraries benchmark_result benchmark_retrieval benchmark_visualization benchmark_overlap surfel_renderer ${OPENGL_gl_LIBRARY} ${QT_QTOPENGL_LIBRARY})
#include_directories(${benchmark_dir}/include)
#link_directories(${benchmark_dir}/build)

# Define the locations of cereal serialization header files
#set(cereal_include_dir ${k_means_tree_dir}/cereal/include)
#include_directories(${cereal_include_dir})

# Find PCL
find_package(PCL 1.6 REQUIRED COMPONENTS common io search visualization surface kdtree features surface segmentation octree filter keypoints)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find QT
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
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
#  INCLUDE_DIRS include
#  LIBRARIES quasimodo_retrieval
  CATKIN_DEPENDS metaroom_xml_parser
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(quasimodo_retrieval
#   src/${PROJECT_NAME}/quasimodo_retrieval.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(quasimodo_retrieval ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(quasimodo_retrieval_server src/quasimodo_retrieval_server.cpp)
add_executable(quasimodo_retrieval_publisher src/quasimodo_retrieval_publisher.cpp)
add_executable(quasimodo_visualization_server src/quasimodo_visualization_server.cpp)
add_executable(quasimodo_visualize_model src/quasimodo_visualize_model.cpp)
add_executable(quasimodo_visualize_retrieval_cloud src/quasimodo_visualize_retrieval_cloud.cpp)
add_executable(quasimodo_retrieval_node src/quasimodo_retrieval_node.cpp)
add_executable(quasimodo_retrieve_observation src/quasimodo_retrieve_observation.cpp)
add_executable(quasimodo_logging_server src/quasimodo_logging_server.cpp)

add_dependencies(quasimodo_retrieval_server quasimodo_msgs_generate_messages_cpp)
add_dependencies(quasimodo_retrieval_publisher quasimodo_msgs_generate_messages_cpp)
add_dependencies(quasimodo_visualization_server quasimodo_msgs_generate_messages_cpp)
add_dependencies(quasimodo_visualize_model quasimodo_msgs_generate_messages_cpp)
add_dependencies(quasimodo_retrieval_node quasimodo_msgs_generate_messages_cpp)
add_dependencies(quasimodo_retrieve_observation quasimodo_msgs_generate_messages_cpp)
add_dependencies(quasimodo_logging_server quasimodo_msgs_generate_messages_cpp soma_msgs_generate_messages_cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(quasimodo_retrieval_server ${catkin_EXPORTED_TARGETS})
#add_dependencies(quasimodo_retrieval_publisher ${catkin_EXPORTED_TARGETS})
#add_dependencies(quasimodo_visualization_server ${catkin_EXPORTED_TARGETS})
add_dependencies(quasimodo_retrieval_node ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(quasimodo_retrieval_server
#  ${retrieval_libraries}
#  ${benchmark_libraries}
  ${OpenCV_LIBS}
  ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(quasimodo_retrieval_publisher
#  ${retrieval_libraries}
#  ${benchmark_libraries}
  ${OpenCV_LIBS}
  ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(quasimodo_visualize_model
#  ${retrieval_libraries}
#  ${benchmark_libraries}
  ${OpenCV_LIBS}
  ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(quasimodo_visualize_retrieval_cloud
  ${OpenCV_LIBS}
  ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

#target_link_libraries(quasimodo_retrieval_publisher
#  pfhrgb_estimation extract_sift sift
#  benchmark_result benchmark_retrieval benchmark_visualization benchmark_overlap
#  dynamic_visualize register_objects
#  ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES}
#  ${catkin_LIBRARIES}
#  grouped_vocabulary_tree vocabulary_tree k_means_tree # The order here matters for some reason
#)

target_link_libraries(quasimodo_visualization_server
#  ${retrieval_libraries}
#  ${benchmark_libraries}
  ${OpenCV_LIBS}
  ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(quasimodo_retrieval_node
#  ${retrieval_libraries}
#  ${benchmark_libraries}
  ${OpenCV_LIBS}
  ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(quasimodo_retrieve_observation
#  ${retrieval_libraries}
#  ${benchmark_libraries}
  ${OpenCV_LIBS}
  ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(quasimodo_logging_server
#  ${retrieval_libraries}
#  ${benchmark_libraries}
  ${OpenCV_LIBS}
  ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(
  DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark executables and/or libraries for installation
install(TARGETS quasimodo_retrieval_server quasimodo_retrieval_publisher quasimodo_visualization_server
                quasimodo_visualize_model quasimodo_visualize_retrieval_cloud quasimodo_retrieval_node quasimodo_retrieve_observation quasimodo_logging_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
