cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(object_3d_benchmark)

add_definitions(-std=c++11 -O4)

# Show where to find the find package scripts
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake")

# Define the locations of the k_means_tree project
set(k_means_tree_dir ${CMAKE_CURRENT_SOURCE_DIR}/../k_means_tree)
set(k_means_tree_library_dir ${k_means_tree_dir}/build)
include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
link_directories(${k_means_tree_library_dir})

set(stopwatch_dir ${CMAKE_CURRENT_SOURCE_DIR}/../stopwatch)
set(stopwatch_include_dir ${stopwatch_dir}/src)
include_directories(${stopwatch_include_dir})

# Define the locations of cereal serialization header files
set(cereal_include_dir ${k_means_tree_dir}/cereal/include)
include_directories(${cereal_include_dir})

# Find PCL
find_package(PCL 1.6 REQUIRED COMPONENTS common io search visualization surface kdtree features surface segmentation octree filter keypoints)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Use custom version of metaroom_xml_parser for labeled data
set(parser_workspace ${CMAKE_CURRENT_SOURCE_DIR}/../../catkin_ws)
set(parser_include_dir ${parser_workspace}/src/strands_3d_mapping/metaroom_xml_parser/include)
set(parser_library_dir ${parser_workspace}/devel/lib)

include_directories(${parser_include_dir})
link_directories(${parser_library_dir})

# Find ROS
find_package(ROS REQUIRED)
include_directories(${ROS_INCLUDE_DIR})
link_directories(${ROS_LIBRARY_DIR})

# Find QT
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

# The main header location
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../build)
set(include_dir include/${PROJECT_NAME})

# DEMO: add_library
add_library(benchmark_retrieval src/benchmark_retrieval.cpp include/${PROJECT_NAME}/benchmark_retrieval.h)
target_link_libraries(benchmark_retrieval dynamic_visualize ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_library(benchmark_result src/benchmark_result.cpp include/${PROJECT_NAME}/benchmark_result.h)
target_link_libraries(benchmark_result ${PCL_LIBRARIES})

add_library(benchmark_visualization src/benchmark_visualization.cpp include/${PROJECT_NAME}/benchmark_visualization.h)
target_link_libraries(benchmark_visualization ${OpenCV_LIBS} ${PCL_LIBRARIES})

# DEMO: configure_file
#configure_file(scripts/menu.py menu.py COPYONLY)

add_executable(benchmark_query_vocabulary src/benchmark_query_vocabulary.cpp)
target_link_libraries(benchmark_query_vocabulary benchmark_retrieval benchmark_result benchmark_visualization
                      vocabulary_tree dynamic_visualize register_objects extract_sift sift pfhrgb_estimation
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

############### TESTING PROGRAMS ##################

add_executable(test_query_annotated_sweep test/test_query_annotated_sweep.cpp)
target_link_libraries(test_query_annotated_sweep benchmark_retrieval
                      vocabulary_tree dynamic_visualize register_objects extract_sift sift pfhrgb_estimation
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_visualize_query test/test_visualize_query.cpp)
target_link_libraries(test_visualize_query benchmark_retrieval benchmark_result
                      vocabulary_tree dynamic_visualize register_objects extract_sift sift pfhrgb_estimation
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_results_image test/test_results_image.cpp)
target_link_libraries(test_results_image benchmark_retrieval benchmark_result benchmark_visualization
                      vocabulary_tree dynamic_visualize register_objects extract_sift sift pfhrgb_estimation
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})
