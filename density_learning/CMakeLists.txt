cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(density_learning)

#set(CMAKE_C_COMPILER clang)
#set(CMAKE_CXX_COMPILER clang++)

add_definitions(-std=c++11 -O4)

set(retrieval_dir ${CMAKE_CURRENT_SOURCE_DIR}/../dynamic_object_retrieval)

# Show where to find the find package scripts
list(APPEND CMAKE_MODULE_PATH "${retrieval_dir}/cmake")

# Define the locations of the k_means_tree project
set(k_means_tree_dir ${CMAKE_CURRENT_SOURCE_DIR}/../k_means_tree)
set(k_means_tree_library_dir ${k_means_tree_dir}/build)
include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
link_directories(${k_means_tree_library_dir})

set(benchmark_dir ${CMAKE_CURRENT_SOURCE_DIR}/../benchmark)
set(benchmark_library_dir ${benchmark_dir}/build)
include_directories(${benchmark_dir}/include)
link_directories(${benchmark_library_dir})

# The main header location
include_directories(${retrieval_dir}/include)
link_directories(${retrieval_dir}/build)

set(stopwatch_dir ${retrieval_dir}/stopwatch)
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

# Be sure to set this to YOUR workspace, use custom version of metaroom_xml_parser for labeled data,
# https://github.com/RaresAmbrus/strands_3d_mapping, branch hydro-devel
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

find_package(OpenGL)

# The main header location
include_directories(include)

# DEMO: add_library
#add_executable(benchmark_query_vocabulary src/benchmark_query_vocabulary.cpp)
#target_link_libraries(benchmark_query_vocabulary benchmark_retrieval benchmark_result benchmark_visualization benchmark_overlap
#                      vocabulary_tree dynamic_visualize register_objects extract_sift sift pfhrgb_estimation shot_estimation
#                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_library(surfel_features src/surfel_features.cpp include/${PROJECT_NAME}/surfel_features.h)
target_link_libraries(surfel_features ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(compute_correct_matrix src/compute_correct_matrix.cpp)
target_link_libraries(compute_correct_matrix grouped_vocabulary_tree vocabulary_tree k_means_tree dynamic_visualize
                      register_objects extract_sift sift pfhrgb_estimation shot_estimation supervoxel_segmentation
                      benchmark_retrieval benchmark_visualization benchmark_overlap surfel_renderer surfel_features
                      ${OPENGL_gl_LIBRARY} ${QT_QTOPENGL_LIBRARY}
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(optimize_densities src/optimize_densities.cpp)
target_link_libraries(optimize_densities dynamic_visualize
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(precompute_labels src/precompute_labels.cpp)
target_link_libraries(precompute_labels benchmark_overlap
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(compute_density_features src/compute_density_features.cpp)
target_link_libraries(compute_density_features dynamic_visualize surfel_features
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

############### TESTING PROGRAMS ##################

add_executable(test_keypoint_threshold test/test_keypoint_threshold.cpp)
target_link_libraries(test_keypoint_threshold dynamic_visualize
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})
