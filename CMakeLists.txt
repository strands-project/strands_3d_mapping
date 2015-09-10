cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(object_3d_retrieval)

include(ExternalProject)

add_definitions(-std=c++11 -O4)

# Show where to find the find package scripts
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Define the locations of the k_means_tree project
set(k_means_tree_dir ${CMAKE_CURRENT_SOURCE_DIR}/k_means_tree)
set(k_means_tree_library_dir ${k_means_tree_dir}/build)
include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
link_directories(${k_means_tree_library_dir})

# Define the locations of cereal serialization header files
set(cereal_include_dir ${k_means_tree_dir}/cereal/include)
include_directories(${cereal_include_dir})

# Compile the k_means_tree project, needed for this
ExternalProject_Add(k_means_tree_project
  DOWNLOAD_COMMAND ""
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/k_means_tree"
  BINARY_DIR ${k_means_tree_library_dir}
  BUILD_COMMAND ${CMAKE_MAKE_PROGRAM}
  INSTALL_COMMAND ""
)

# Find PCL
find_package(PCL 1.6 REQUIRED COMPONENTS common io search visualization surface kdtree features surface segmentation octree filter keypoints)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find ROS
find_package(ROS REQUIRED)
include_directories(${ROS_INCLUDE_DIR})
link_directories(${ROS_LIBRARY_DIR})

# Find QT
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

# If the debian package metaroom_xml_parser fails to work for you, get
# https://github.com/strands-project/strands_3d_mapping, compile in a catkin_ws
# and add something like the following:
#set(room_parser_dir /home/nbore/catkin_ws/src/strands_3d_mapping/metaroom_xml_parser)
#include_directories(${room_parser_dir}/include)
#link_directories(/home/nbore/catkin_ws/devel/lib)

# The main header location
include_directories(include)
set(include_dir include/${PROJECT_NAME})

# This library includes the old OpenCV SIFT library
add_library(sift include/sift/sift.cpp include/sift/sift.h)
# This library registers objects for us using SIFT
add_library(register_objects src/register_objects.cpp ${include_dir}/register_objects.h)
# This library provides a lot of convenience functions for dealing with VT
add_library(object_retrieval src/descriptor_config.cpp ${include_dir}/descriptor_config.h src/object_retrieval.cpp ${include_dir}/object_retrieval.h)
# This library does the convex segmentation
add_library(supervoxel_segmentation src/supervoxel_segmentation.cpp ${include_dir}/supervoxel_segmentation.h)
# This library computes PFHRGB features, among other things
add_library(pfhrgb_estimation src/pfhrgb_estimation.cpp ${include_dir}/pfhrgb_estimation.h)
# This library deals with the annotations in the data set
add_library(dataset_annotations src/dataset_annotations.cpp ${include_dir}/dataset_annotations.h)
# This is the library bringing it all together, for easy querying
add_library(retrieval_client src/retrieval_client.cpp ${include_dir}/retrieval_client.h)

# This library needs the k_means_tree compiled to work
add_dependencies(object_retrieval k_means_tree_project)

# Link all of our libraries
target_link_libraries(sift ${OpenCV_LIBS})
target_link_libraries(register_objects sift ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(object_retrieval vocabulary_tree reweighted_vocabulary_tree register_objects ${PCL_LIBRARIES})
target_link_libraries(dataset_annotations object_retrieval ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(supervoxel_segmentation ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(pfhrgb_estimation ${PCL_LIBRARIES})
target_link_libraries(retrieval_client object_retrieval sift supervoxel_segmentation pfhrgb_estimation k_means_tree ${OpenCV_LIBS} ${PCL_LIBRARIES})

# This executable does all the actual experiments
add_executable(test_retrieval src/test_retrieval.cpp)
target_link_libraries(test_retrieval object_retrieval retrieval_client dataset_annotations sift supervoxel_segmentation pfhrgb_estimation k_means_tree ${OpenCV_LIBS} ${PCL_LIBRARIES})

# This executable visualizes the query as part of a big map
add_executable(test_visualize_query src/test_visualize_query.cpp)
target_link_libraries(test_visualize_query object_retrieval retrieval_client dataset_annotations sift supervoxel_segmentation pfhrgb_estimation k_means_tree
                      ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${ROS_LIBRARIES})

# This executable visualizes the convex segmentation, interactively
add_executable(test_convex_segmentation src/test_convex_segmentation.cpp)
target_link_libraries(test_convex_segmentation supervoxel_segmentation pfhrgb_estimation ${PCL_LIBRARIES})

# This executable tests the convex segmentation on a merged room sweep
add_executable(test_sweep_segmentation src/test_sweep_segmentation.cpp)
target_link_libraries(test_sweep_segmentation supervoxel_segmentation pfhrgb_estimation
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

# This executable creates a folder structure from the sweeps
add_executable(create_scan_folders src/create_scan_folders.cpp)
target_link_libraries(create_scan_folders ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

# This executable splits the scans into convex segments and creates a folder hierarchy
add_executable(create_convex_folders src/create_convex_folders.cpp)
target_link_libraries(create_convex_folders object_retrieval retrieval_client)

# This executable extracts PFHRGB features for each of the convex segments
add_executable(create_convex_features src/create_convex_features.cpp)
target_link_libraries(create_convex_features object_retrieval retrieval_client)

# This executable precomputes SIFT features for all of the scans
add_executable(create_scan_sift_features src/create_scan_sift_features.cpp)
target_link_libraries(create_scan_sift_features object_retrieval retrieval_client)

# This executable further splits the convex segments into subsegments and saves the corresponding features
add_executable(create_subsegment_features src/create_subsegment_features.cpp)
target_link_libraries(create_subsegment_features object_retrieval retrieval_client)

# This executable precomputes vocabulary vectors for the subsegments
add_executable(create_subsegment_vocabulary_vectors src/create_subsegment_vocabulary_vectors.cpp)
target_link_libraries(create_subsegment_vocabulary_vectors object_retrieval retrieval_client)

# This executable precomputes vocabulary vectors for the subsegments
add_executable(train_vocabulary_tree src/train_vocabulary_tree.cpp)
target_link_libraries(train_vocabulary_tree object_retrieval retrieval_client)

add_executable(query_vocabulary src/query_vocabulary.cpp)
target_link_libraries(query_vocabulary object_retrieval retrieval_client dataset_annotations)

configure_file(scripts/menu.py menu.py COPYONLY)
configure_file(scripts/training_menu.py training_menu.py COPYONLY)
configure_file(scripts/querying_menu.py querying_menu.py COPYONLY)

##################### NEW DYNAMIC OBJECT RETRIEVAL VERSION ###########################

add_library(dynamic_visualize src/visualize.cpp include/dynamic_object_retrieval/visualize.h)
target_link_libraries(dynamic_visualize vocabulary_tree ${PCL_LIBRARIES})

add_library(extract_sift src/extract_sift.cpp include/extract_sift/extract_sift.h)
target_link_libraries(extract_sift dynamic_visualize sift ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_library(dynamic_retrieval src/dynamic_retrieval.cpp include/dynamic_object_retrieval/dynamic_retrieval.h
            include/dynamic_object_retrieval/summary_types.h include/dynamic_object_retrieval/summary_iterators.h)
target_link_libraries(dynamic_retrieval vocabulary_tree extract_sift ${PCL_LIBRARIES})

add_executable(dynamic_init_folders src/dynamic_init_folders.cpp)
target_link_libraries(dynamic_init_folders ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_convex_segmentation src/dynamic_convex_segmentation.cpp)
target_link_libraries(dynamic_convex_segmentation supervoxel_segmentation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_extract_convex_features src/dynamic_extract_convex_features.cpp)
target_link_libraries(dynamic_extract_convex_features pfhrgb_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_create_subsegments src/dynamic_create_subsegments.cpp)
target_link_libraries(dynamic_create_subsegments pfhrgb_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_init_vocabulary src/dynamic_init_vocabulary.cpp)
target_link_libraries(dynamic_init_vocabulary ${PCL_LIBRARIES})

add_executable(dynamic_train_vocabulary src/dynamic_train_vocabulary.cpp)
target_link_libraries(dynamic_train_vocabulary vocabulary_tree dynamic_visualize ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_query_vocabulary src/dynamic_query_vocabulary.cpp)
target_link_libraries(dynamic_query_vocabulary vocabulary_tree register_objects dynamic_visualize dynamic_retrieval extract_sift ${PCL_LIBRARIES})

add_executable(dynamic_extract_sift src/dynamic_extract_sift.cpp)
target_link_libraries(dynamic_extract_sift extract_sift  ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

# We are quite strict with the warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-old-style-cast")

