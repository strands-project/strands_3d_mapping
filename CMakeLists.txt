cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(object_3d_retrieval)

add_definitions(-std=c++11 -O4)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(PCL 1.6 REQUIRED COMPONENTS common io search visualization surface kdtree features surface segmentation octree filter keypoints)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(ROS REQUIRED)
include_directories(${ROS_INCLUDE_DIR})
link_directories(${ROS_LIBRARY_DIR})

find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

set(k_means_tree_dir /home/nbore/Workspace/k_means_tree)
include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
link_directories(${k_means_tree_dir}/build)

# THIS WILL GET REPLACED BY THE RELEASED VERSION, PART OF ROS INSTALLATION
set(room_parser_dir /home/nbore/catkin_ws/src/strands_3d_mapping/metaroom_xml_parser)
include_directories(${room_parser_dir}/include)
link_directories(/home/nbore/catkin_ws/devel/lib)

include_directories(include) # cereal, object_3d_retrieval
set(include_dir include/${PROJECT_NAME})

add_library(sift include/sift/sift.cpp include/sift/sift.h)
add_library(register_objects src/register_objects.cpp ${include_dir}/register_objects.h)
add_library(object_retrieval src/descriptor_config.cpp ${include_dir}/descriptor_config.h src/object_retrieval.cpp ${include_dir}/object_retrieval.h)
add_library(supervoxel_segmentation src/supervoxel_segmentation.cpp ${include_dir}/supervoxel_segmentation.h)
add_library(pfhrgb_estimation src/pfhrgb_estimation.cpp ${include_dir}/pfhrgb_estimation.h)
add_library(dataset_annotations src/dataset_annotations.cpp ${include_dir}/dataset_annotations.h)
add_library(retrieval_client src/retrieval_client.cpp ${include_dir}/retrieval_client.h)

target_link_libraries(sift ${OpenCV_LIBS})
target_link_libraries(register_objects sift ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(object_retrieval vocabulary_tree reweighted_vocabulary_tree register_objects ${PCL_LIBRARIES})
target_link_libraries(dataset_annotations object_retrieval ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(supervoxel_segmentation ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(pfhrgb_estimation ${PCL_LIBRARIES})
target_link_libraries(retrieval_client object_retrieval sift supervoxel_segmentation pfhrgb_estimation k_means_tree ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(create_scan_folders src/create_scan_folders.cpp)
target_link_libraries(create_scan_folders metaroom_xml_parser ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_retrieval src/test_retrieval.cpp)
target_link_libraries(test_retrieval object_retrieval retrieval_client dataset_annotations sift supervoxel_segmentation pfhrgb_estimation k_means_tree ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(test_visualize_query src/test_visualize_query.cpp)
target_link_libraries(test_visualize_query object_retrieval retrieval_client dataset_annotations sift supervoxel_segmentation pfhrgb_estimation k_means_tree
                      ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES} metaroom_xml_parser ${ROS_LIBRARIES})


add_executable(test_convex_segmentation src/test_convex_segmentation.cpp)
target_link_libraries(test_convex_segmentation supervoxel_segmentation pfhrgb_estimation ${PCL_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-old-style-cast")

