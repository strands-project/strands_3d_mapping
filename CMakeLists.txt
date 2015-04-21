cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(object_3d_retrieval)

add_definitions(-std=c++11 -O4)

find_package(PCL 1.6 REQUIRED COMPONENTS common io search visualization surface kdtree features surface segmentation octree filter keypoints)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

#set(VTK_DIR "PATH/TO/VTK/BUILD/DIRECTORY")
#find_package(VTK 5.8 REQUIRED)
#include(${VTK_USE_FILE})

set(k_means_tree_dir /home/nbore/Workspace/k_means_tree)
include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
link_directories(${k_means_tree_dir}/build)

set(room_parser_dir /home/nbore/catkin_ws/src/strands_3d_mapping/metaroom_xml_parser)
include_directories(${room_parser_dir}/include)
include_directories(/opt/ros/indigo/include)
link_directories(/home/nbore/catkin_ws/devel/lib)
link_directories(/opt/ros/indigo/lib)

include_directories(include) # cereal, object_3d_retrieval
set(include_dir include/${PROJECT_NAME})

add_library(segment_features src/segment_features.cpp ${include_dir}/segment_features.h)
add_library(convex_voxel_segmentation src/convex_voxel_segmentation.cpp ${include_dir}/convex_voxel_segmentation.h)
add_library(sift include/sift/sift.cpp include/sift/sift.h)
add_library(register_objects src/register_objects.cpp ${include_dir}/register_objects.h)
add_library(object_retrieval src/object_retrieval.cpp ${include_dir}/object_retrieval.h)
add_library(dataset_convenience src/dataset_convenience.cpp ${include_dir}/dataset_convenience.h)
#add_library(train_metric src/train_metric.cpp ${include_dir}/train_metric.h)
add_library(supervoxel_segmentation src/supervoxel_segmentation.cpp ${include_dir}/supervoxel_segmentation.h)
add_library(pfhrgb_estimation src/pfhrgb_estimation.cpp ${include_dir}/pfhrgb_estimation.h)

target_link_libraries(convex_voxel_segmentation ${PCL_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries(sift ${OpenCV_LIBS})
target_link_libraries(segment_features sift ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(register_objects sift ${OpenCV_LIBS} ${PCL_LIBRARIES})

target_link_libraries(object_retrieval convex_voxel_segmentation segment_features vocabulary_tree reweighted_vocabulary_tree register_objects ${PCL_LIBRARIES})
target_link_libraries(dataset_convenience object_retrieval metaroom_xml_parser image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(supervoxel_segmentation ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(pfhrgb_estimation ${PCL_LIBRARIES})

#add_executable(test_metric_map src/test_metric_map.cpp)
#target_link_libraries(test_metric_map object_retrieval metaroom_xml_parser image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(test_registration src/test_registration.cpp)
#target_link_libraries(test_registration register_objects ${PCL_LIBRARIES})

#add_executable(test_rgbd_dataset src/test_rgbd_dataset.cpp)
#target_link_libraries(test_rgbd_dataset object_retrieval ${OpenCV_LIBS} ${PCL_LIBRARIES})

#add_executable(test_validate_rgbd src/test_validate_rgbd.cpp)
#target_link_libraries(test_validate_rgbd object_retrieval ${PCL_LIBRARIES})

#add_executable(test_instances src/test_instances.cpp)
#target_link_libraries(test_instances object_retrieval metaroom_xml_parser image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(test_metric_map_incremental src/test_metric_map_incremental.cpp)
#target_link_libraries(test_metric_map_incremental object_retrieval metaroom_xml_parser image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(test_annotate src/test_annotate.cpp)
#target_link_libraries(test_annotate object_retrieval metaroom_xml_parser image_geometry cpp_common roscpp rosconsole tf_conversions ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(test_generate_segmentation_image src/test_generate_segmentation_image.cpp)
#target_link_libraries(test_generate_segmentation_image convex_voxel_segmentation metaroom_xml_parser image_geometry cpp_common roscpp rosconsole tf_conversions ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(test_reweight_results src/test_reweight_results.cpp)
#target_link_libraries(test_reweight_results object_retrieval metaroom_xml_parser image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(test_count_annotated src/test_count_annotated.cpp)
#target_link_libraries(test_count_annotated dataset_convenience ${PCL_LIBRARIES})

add_executable(create_scan_folders src/create_scan_folders.cpp)
target_link_libraries(create_scan_folders metaroom_xml_parser image_geometry cpp_common roscpp rosconsole tf_conversions ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})
#add_executable(test_global_search src/test_global_search.cpp)
#target_link_libraries(test_global_search dataset_convenience sift supervoxel_segmentation pfhrgb_estimation k_means_tree ${OpenCV_LIBS} ${PCL_LIBRARIES})
#add_executable(test_partial_search src/test_partial_search.cpp)
#target_link_libraries(test_partial_search dataset_convenience sift supervoxel_segmentation pfhrgb_estimation k_means_tree ${OpenCV_LIBS} ${PCL_LIBRARIES})
add_executable(test_retrieval src/test_retrieval.cpp)
target_link_libraries(test_retrieval dataset_convenience sift supervoxel_segmentation pfhrgb_estimation k_means_tree ${OpenCV_LIBS} ${PCL_LIBRARIES})
#add_executable(test_sliding_window src/test_sliding_window.cpp)
#target_link_libraries(test_sliding_window dataset_convenience sift supervoxel_segmentation pfhrgb_estimation k_means_tree ${OpenCV_LIBS} ${PCL_LIBRARIES})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-old-style-cast")

