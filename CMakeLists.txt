cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(OBJECT_DISCOVERY)

add_definitions(-std=c++11 -O4)

find_package(PCL 1.6 REQUIRED COMPONENTS common io search visualization surface kdtree features surface segmentation octree)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(VTK_DIR "PATH/TO/VTK/BUILD/DIRECTORY")
#find_package(VTK 5.8 REQUIRED)
#include(${VTK_USE_FILE})

add_library(vis_wrapper vis_wrapper.cpp lockable.cpp)
add_library(segment_features segment_features.cpp)
add_library(convex_voxel_segmentation convex_voxel_segmentation.cpp)

target_link_libraries(convex_voxel_segmentation ${PCL_LIBRARIES} ${VTK_LIBRARIES})

add_executable(browse browse_scenes.cpp)
target_link_libraries(browse ${PCL_LIBRARIES} vis_wrapper)

add_executable(browse_kinect browse_scenes_kinect.cpp)
target_link_libraries(browse_kinect ${PCL_LIBRARIES} vis_wrapper)

add_executable(pcd_to_mesh pcd_to_mesh.cpp)
target_link_libraries(pcd_to_mesh ${PCL_LIBRARIES} vis_wrapper)

add_executable(segment segment_scene.cpp)
target_link_libraries(segment ${PCL_LIBRARIES} vis_wrapper)

add_executable(segment_kinect segment_scene_kinect.cpp)
target_link_libraries(segment_kinect ${PCL_LIBRARIES} vis_wrapper)

add_executable(segment_voxel_kinect segment_voxel_kinect.cpp)
target_link_libraries(segment_voxel_kinect convex_voxel_segmentation ${PCL_LIBRARIES} ${VTK_LIBRARIES} vis_wrapper segment_features)

add_executable(annotate annotate_bag.cpp)
target_link_libraries(annotate ${PCL_LIBRARIES} vis_wrapper)

add_executable(cooccurrence cooccurrence.cpp)
target_link_libraries(cooccurrence ${PCL_LIBRARIES} vis_wrapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-old-style-cast")

