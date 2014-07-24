cmake_minimum_required(VERSION 2.8.3)
project(ekz_public_lib)

set(CMAKE_CXX_FLAGS "-O4 -g -pg -Wunknown-pragmas -Wno-unknown-pragmas -Wsign-compare ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS rosbag cv_bridge roscpp rospy std_msgs pcl_ros sensor_msgs)

##############################################
##################### INCLUDES ###############
##############################################

#find_package(catkin REQUIRED)
# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)

#OPENCV
link_directories(/opt/ros/hydro/lib)
include_directories(/opt/ros/hydro/include)
include_directories(/opt/ros/hydro/include/opencv)
include_directories(/opt/ros/hydro/include/opencv2)

#OPENCV
#link_directories(/opt/ros/groovy/lib)
#include_directories(/opt/ros/groovy/include)
#include_directories(/opt/ros/groovy/include/opencv)
#include_directories(/opt/ros/groovy/include/opencv2)

include_directories(src)
#include_directories(src/apps)
include_directories(src/FeatureDescriptor)
include_directories(src/mygeometry)
include_directories(src/FeatureExtractor)
include_directories(src/RGBDSegmentation)
include_directories(src/core)
#include_directories(src/graphcutting)
#include_directories(src/graph)
include_directories(src/TransformationFilter)
include_directories(src/FrameMatcher)
include_directories(src/Map)
#include_directories(src/g2otypes)

####################################################
##################### BUILD LIBS ###################
####################################################

add_library(frameinput_ekzpublic src/core/FrameInput.cpp)
target_link_libraries(frameinput_ekzpublic opencv_calib3d opencv_gpu opencv_nonfree opencv_ts opencv_contrib opencv_highgui opencv_objdetect opencv_video opencv_core opencv_imgproc opencv_photo opencv_videostab opencv_features2d opencv_legacy   opencv_stitching opencv_flann opencv_ml opencv_superres ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_library(FeatureDescriptor_ekzpublic
src/FeatureDescriptor/FeatureDescriptor.cpp
src/FeatureDescriptor/OrbFeatureDescriptor.cpp
src/FeatureDescriptor/SurfFeatureDescriptor64.cpp
src/FeatureDescriptor/SurfFeatureDescriptor128.cpp
src/FeatureDescriptor/FloatHistogramFeatureDescriptor.cpp )

add_library(mygeometry_ekzpublic src/mygeometry/HasDistance.cpp src/mygeometry/Point.cpp src/mygeometry/Line.cpp src/mygeometry/Plane.cpp src/mygeometry/PlaneChain.cpp src/mygeometry/KeyPoint.cpp src/mygeometry/KeyPointChain.cpp src/mygeometry/KeyPointSet.cpp)
target_link_libraries(mygeometry_ekzpublic FeatureDescriptor_ekzpublic frameinput_ekzpublic)

add_library(FeatureExtractor_ekzpublic 
src/FeatureExtractor/FeatureExtractor.cpp 
src/FeatureExtractor/OrbExtractor.cpp
src/FeatureExtractor/SurfExtractor.cpp)
target_link_libraries(FeatureExtractor_ekzpublic mygeometry_ekzpublic frameinput_ekzpublic)
target_link_libraries(FeatureExtractor_ekzpublic FeatureDescriptor_ekzpublic)
target_link_libraries(FeatureExtractor_ekzpublic ${PCL_LIBRARIES})
target_link_libraries(FeatureExtractor_ekzpublic ${OpenCV_LIBRARIES})
target_link_libraries(FeatureExtractor_ekzpublic opencv_calib3d opencv_core opencv_flann opencv_highgui opencv_legacy opencv_contrib opencv_features2d opencv_gpu  opencv_imgproc opencv_ml opencv_objdetect opencv_video)

add_library(RGBDSegmentation_ekzpublic src/RGBDSegmentation/RGBDSegmentation.cpp)

add_library(core_ekzpublic src/core/RGBDFrame.cpp src/core/Transformation.cpp)
target_link_libraries(core_ekzpublic RGBDSegmentation_ekzpublic frameinput_ekzpublic)
target_link_libraries(core_ekzpublic FeatureExtractor_ekzpublic)
target_link_libraries(core_ekzpublic ${catkin_LIBRARIES})

add_library(TransformationFilter_ekzpublic src/TransformationFilter/TransformationFilter.cpp)
target_link_libraries(TransformationFilter_ekzpublic core_ekzpublic)

add_library(FrameMatcher_ekzpublic src/FrameMatcher/FrameMatcher.cpp src/FrameMatcher/AICK.cpp src/FrameMatcher/BowAICK.cpp)
#target_link_libraries(FrameMatcher TransformationFilter)
#target_link_libraries(FrameMatcher graphcutting)
#target_link_libraries(FrameMatcher g2otypes g2oedges)

add_library(map3d_ekzpublic src/Map/Map3D.cpp src/Map/Map3Dbow.cpp)
target_link_libraries(map3d_ekzpublic core_ekzpublic FrameMatcher_ekzpublic)

#####################################################
##################### BUILD APPS ####################
#####################################################

#Generate bag of words model example
add_executable(example_bow_images src/apps/example_bow_images.cpp)
target_link_libraries(example_bow_images ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} core_ekzpublic map3d_ekzpublic)

#Register images example
add_executable(example_register_images_map src/apps/example_register_images_map.cpp)
target_link_libraries(example_register_images_map ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} core_ekzpublic map3d_ekzpublic)

add_executable(example_register_images_standalone src/apps/example_register_images_standalone.cpp)
target_link_libraries(example_register_images_standalone ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} core_ekzpublic map3d_ekzpublic)

#Register pcds examples
add_executable(example_register_pcd_map src/apps/example_register_pcd_map.cpp)
target_link_libraries(example_register_pcd_map ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} core_ekzpublic map3d_ekzpublic)

add_executable(example_register_pcd_standalone src/apps/example_register_pcd_standalone.cpp)
target_link_libraries(example_register_pcd_standalone ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} core_ekzpublic map3d_ekzpublic)

add_executable(example_register_images_fast_map src/apps/example_register_images_fast_map.cpp)
target_link_libraries(example_register_images_fast_map ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} core_ekzpublic map3d_ekzpublic)


#Recorders

add_executable(pcd_recorder src/apps/pcd_recorder.cpp)
target_link_libraries(pcd_recorder ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} core_ekzpublic map3d_ekzpublic)

add_executable(image_recorder src/apps/image_recorder.cpp)
target_link_libraries(image_recorder ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} core_ekzpublic map3d_ekzpublic)


catkin_package(
   INCLUDE_DIRS src
   LIBRARIES frameinput_ekzpublic FeatureDescriptor_ekzpublic FrameMatcher_ekzpublic TransformationFilter_ekzpublic core_ekzpublic map3d_ekzpublic RGBDSegmentation_ekzpublic FeatureExtractor_ekzpublic FeatureDescriptor_ekzpublic mygeometry_ekzpublic 
   CATKIN_DEPENDS 
   DEPENDS OpenCV PCL 
)


