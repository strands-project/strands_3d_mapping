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
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)
#include_directories(src/apps)
include_directories(include/FeatureDescriptor)
include_directories(include/mygeometry)
include_directories(include/FeatureExtractor)
include_directories(include/RGBDSegmentation)
include_directories(include/core)
#include_directories(include/graphcutting)
#include_directories(include/graph)
include_directories(include/TransformationFilter)
include_directories(include/FrameMatcher)
include_directories(include/Map)
#include_directories(include/g2otypes)

####################################################
##################### BUILD LIBS ###################
####################################################

add_library(frameinput_ekzpublic src/core/FrameInput.cpp)
target_link_libraries(frameinput_ekzpublic ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

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

install(TARGETS frameinput_ekzpublic FeatureDescriptor_ekzpublic FrameMatcher_ekzpublic TransformationFilter_ekzpublic core_ekzpublic map3d_ekzpublic RGBDSegmentation_ekzpublic FeatureExtractor_ekzpublic FeatureDescriptor_ekzpublic mygeometry_ekzpublic example_bow_images example_register_images_map example_register_images_standalone example_register_pcd_map example_register_pcd_standalone example_register_images_fast_map pcd_recorder image_recorder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


