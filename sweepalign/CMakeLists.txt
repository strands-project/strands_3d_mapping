cmake_minimum_required(VERSION 2.8.3)
project(strands_room)



set(CMAKE_CXX_FLAGS "-O4 -g -pg -Wunknown-pragmas -Wno-unknown-pragmas -Wsign-compare -fPIC -std=c++0x -o popcnt -mssse3")

find_package(catkin REQUIRED COMPONENTS
	metaroom_xml_parser
#	semantic_map
	pcl_ros
        image_geometry
        cpp_common
        roscpp
        rosconsole
        tf_conversions
        actionlib
        actionlib_msgs
)

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   INCLUDE_DIRS src src/Camera src/Camera/PixelFunction src/Frame src/Sweep
   CATKIN_DEPENDS metaroom_xml_parser image_geometry cpp_common roscpp rosconsole tf_conversions
   DEPENDS Ceres
   LIBRARIES PixelFunction_strands_room Camera_strands_room  Frame_strands_room Sweep_strands_room util_strands_room pair3DError_strands_room ProblemFrameConnection_strands_room RobotContainer_strands_room
)

include_directories(${catkin_INCLUDE_DIRS})

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Opencv stuff
link_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/include/)

#QT
find_package(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

#ROS INDIGO STUFF
include_directories(/opt/ros/indigo/include)
link_directories(/opt/ros/indigo/lib)

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

include_directories(src)
include_directories(src/Camera)
include_directories(src/Camera/PixelFunction)
include_directories(src/Frame)
include_directories(src/Match)
include_directories(src/PoseTransform)
include_directories(src/Sweep)



#PixelFunction is a per pixel library for updating the pixelvalues individually. Will learn per pixel biases and fix those
add_library(PixelFunction_strands_room src/Camera/PixelFunction/PixelFunction.cpp)

#Keeps camera parameters etc
add_library(Camera_strands_room src/Camera/Camera.cpp)
target_link_libraries(Camera_strands_room PixelFunction_strands_room ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_library(Frame_strands_room src/Frame/Frame.cpp)
target_link_libraries(Frame_strands_room Camera_strands_room  ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_library(Sweep_strands_room src/Sweep/Sweep.cpp)
target_link_libraries(Sweep_strands_room Frame_strands_room ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_library(util_strands_room src/util.cpp)
target_link_libraries(util_strands_room ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_library(pair3DError_strands_room src/pair3DError.cpp)
target_link_libraries(pair3DError_strands_room util_strands_room ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_library(ProblemFrameConnection_strands_room src/ProblemFrameConnection.cpp)
target_link_libraries(ProblemFrameConnection_strands_room pair3DError_strands_room Frame_strands_room ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_library(RobotContainer_strands_room src/RobotContainer.cpp)
target_link_libraries(RobotContainer_strands_room ProblemFrameConnection_strands_room pair3DError_strands_room util_strands_room Sweep_strands_room ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

###########################
##APPS#####################
###########################

#add_executable(room_reader_rares src/apps/reader_rares.cpp )
#target_link_libraries(room_reader_rares RobotContainer_strands_room Sweep_strands_room image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable(reader src/apps/reader.cpp )
#target_link_libraries(reader RobotContainer_strands_room Sweep_strands_room ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})



