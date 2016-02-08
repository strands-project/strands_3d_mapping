cmake_minimum_required(VERSION 2.8.3)
project(quasimodo_brain)

find_package(catkin REQUIRED COMPONENTS roscpp quasimodo_msgs quasimodo_models message_runtime pcl_ros metaroom_xml_parser eigen_conversions cv_bridge )

set(CMAKE_CXX_FLAGS "-O4 -g -pg -Wunknown-pragmas -Wno-unknown-pragmas -Wsign-compare -fPIC -std=c++0x -o popcnt -mssse3")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

##############################################
##################### INCLUDES ###############
##############################################
#rares library
rosbuild_prepare_qt4(QtCore QtXml)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(/home/johane/catkin_ws_dyn/devel/lib)

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Opencv stuff
link_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/include/)

#find_package(OpenCV)
#include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})

#QT
find_package(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

#ROS INDIGO STUFF
include_directories(/opt/ros/indigo/include)
link_directories(/opt/ros/indigo/lib)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_quasimodo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#include_directories(/home/johane/catkin_ws/src/g2o/include)
#target_link_libraries(/home/johane/catkin_ws/src/g2o/lib)

include_directories(${catkin_INCLUDE_DIRS})

add_library(quasimodo_ModelDatabase src/ModelDatabase/ModelDatabase.cpp src/ModelDatabase/ModelDatabaseBasic.cpp)
target_link_libraries(quasimodo_ModelDatabase ${catkin_LIBRARIES})

add_executable(			test_ModelDatabase src/test_ModelDatabase.cpp)
add_dependencies(		test_ModelDatabase roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	test_ModelDatabase quasimodo_ModelDatabase quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			brain_test_msgs src/test_msgs.cpp)
add_dependencies(		brain_test_msgs roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	brain_test_msgs quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			preload_data src/preload_data.cpp)
add_dependencies(		preload_data roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	preload_data quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


add_executable(			preload_chair_data src/preload_chair_data.cpp)
add_dependencies(		preload_chair_data roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	preload_chair_data quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			preload_test_data src/preload_test_data.cpp)
add_dependencies(		preload_test_data roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	preload_test_data quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


add_executable(			usepreload src/usepreload.cpp)
add_dependencies(		usepreload roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	usepreload quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			modelserver src/modelserver.cpp)
add_dependencies(		modelserver roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	modelserver quasimodo_ModelDatabase quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


add_executable(			modelserver2 src/modelserver2.cpp)
add_dependencies(		modelserver2 roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	modelserver2 quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			kinect_node src/kinect_node.cpp)
#add_dependencies(		kinect_node roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	kinect_node quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			pcd_node src/pcd_node.cpp)
add_dependencies(		pcd_node roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	pcd_node quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			graph_part src/graph_part.cpp)
add_dependencies(		graph_part roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	graph_part quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			super src/super.cpp)
add_dependencies(		super roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	super quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


add_executable(			bag_reader src/bag_reader.cpp)
add_dependencies(		bag_reader roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	bag_reader quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			make_ply2pcd src/make_ply2pcd.cpp)
add_dependencies(		make_ply2pcd roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	make_ply2pcd quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(			fb_node src/fb_node.cpp)
#add_dependencies(		fb_node roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	fb_node quasimodo_ModelUpdater2 image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			test_noise src/test_noise.cpp)
add_dependencies(		test_noise roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	test_noise quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

