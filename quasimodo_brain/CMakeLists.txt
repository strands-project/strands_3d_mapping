cmake_minimum_required(VERSION 2.8.3)
project(quasimodo_brain)

find_package(catkin REQUIRED COMPONENTS roscpp soma2_msgs soma_manager quasimodo_msgs quasimodo_models message_runtime pcl_ros metaroom_xml_parser eigen_conversions cv_bridge)

set(CMAKE_CXX_FLAGS "-O4 -g -pg -Wunknown-pragmas -Wno-unknown-pragmas -Wsign-compare -fPIC -std=c++0x -o popcnt -mssse3")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Required for retrieval stuff
include_directories(${catkin_INCLUDE_DIRS})

# This is the root dir of https://github.com/nilsbore/dynamic_object_retrieval
set(retrieval_project_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../dynamic_object_retrieval/)


# This is the base vocabulary tree representation
#set(k_means_tree_dir ${retrieval_project_dir}/k_means_tree)
#set(k_means_tree_library_dir ${k_means_tree_dir}/build)
#include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
#link_directories(${k_means_tree_library_dir})

# This is the stopwatch by Whelan, also included in the base dir
#set(stopwatch_dir ${retrieval_dir}/stopwatch)
#set(stopwatch_include_dir ${stopwatch_dir}/src)
#include_directories(${stopwatch_include_dir})

find_package(OpenGL)

# The benchmark project is also in the base dir
#set(benchmark_dir ${retrieval_project_dir}/benchmark)
#set(benchmark_libraries benchmark_result benchmark_retrieval benchmark_visualization benchmark_overlap surfel_renderer ${OPENGL_gl_LIBRARY} ${QT_QTOPENGL_LIBRARY})
#include_directories(${benchmark_dir}/include)
#link_directories(${benchmark_dir}/build)

# Define the locations of cereal serialization header files
#set(cereal_include_dir ${k_means_tree_dir}/cereal/include)
#include_directories(${cereal_include_dir})

##############################################
##################### INCLUDES ###############
##############################################
#rares library
rosbuild_prepare_qt4(QtCore QtXml)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
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

#Ceres
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_quasimodo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#include_directories(/home/johane/catkin_ws/src/g2o/include)
#target_link_libraries(/home/johane/catkin_ws/src/g2o/lib)

include_directories(${catkin_INCLUDE_DIRS})

add_library(quasimodo_ModelDatabase src/ModelDatabase/ModelDatabase.cpp src/ModelDatabase/ModelDatabaseBasic.cpp)# src/ModelDatabase/ModelDatabaseRetrieval.cpp)
target_link_libraries(quasimodo_ModelDatabase
					  #${retrieval_libraries}
					  #${benchmark_libraries}
                      ${OpenCV_LIBS}
                      ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES})

#add_executable(			test_ModelDatabase src/test_ModelDatabase.cpp)
#add_dependencies(		test_ModelDatabase roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	test_ModelDatabase quasimodo_ModelDatabase quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			brain_test_msgs src/test_msgs.cpp)
#add_dependencies(		brain_test_msgs roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	brain_test_msgs quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			preload_data src/preload_data.cpp)
#add_dependencies(		preload_data roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	preload_data quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(			preload_chair_data src/preload_chair_data.cpp)
#add_dependencies(		preload_chair_data roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	preload_chair_data quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			preload_object_data src/preload_object_data.cpp)
target_link_libraries(	preload_object_data ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} metaroom_xml_parser)
add_dependencies(		preload_object_data roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	preload_object_data quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(		robot_listener src/robot_listener.cpp)
target_link_libraries(	robot_listener ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} metaroom_xml_parser)
add_dependencies(	robot_listener roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	robot_listener quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(			load_additional_views src/load_additional_views.cpp)
#add_dependencies(		load_additional_views roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	load_additional_views quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			preload_test_data src/preload_test_data.cpp)
#add_dependencies(		preload_test_data roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	preload_test_data quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			usepreload src/usepreload.cpp)
#add_dependencies(		usepreload roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	usepreload quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(			modelserver src/modelserver.cpp)
add_dependencies(		modelserver roscpp quasimodo_msgs_generate_messages_cpp)
target_link_libraries(	modelserver quasimodo_ModelDatabase quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(			MeshTest src/MeshTest.cpp)
#add_dependencies(		MeshTest roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	MeshTest quasimodo_ModelDatabase quasimodo_Mesher quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			modelserver2 src/modelserver2.cpp)
#add_dependencies(		modelserver2 roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	modelserver2 quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			kinect_node src/kinect_node.cpp)
#add_dependencies(		kinect_node roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	kinect_node quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			build_dataset_node src/build_dataset_node.cpp)
#add_dependencies(		build_dataset_node roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	build_dataset_node quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			load_dataset_node src/load_dataset_node.cpp)
#add_dependencies(		load_dataset_node roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	load_dataset_node quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			pcd_node src/pcd_node.cpp)
#add_dependencies(		pcd_node roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	pcd_node quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			graph_part src/graph_part.cpp)
#add_dependencies(		graph_part roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	graph_part quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			super src/super.cpp)
#add_dependencies(		super roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	super quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(			bag_reader src/bag_reader.cpp)
#add_dependencies(		bag_reader roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	bag_reader quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			make_ply2pcd src/make_ply2pcd.cpp)
#add_dependencies(		make_ply2pcd roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	make_ply2pcd quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(		fb_node src/fb_node.cpp)
#add_dependencies(		fb_node roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	fb_node quasimodo_ModelUpdater2 image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			test_noise src/test_noise.cpp)
#add_dependencies(		test_noise roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	test_noise quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(			test_align src/test_align.cpp)
#add_dependencies(		test_align roscpp quasimodo_msgs_generate_messages_cpp)
#target_link_libraries(	test_align  ${CERES_LIBRARIES} quasimodo_ModelUpdater image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
#install(TARGETS quasimodo_ModelDatabase test_ModelDatabase brain_test_msgs preload_data preload_chair_data preload_object_data preload_test_data
#                usepreload modelserver MeshTest modelserver2 kinect_node build_dataset_node load_dataset_node pcd_node
#                graph_part super bag_reader make_ply2pcd fb_node test_noise test_align
install(TARGETS quasimodo_ModelDatabase preload_object_data modelserver robot_listener
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/ModelDatabase
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION} # ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
