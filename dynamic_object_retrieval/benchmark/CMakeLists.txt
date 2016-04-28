cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(object_3d_benchmark)

OPTION(NOCATKIN "Disable CATKIN compilation" OFF)

if (NOT NOCATKIN) # double negation, hell yeah
    find_package(catkin QUIET)
else()
    message("NOT USING CATKIN!")
endif()

#set(CMAKE_C_COMPILER clang)
#set(CMAKE_CXX_COMPILER clang++)

add_definitions(-std=c++11 -O3)

set(retrieval_dir ${CMAKE_CURRENT_SOURCE_DIR}/../dynamic_object_retrieval)

# Show where to find the find package scripts
list(APPEND CMAKE_MODULE_PATH "${retrieval_dir}/cmake")

# Define the locations of the k_means_tree project
# If using catkin, including it using catkin instead
if (catkin_FOUND)
    find_package(catkin REQUIRED COMPONENTS roscpp tf tf_conversions pcl_ros
                        metaroom_xml_parser object_manager k_means_tree
                        dynamic_object_retrieval convex_segmentation)
    set(ROS_LIBRARIES ${catkin_LIBRARIES})
    include_directories(${catkin_INCLUDE_DIRS})
else()
    # Define the locations of the k_means_tree project
    set(k_means_tree_dir ${CMAKE_CURRENT_SOURCE_DIR}/../k_means_tree)
    set(k_means_tree_library_dir ${k_means_tree_dir}/build)
    include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
    link_directories(${k_means_tree_library_dir})

    # The main header location
    include_directories(${retrieval_dir}/include)
    link_directories(${retrieval_dir}/build)

    # The location of the convex_segmentation library
    set(segmentation_dir ${CMAKE_CURRENT_SOURCE_DIR}/../convex_segmentation)
    include_directories(${segmentation_dir}/include)
    link_directories(${segmentation_dir}/build)

    set(stopwatch_dir ${retrieval_dir}/stopwatch)
    set(stopwatch_include_dir ${stopwatch_dir}/src)
    include_directories(${stopwatch_include_dir})

    # Define the locations of cereal serialization header files
    set(cereal_include_dir ${k_means_tree_dir}/cereal/include)
    include_directories(${cereal_include_dir})

    # Be sure to set this to YOUR workspace, use custom version of metaroom_xml_parser for labeled data,
    # https://github.com/RaresAmbrus/strands_3d_mapping, branch hydro-devel
    set(parser_workspace ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
    set(parser_include_dir ${parser_workspace}/src/strands_3d_mapping/metaroom_xml_parser/include)
    set(parser_library_dir ${parser_workspace}/devel/lib)
    include_directories(${parser_include_dir})
    link_directories(${parser_library_dir})

    # Find ROS
    find_package(ROS REQUIRED)
    include_directories(${ROS_INCLUDE_DIR})
    link_directories(${ROS_LIBRARY_DIR})
endif()

# Find PCL
find_package(PCL 1.6 REQUIRED COMPONENTS common io search visualization surface kdtree features surface segmentation octree filter keypoints)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find QT
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

# Find packages for the surfel_renderer
find_package(OpenGL)
find_package(OpenEXR)
find_package(PNG)
#${AQSIS_OPENEXR_INCLUDE_DIR}
include_directories(${QT_INCLUDES} ${OPENGL_INCLUDE_DIR})

if (catkin_FOUND)
    catkin_package(
        LIBRARIES benchmark_overlap benchmark_retrieval benchmark_result
                  surfel_renderer benchmark_visualization benchmark_segmentation
        INCLUDE_DIRS include
    )
endif()

# The main header location
include_directories(include)

# DEMO: add_library
add_library(benchmark_overlap src/benchmark_overlap include/${PROJECT_NAME}/benchmark_overlap.h)
target_link_libraries(benchmark_overlap ${PCL_LIBRARIES})

add_library(benchmark_retrieval src/benchmark_retrieval.cpp include/${PROJECT_NAME}/benchmark_retrieval.h)
target_link_libraries(benchmark_retrieval benchmark_overlap ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_library(benchmark_result src/benchmark_result.cpp include/${PROJECT_NAME}/benchmark_result.h)
target_link_libraries(benchmark_result ${PCL_LIBRARIES})

qt4_wrap_cpp(surfel_renderer_SRC include/${PROJECT_NAME}/surfel_renderer.h)
add_library(surfel_renderer src/surfel_renderer.cpp include/${PROJECT_NAME}/surfel_renderer.h
            ${surfel_renderer_SRC})
target_link_libraries(surfel_renderer ${OPENGL_gl_LIBRARY} ${OpenCV_LIBS} ${QT_QTCORE_LIBRARY} ${QT_QTMAIN_LIBRARY}
                      ${QT_LIBRARIES} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${PCL_LIBRARIES})

add_library(benchmark_visualization src/benchmark_visualization.cpp include/${PROJECT_NAME}/benchmark_visualization.h)
target_link_libraries(benchmark_visualization surfel_renderer ${ROS_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(benchmark_segmentation src/benchmark_segmentation.cpp include/${PROJECT_NAME}/benchmark_segmentation.h)
target_link_libraries(benchmark_segmentation benchmark_overlap benchmark_retrieval 
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})


# DEMO: configure_file
#configure_file(scripts/menu.py menu.py COPYONLY)

add_executable(benchmark_query_vocabulary src/benchmark_query_vocabulary.cpp)
target_link_libraries(benchmark_query_vocabulary benchmark_retrieval benchmark_result benchmark_visualization benchmark_overlap 
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(benchmark_convex_segmentation src/benchmark_convex_segmentation.cpp)
target_link_libraries(benchmark_convex_segmentation benchmark_segmentation 
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(benchmark_incremental_segmentation src/benchmark_incremental_segmentation.cpp)
target_link_libraries(benchmark_incremental_segmentation benchmark_segmentation benchmark_overlap 
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

############### TESTING PROGRAMS ##################

add_executable(test_query_annotated_sweep test/test_query_annotated_sweep.cpp)
target_link_libraries(test_query_annotated_sweep benchmark_retrieval
                      
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_visualize_query test/test_visualize_query.cpp)
target_link_libraries(test_visualize_query benchmark_retrieval benchmark_result
                      
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_results_image test/test_results_image.cpp)
target_link_libraries(test_results_image benchmark_retrieval benchmark_result benchmark_visualization                      
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_static_matching test/test_static_matching.cpp)
target_link_libraries(test_static_matching benchmark_retrieval benchmark_result benchmark_visualization benchmark_overlap
                      
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_renderer test/test_renderer.cpp)
target_link_libraries(test_renderer surfel_renderer benchmark_retrieval
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_summarize_benchmark test/test_summarize_benchmark)
target_link_libraries(test_summarize_benchmark benchmark_result ${PCL_LIBRARIES})

if (catkin_FOUND)
    # Mark cpp header files for installation
    install(DIRECTORY include/object_3d_benchmark
      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION} # ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    # Mark executables and/or libraries for installation
    install(TARGETS benchmark_overlap benchmark_retrieval benchmark_result surfel_renderer benchmark_visualization benchmark_segmentation
                    benchmark_query_vocabulary benchmark_convex_segmentation benchmark_incremental_segmentation
                    test_query_annotated_sweep test_visualize_query test_results_image test_static_matching test_renderer test_summarize_benchmark
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()
