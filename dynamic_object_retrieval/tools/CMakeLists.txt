cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(retrieval_tools)

OPTION(NOCATKIN "Disable CATKIN compilation" OFF)

if (NOT NOCATKIN) # double negation, hell yeah
    find_package(catkin QUIET)
else()
    message("NOT USING CATKIN!")
endif()

#set(CMAKE_C_COMPILER clang)
#set(CMAKE_CXX_COMPILER clang++)

add_definitions(-std=c++11 -O3)

# Define the locations of the k_means_tree project
# If using catkin, including it using catkin instead
if (catkin_FOUND)
    find_package(catkin REQUIRED COMPONENTS roscpp tf tf_conversions pcl_ros
                        metaroom_xml_parser object_manager k_means_tree
                        dynamic_object_retrieval convex_segmentation)
    set(ROS_LIBRARIES ${catkin_LIBRARIES})
    include_directories(${catkin_INCLUDE_DIRS})
else()
    # Show where to find the find package scripts
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

    # For iterators to access the data
    set(retrieval_dir ${CMAKE_CURRENT_SOURCE_DIR}/../dynamic_object_retrieval)
    include_directories(${retrieval_dir}/include)
    link_directories(${retrieval_dir}/build)

    # Cereal serialization
    set(cereal_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/../k_means_tree/cereal/include)
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

if (catkin_FOUND)
    catkin_package(
        INCLUDE_DIRS include
    )
endif()

include_directories(include)

add_executable(create_convex_surfels src/create_convex_surfels.cpp)
target_link_libraries(create_convex_surfels ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(create_sweep_illustration src/create_sweep_illustration.cpp)
target_link_libraries(create_sweep_illustration ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(annotate_sweep_segment src/annotate_sweep_segment.cpp)
target_link_libraries(annotate_sweep_segment ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(create_surfel_overlap_cloud src/create_surfel_overlap_cloud.cpp)
target_link_libraries(create_surfel_overlap_cloud ${PCL_LIBRARIES})

add_executable(create_overlap_cloud src/create_overlap_cloud.cpp)
target_link_libraries(create_overlap_cloud ${PCL_LIBRARIES})

add_executable(create_line_point_clouds src/create_line_point_clouds.cpp)
target_link_libraries(create_line_point_clouds ${PCL_LIBRARIES})

add_executable(transform_surfel_cloud src/transform_surfel_cloud.cpp)
target_link_libraries(transform_surfel_cloud ${PCL_LIBRARIES})

if (catkin_FOUND)
    # Mark cpp header files for installation
    install(DIRECTORY include/retrieval_tools
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    # Mark executables and/or libraries for installation
    install(TARGETS create_convex_surfels create_sweep_illustration annotate_sweep_segment create_surfel_overlap_cloud
                    create_overlap_cloud create_line_point_clouds transform_surfel_cloud
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()
