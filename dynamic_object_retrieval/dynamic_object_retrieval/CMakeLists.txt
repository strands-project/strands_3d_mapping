cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(dynamic_object_retrieval)

OPTION(NOCATKIN "Disable CATKIN compilation" OFF)

include(ExternalProject)

if (NOT NOCATKIN) # double negation, hell yeah
    find_package(catkin QUIET)
else()
    message("NOT USING CATKIN!")
endif()

add_definitions(-std=c++11 -O3)

# Show where to find the find package scripts
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Define the locations of the k_means_tree project
# If using catkin, including it using catkin instead
if (catkin_FOUND)
    find_package(catkin REQUIRED COMPONENTS roscpp tf tf_conversions pcl_ros
                        metaroom_xml_parser object_manager k_means_tree convex_segmentation)
    set(ROS_LIBRARIES ${catkin_LIBRARIES})
    include_directories(${catkin_INCLUDE_DIRS})
else()
    set(k_means_tree_dir ${CMAKE_CURRENT_SOURCE_DIR}/../k_means_tree)
    set(k_means_tree_library_dir ${k_means_tree_dir}/build)
    include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
    link_directories(${k_means_tree_library_dir})
    # Compile the k_means_tree project, needed for this
    ExternalProject_Add(k_means_tree_project
        DOWNLOAD_COMMAND ""
        SOURCE_DIR ${k_means_tree_dir}
        BINARY_DIR ${k_means_tree_library_dir}
        BUILD_COMMAND ${CMAKE_MAKE_PROGRAM}
        INSTALL_COMMAND ""
    )

    set(segmentation_dir ${CMAKE_CURRENT_SOURCE_DIR}/../convex_segmentation)
    include_directories(${segmentation_dir}/include)
    link_directories(${segmentation_dir}/build)

    # Be sure to set this to YOUR workspace, use custom version of metaroom_xml_parser for labeled data,
    # https://github.com/RaresAmbrus/strands_3d_mapping, branch hydro-devel
    set(parser_workspace ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
    set(parser_include_dir ${parser_workspace}/src/strands_3d_mapping/metaroom_xml_parser/include)
    set(parser_library_dir ${parser_workspace}/devel/lib)
    include_directories(${parser_workspace}/src/strands_3d_mapping/object_manager/include)
    include_directories(${parser_include_dir})
    link_directories(${parser_library_dir})

    # Find ROS
    find_package(ROS REQUIRED)
    include_directories(${ROS_INCLUDE_DIR})
    link_directories(${ROS_LIBRARY_DIR})

    # Define the locations of cereal serialization header files
    set(cereal_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/../k_means_tree/cereal/include)
    include_directories(${cereal_include_dir})
endif()

# Eh, we need to somehow build this maybe?
set(stopwatch_dir ${CMAKE_CURRENT_SOURCE_DIR}/stopwatch)
set(stopwatch_include_dir ${stopwatch_dir}/src)
set(stopwatch_library_dir ${stopwatch_dir}/build)
include_directories(${stopwatch_include_dir})

# Compile the stopwatch project, needed for this
ExternalProject_Add(stopwatch_project
  DOWNLOAD_COMMAND ""
  SOURCE_DIR ${stopwatch_include_dir}
  BINARY_DIR ${stopwatch_library_dir}
  BUILD_COMMAND ${CMAKE_MAKE_PROGRAM}
  INSTALL_COMMAND ""
)

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
        LIBRARIES sift register_objects supervoxel_segmentation pfhrgb_estimation shot_estimation
                  dynamic_visualize extract_sift dynamic_retrieval extract_surfel_features
        INCLUDE_DIRS include ${stopwatch_include_dir}
    )
endif()

# The main header location
include_directories(include)
set(include_dir include/object_3d_retrieval)

# This library includes the old OpenCV SIFT library
add_library(sift include/sift/sift.cpp include/sift/sift.h)
# This library registers objects for us using SIFT
add_library(register_objects src/register_objects.cpp ${include_dir}/register_objects.h)
# This library does the convex segmentation
#add_library(supervoxel_segmentation src/supervoxel_segmentation.cpp ${include_dir}/supervoxel_segmentation.h)
# This library computes PFHRGB features, among other things
add_library(pfhrgb_estimation src/pfhrgb_estimation.cpp ${include_dir}/pfhrgb_estimation.h)
# This library computes SHOTCOLOR features, among other things
add_library(shot_estimation src/shot_estimation.cpp ${include_dir}/shot_estimation.h)

# Link all of our libraries
target_link_libraries(sift ${OpenCV_LIBS})
target_link_libraries(register_objects sift ${OpenCV_LIBS} ${PCL_LIBRARIES})
#target_link_libraries(supervoxel_segmentation ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(pfhrgb_estimation ${PCL_LIBRARIES})
target_link_libraries(shot_estimation ${PCL_LIBRARIES})

# This executable visualizes the convex segmentation, interactively
add_executable(demo_convex_segmentation src/demo_convex_segmentation.cpp)
target_link_libraries(demo_convex_segmentation ${ROS_LIBRARIES} pfhrgb_estimation shot_estimation ${PCL_LIBRARIES})

# This executable demos the convex segmentation on a merged room sweep
add_executable(demo_sweep_segmentation src/demo_sweep_segmentation.cpp)
target_link_libraries(demo_sweep_segmentation pfhrgb_estimation shot_estimation
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

configure_file(scripts/menu.py menu.py COPYONLY)
configure_file(scripts/training_menu.py training_menu.py COPYONLY)
configure_file(scripts/querying_menu.py querying_menu.py COPYONLY)

##################### NEW DYNAMIC OBJECT RETRIEVAL VERSION ###########################

add_library(dynamic_visualize src/visualize.cpp include/dynamic_object_retrieval/visualize.h)
add_dependencies(dynamic_visualize k_means_tree_project)
target_link_libraries(dynamic_visualize ${ROS_LIBRARIES}  ${PCL_LIBRARIES})

add_library(extract_sift src/extract_sift.cpp include/extract_sift/extract_sift.h)
target_link_libraries(extract_sift dynamic_visualize sift ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_library(dynamic_retrieval src/dynamic_retrieval.cpp include/dynamic_object_retrieval/dynamic_retrieval.h
            include/dynamic_object_retrieval/summary_types.h include/dynamic_object_retrieval/summary_iterators.h)
add_dependencies(dynamic_retrieval k_means_tree_project)
target_link_libraries(dynamic_retrieval ${ROS_LIBRARIES}   extract_sift ${PCL_LIBRARIES})

add_library(extract_surfel_features src/extract_surfel_features.cpp include/dynamic_object_retrieval/extract_surfel_features.h)
target_link_libraries(extract_surfel_features ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(dynamic_init_folders src/dynamic_init_folders.cpp)
target_link_libraries(dynamic_init_folders ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_convex_segmentation src/dynamic_convex_segmentation.cpp)
target_link_libraries(dynamic_convex_segmentation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_supervoxel_convex_segmentation src/dynamic_supervoxel_convex_segmentation.cpp)
target_link_libraries(dynamic_supervoxel_convex_segmentation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_extract_convex_features src/dynamic_extract_convex_features.cpp)
target_link_libraries(dynamic_extract_convex_features extract_surfel_features pfhrgb_estimation shot_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_extract_supervoxel_features src/dynamic_extract_supervoxel_features.cpp)
target_link_libraries(dynamic_extract_supervoxel_features dynamic_visualize pfhrgb_estimation shot_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_create_subsegments src/dynamic_create_subsegments.cpp)
target_link_libraries(dynamic_create_subsegments pfhrgb_estimation shot_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_init_vocabulary src/dynamic_init_vocabulary.cpp)
target_link_libraries(dynamic_init_vocabulary ${PCL_LIBRARIES})

add_executable(dynamic_train_vocabulary src/dynamic_train_vocabulary.cpp)
target_link_libraries(dynamic_train_vocabulary ${ROS_LIBRARIES}   dynamic_visualize ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_query_vocabulary src/dynamic_query_vocabulary.cpp)
target_link_libraries(dynamic_query_vocabulary ${ROS_LIBRARIES}   register_objects dynamic_visualize dynamic_retrieval extract_sift ${PCL_LIBRARIES})

add_executable(dynamic_extract_sift src/dynamic_extract_sift.cpp)
target_link_libraries(dynamic_extract_sift extract_sift  ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

# We are quite strict with the warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-old-style-cast")

############### TESTING PROGRAMS ##################

add_executable(test_added_count test/test_added_count.cpp)
target_link_libraries(test_added_count ${ROS_LIBRARIES}  dynamic_visualize ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_feature_keypoint_match test/test_feature_keypoint_match.cpp)
target_link_libraries(test_feature_keypoint_match ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_cloud_segmentation test/test_segmentation.cpp)
target_link_libraries(test_cloud_segmentation  dynamic_visualize pfhrgb_estimation shot_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_surfel_segmentation test/test_surfel_segmentation.cpp)
target_link_libraries(test_surfel_segmentation  dynamic_visualize pfhrgb_estimation shot_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_gt_labelled_data test/test_gt_labelled_data.cpp)
target_link_libraries(test_gt_labelled_data  dynamic_visualize pfhrgb_estimation shot_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_supervoxel_keypoints test/test_supervoxel_keypoints.cpp)
target_link_libraries(test_supervoxel_keypoints dynamic_visualize ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_supervoxel_convex_mapping test/test_supervoxel_convex_mapping.cpp)
target_link_libraries(test_supervoxel_convex_mapping dynamic_visualize ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_visualize_keypoints test/test_visualize_keypoints.cpp)
target_link_libraries(test_visualize_keypoints dynamic_visualize pfhrgb_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_query_keypoints test/test_query_keypoints.cpp)
target_link_libraries(test_query_keypoints dynamic_visualize pfhrgb_estimation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_adjacencies test/test_adjacencies.cpp)
target_link_libraries(test_adjacencies dynamic_visualize ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_top_match_one_map test/test_top_match_one_map.cpp)
target_link_libraries(test_top_match_one_map extract_sift register_objects pfhrgb_estimation
                         dynamic_visualize
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

if (catkin_FOUND)
    # Mark cpp header files for installation
    install(DIRECTORY include/dynamic_object_retrieval include/extract_sift include/object_3d_retrieval include/sift
      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION} # ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    # Mark cpp header files for installation
    install(FILES stopwatch/src/Stopwatch.h
      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION} # ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    install(PROGRAMS scripts/menu.py scripts/training_menu.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    # Mark executables and/or libraries for installation
    install(TARGETS sift register_objects pfhrgb_estimation shot_estimation demo_convex_segmentation demo_sweep_segmentation
                    dynamic_visualize extract_sift dynamic_retrieval extract_surfel_features dynamic_init_folders dynamic_convex_segmentation
                    dynamic_supervoxel_convex_segmentation dynamic_extract_convex_features dynamic_extract_supervoxel_features
                    dynamic_create_subsegments dynamic_init_vocabulary dynamic_train_vocabulary dynamic_query_vocabulary dynamic_extract_sift
                    test_added_count test_feature_keypoint_match test_cloud_segmentation test_surfel_segmentation test_gt_labelled_data
                    test_supervoxel_keypoints test_supervoxel_convex_mapping test_visualize_keypoints test_query_keypoints test_adjacencies test_top_match_one_map
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()
