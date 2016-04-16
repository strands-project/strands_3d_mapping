cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(convex_segmentation)

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
    find_package(catkin REQUIRED COMPONENTS roscpp tf tf_conversions pcl_ros k_means_tree)
    set(ROS_LIBRARIES ${catkin_LIBRARIES})
    include_directories(${catkin_INCLUDE_DIRS})

    add_message_files(
        FILES
        CloudArray.msg
    )

    generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
        sensor_msgs
    )
else()
    # Find ROS
    find_package(ROS REQUIRED)
    include_directories(${ROS_INCLUDE_DIR})
    link_directories(${ROS_LIBRARY_DIR})

    # Define the locations of cereal serialization header files
    set(cereal_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/../k_means_tree/cereal/include)
    include_directories(${cereal_include_dir})
endif()

# Find PCL
find_package(PCL 1.6 REQUIRED COMPONENTS common io search visualization surface kdtree features surface segmentation octree filter keypoints)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find OpenCV
find_package(OpenCV REQUIRED)

if (catkin_FOUND)
    catkin_package(
        LIBRARIES supervoxel_segmentation
        INCLUDE_DIRS include
    )
endif()

# The main header location
include_directories(include)
set(include_dir include/object_3d_retrieval)

# This library does the convex segmentation
add_library(supervoxel_segmentation src/supervoxel_segmentation.cpp ${include_dir}/supervoxel_segmentation.h)

# Link all of our libraries
target_link_libraries(supervoxel_segmentation ${OpenCV_LIBS} ${PCL_LIBRARIES})

# We are quite strict with the warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-old-style-cast")

############### TESTING PROGRAMS ##################

if (catkin_FOUND)
    add_executable(convex_segmentation_node src/convex_segmentation_node.cpp)
    target_link_libraries(convex_segmentation_node supervoxel_segmentation ${ROS_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
endif()

if (catkin_FOUND)
    install(DIRECTORY include/object_3d_retrieval
      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION} # ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    # Mark executables and/or libraries for installation
    install(TARGETS supervoxel_segmentation convex_segmentation_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()
