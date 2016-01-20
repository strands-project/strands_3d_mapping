cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(comparisons)

#set(CMAKE_C_COMPILER clang)
#set(CMAKE_CXX_COMPILER clang++)
add_definitions(-std=c++11 -O3)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

find_package(OpenGL)

# Find and configure OpenMP
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -stdlib=libstdc++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -lstdc++ -stdlib=libstdc++")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lstdc++ -stdlib=libstdc++")

# Find and configure VLFeat
set(vlfeat_dir ${CMAKE_CURRENT_SOURCE_DIR}/vlfeat)
set(vlfeat_include_dir ${vlfeat_dir}/vl)
set(vlfeat_library_dir ${vlfeat_dir}/bin/glnxa64/objs)
set(vlfeat_objects ${vlfeat_library_dir}/kmeans.o
    ${vlfeat_library_dir}/vlad.o ${vlfeat_library_dir}/generic.o
    ${vlfeat_library_dir}/mathop.o ${vlfeat_library_dir}/host.o
    ${vlfeat_library_dir}/kdtree.o ${vlfeat_library_dir}/random.o
    ${vlfeat_library_dir}/mathop_sse2.o ${vlfeat_library_dir}/mathop_avx.o)
include_directories(include ${vlfeat_include_dir})

# Needed for the parsing of Meta Rooms
set(parser_workspace ${CMAKE_CURRENT_SOURCE_DIR}/../../catkin_ws)
set(parser_include_dir ${parser_workspace}/src/strands_3d_mapping/metaroom_xml_parser/include)
set(parser_library_dir ${parser_workspace}/devel/lib)
include_directories(${parser_include_dir})
link_directories(${parser_library_dir})

# Configure the object_3d_retrieval project, needed for iterating features
set(retrieval_dir ${CMAKE_CURRENT_SOURCE_DIR}/../dynamic_object_retrieval)
include_directories(${retrieval_dir}/include)
link_directories(${retrieval_dir}/build)

# Define the locations of the k_means_tree project
set(k_means_tree_dir ${CMAKE_CURRENT_SOURCE_DIR}/../k_means_tree)
set(k_means_tree_library_dir ${k_means_tree_dir}/build)
include_directories(${k_means_tree_dir}/include ${k_means_tree_dir}/impl)
link_directories(${k_means_tree_library_dir})

set(benchmark_dir ${CMAKE_CURRENT_SOURCE_DIR}/../benchmark)
set(benchmark_library_dir ${benchmark_dir}/build)
include_directories(${benchmark_dir}/include)
link_directories(${benchmark_library_dir})

set(stopwatch_dir ${retrieval_dir}/stopwatch)
set(stopwatch_include_dir ${stopwatch_dir}/src)
include_directories(${stopwatch_include_dir})

# Show where to find the find package scripts
list(APPEND CMAKE_MODULE_PATH "${retrieval_dir}/cmake")

# Find ROS
find_package(ROS REQUIRED)
include_directories(${ROS_INCLUDE_DIR})
link_directories(${ROS_LIBRARY_DIR})

find_package(HDF5)
include_directories(${HDF5_INCLUDE_DIR})

find_package(FLANN)
include_directories(${FLANN_INCLUDE_DIRS})


# Define the locations of cereal serialization header files
set(cereal_include_dir ${k_means_tree_dir}/cereal/include)
include_directories(${cereal_include_dir})

# Libraries

add_library(vlad_common src/common.cpp include/vlad/common.h)
target_link_libraries(vlad_common ${PCL_LIBRARIES})

add_library(vlad_representation src/vlad_representation.cpp include/vlad/vlad_representation.h ${vlfeat_objects})
target_link_libraries(vlad_representation vlad_common ${ROS_LIBRARIES} ${OpenCV_LIBS}
                      ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES}
                      dynamic_visualize)

add_library(bow_representation src/bow_representation.cpp include/vlad/bow_representation.h)
target_link_libraries(bow_representation vlad_common ${ROS_LIBRARIES} ${OpenCV_LIBS}
                      ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES}
                      dynamic_visualize)

# Executables
add_executable(test_vlfeat src/test_vlfeat.cpp)
target_link_libraries(test_vlfeat vlad_common vlad_representation bow_representation dynamic_visualize)

add_executable(compute_esf_features src/compute_esf_features.cpp)
target_link_libraries(compute_esf_features ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY}  ${ROS_LIBRARIES} ${OpenCV_LIBS}
                      ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_query src/test_query.cpp)
target_link_libraries(test_query vlad_common vlad_representation bow_representation
                      benchmark_retrieval benchmark_visualization surfel_renderer
                      pfhrgb_estimation benchmark_overlap
                      ${OPENGL_gl_LIBRARY} ${QT_QTOPENGL_LIBRARY})

add_executable(benchmark_query_vlad src/benchmark_query_vlad.cpp)
target_link_libraries(benchmark_query_vlad vlad_common vlad_representation bow_representation
                      pfhrgb_estimation
                      benchmark_result benchmark_retrieval benchmark_visualization surfel_renderer benchmark_overlap
                      grouped_vocabulary_tree vocabulary_tree k_means_tree # The order here matters for some reason
                      ${OPENGL_gl_LIBRARY} ${QT_QTOPENGL_LIBRARY}
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(benchmark_query_esf src/benchmark_query_esf.cpp)
target_link_libraries(benchmark_query_esf
                      ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} pfhrgb_estimation
                      benchmark_result benchmark_retrieval benchmark_visualization surfel_renderer benchmark_overlap
                      grouped_vocabulary_tree vocabulary_tree k_means_tree # The order here matters for some reason
                      ${OPENGL_gl_LIBRARY} ${QT_QTOPENGL_LIBRARY}
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(benchmark_query_discovery src/benchmark_query_discovery.cpp
               include/detection/UnionFindSimple.hpp include/detection/segmenter.cpp)
target_link_libraries(benchmark_query_discovery
                      benchmark_result benchmark_retrieval benchmark_visualization surfel_renderer benchmark_overlap
                      grouped_vocabulary_tree vocabulary_tree k_means_tree # The order here matters for some reason
                      ${OPENGL_gl_LIBRARY} ${QT_QTOPENGL_LIBRARY}
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(benchmark_discovery_segmentation src/benchmark_discovery_segmentation.cpp
               include/detection/UnionFindSimple.hpp include/detection/segmenter.cpp)
target_link_libraries(benchmark_discovery_segmentation benchmark_segmentation
                      benchmark_result benchmark_retrieval benchmark_overlap dynamic_visualize
                      grouped_vocabulary_tree vocabulary_tree k_means_tree # The order here matters for some reason
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})

add_executable(compute_discovery_graphs src/compute_discovery_graphs.cpp
               include/detection/UnionFindSimple.hpp include/detection/segmenter.cpp)
target_link_libraries(compute_discovery_graphs dynamic_visualize
                      ${ROS_LIBRARIES} ${OpenCV_LIBS} ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES})
