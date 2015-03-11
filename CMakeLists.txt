cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(k_means_tree)

add_definitions(-std=c++11 -O4)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

include_directories(include impl)

add_library(k_means_tree src/k_means_tree.cpp include/k_means_tree/k_means_tree.h impl/k_means_tree.hpp)
add_library(vocabulary_tree src/vocabulary_tree.cpp include/vocabulary_tree/vocabulary_tree.h impl/vocabulary_tree.hpp)
add_library(grouped_vocabulary_tree src/grouped_vocabulary_tree.cpp
            include/grouped_vocabulary_tree/grouped_vocabulary_tree.h
            impl/grouped_vocabulary_tree.hpp)
add_library(reweighted_vocabulary_tree src/reweighted_vocabulary_tree.cpp
            include/reweighted_vocabulary_tree/reweighted_vocabulary_tree.h
            impl/reweighted_vocabulary_tree.hpp)

add_executable(test src/test.cpp)

target_link_libraries(k_means_tree ${PCL_LIBRARIES})
target_link_libraries(vocabulary_tree k_means_tree ${PCL_LIBRARIES})
target_link_libraries(grouped_vocabulary_tree k_means_tree ${PCL_LIBRARIES})
target_link_libraries(reweighted_vocabulary_tree vocabulary_tree ${PCL_LIBRARIES})
target_link_libraries(test k_means_tree vocabulary_tree reweighted_vocabulary_tree grouped_vocabulary_tree)
