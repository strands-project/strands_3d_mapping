cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(k_means_tree)

OPTION(NOCATKIN "Disable CATKIN compilation" OFF)

if (NOT NOCATKIN) # double negation, hell yeah
    find_package(catkin QUIET)
else()
    message("NOT USING CATKIN!")
endif()

add_definitions(-std=c++11 -O3)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

if (catkin_FOUND)
    catkin_package(
        LIBRARIES k_means_tree vocabulary_tree grouped_vocabulary_tree
        INCLUDE_DIRS impl include cereal/include
    )
endif()

include_directories(include impl cereal/include)

add_library(k_means_tree src/k_means_tree.cpp include/k_means_tree/k_means_tree.h impl/k_means_tree.hpp)
add_library(vocabulary_tree src/vocabulary_tree.cpp include/vocabulary_tree/vocabulary_tree.h impl/vocabulary_tree.hpp)
add_library(grouped_vocabulary_tree src/grouped_vocabulary_tree.cpp
            include/grouped_vocabulary_tree/grouped_vocabulary_tree.h
            impl/grouped_vocabulary_tree.hpp)

add_executable(test_tree src/test.cpp)
add_executable(test_vocabulary_tree src/test_vocabulary_tree.cpp)

target_link_libraries(k_means_tree ${PCL_LIBRARIES})
target_link_libraries(vocabulary_tree k_means_tree ${PCL_LIBRARIES})
target_link_libraries(grouped_vocabulary_tree vocabulary_tree k_means_tree ${PCL_LIBRARIES})
target_link_libraries(test_tree k_means_tree vocabulary_tree grouped_vocabulary_tree)
target_link_libraries(test_vocabulary_tree k_means_tree vocabulary_tree)

if (catkin_FOUND)
    # Mark cpp header files for installation
    install(DIRECTORY include/k_means_tree include/vocabulary_tree include/grouped_vocabulary_tree
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    # Mark cpp header files for installation
    install(FILES impl/k_means_tree.hpp impl/vocabulary_tree.hpp impl/grouped_vocabulary_tree.hpp
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

    # Mark executables and/or libraries for installation
    install(TARGETS k_means_tree vocabulary_tree grouped_vocabulary_tree test_tree test_vocabulary_tree
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()
