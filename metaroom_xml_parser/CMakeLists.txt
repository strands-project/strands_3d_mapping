cmake_minimum_required(VERSION 2.8.3)
project(metaroom_xml_parser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs qt_build image_geometry tf pcl_ros)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES metaroom_xml_parser
   DEPENDS qt_build image_geometry libqt4-dev pcl_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

set(HDRS
    include/metaroom_xml_parser/simple_xml_parser.h
    include/metaroom_xml_parser/simple_summary_parser.h
    include/metaroom_xml_parser/load_utilities.h
    include/metaroom_xml_parser/load_utilities.hpp
    include/metaroom_xml_parser/simple_dynamic_object_parser.h
    )

set(SRCS
    src/simple_xml_parser.cpp
    src/simple_summary_parser.cpp
    src/load_utilities.cpp
    src/simple_dynamic_object_parser.cpp
    )

add_library(metaroom_xml_parser ${HDRS}  ${SRCS})

# APPS

add_executable(load_single_file apps/load_single_file_main.cpp )

add_executable(load_multiple_files apps/load_multiple_files_main.cpp)

add_executable(load_labelled_data apps/load_labelled_data.cpp )

add_executable(test_dynamic_object_parser apps/test_dynamic_object_parser.cpp )

add_executable(load_additional_views apps/load_additional_views_main.cpp )

add_executable(print_objects_with_views apps/print_objects_with_views.cpp )

add_executable(print_sweep_xmls_at_waypoint apps/print_sweep_xmls_at_waypoint.cpp )

add_executable(print_sweep_xmls apps/print_sweep_xmls.cpp )

 target_link_libraries(metaroom_xml_parser
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

 target_link_libraries(load_single_file
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   metaroom_xml_parser
 )

 target_link_libraries(load_multiple_files
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   metaroom_xml_parser
 )

 target_link_libraries(load_labelled_data
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   metaroom_xml_parser
 )

 target_link_libraries(test_dynamic_object_parser
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   metaroom_xml_parser
 )

 target_link_libraries(load_additional_views
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   metaroom_xml_parser
 )

 target_link_libraries(print_objects_with_views
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   metaroom_xml_parser
 )

 target_link_libraries(print_sweep_xmls_at_waypoint
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   metaroom_xml_parser
 )

 target_link_libraries(print_sweep_xmls
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   metaroom_xml_parser
 )



############################# INSTALL TARGETS

install(TARGETS metaroom_xml_parser  load_single_file load_multiple_files load_labelled_data test_dynamic_object_parser load_additional_views print_objects_with_views print_sweep_xmls_at_waypoint print_sweep_xmls
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/metaroom_xml_parser/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
