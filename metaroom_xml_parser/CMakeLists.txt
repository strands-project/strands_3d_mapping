cmake_minimum_required(VERSION 2.8.3)
project(metaroom_xml_parser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs qt_build image_geometry tf)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES metaroom_xml_parser
   DEPENDS qt_build image_geometry libqt4-dev
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

set(HDRS
    include/simpleXMLparser.h
    include/simpleSummaryParser.h
    )

set(SRCS
    src/simpleXMLparser.cpp
    src/simpleSummaryParser.cpp
    )

add_library(metaroom_xml_parser ${HDRS}  src/simpleXMLparser.cpp src/simpleSummaryParser.cpp)

add_executable(load_single_file src/loadSingleFileMain.cpp )

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

add_executable(load_multiple_files src/loadMultipleFilesMain.cpp)

 target_link_libraries(load_multiple_files
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   metaroom_xml_parser
 )

############################# INSTALL TARGETS

install(TARGETS metaroom_xml_parser  load_single_file load_multiple_files
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
