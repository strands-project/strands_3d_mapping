# Find ROS library
#
# ROS_INCLUDE_DIR      where to find the include files
# ROS_LIBRARY_DIR      where to find the libraries
# ROS_LIBRARIES        list of libraries to link
# ROS_FOUND            true if ROS was found

SET( ROS_LIBRARYDIR / CACHE PATH "Alternative library directory" )
SET( ROS_INCLUDEDIR / CACHE PATH "Alternative include directory" )
MARK_AS_ADVANCED( ROS_LIBRARYDIR ROS_INCLUDEDIR )

FIND_LIBRARY(ROS_LIBRARY_DIR
  NAMES rviz rosconsole roscpp rqt_rviz message_store roscpp_serialization rostime
  PATHS ${ROS_LIBRARYDIR})
FIND_PATH(ROS_INCLUDE_DIR
  NAMES ros/ros.h ros/time.h rqt_rviz/rviz.h
  PATHS ${ROS_INCLUDEDIR})

GET_FILENAME_COMPONENT( ROS_LIBRARY_DIR ${ROS_LIBRARY_DIR} PATH )

IF( ROS_INCLUDE_DIR AND ROS_LIBRARY_DIR )
    SET( ROS_LIBRARIES image_geometry cpp_common roscpp rosconsole tf_conversions metaroom_xml_parser )
    SET( ROS_FOUND TRUE )
ELSE( ROS_INCLUDE_DIR AND ROS_LIBRARY_DIR )
    SET( ROS_FOUND FALSE )
ENDIF( ROS_INCLUDE_DIR AND ROS_LIBRARY_DIR )
