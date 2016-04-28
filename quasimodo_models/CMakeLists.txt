cmake_minimum_required(VERSION 2.8.3)
project(quasimodo_models)

#find_package(catkin REQUIRED)# quasimodo_msgs)# COMPONENTS quasimodo_msgs)
set(CMAKE_CXX_FLAGS "-O4 -g -pg -Wunknown-pragmas -Wno-unknown-pragmas -Wsign-compare -fPIC -std=c++0x -o popcnt -mssse3")


find_package(catkin REQUIRED COMPONENTS
	#metaroom_xml_parser
	#semantic_map
	pcl_ros
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quasimodo_ModelUpdater quasimodo_SICP quasimodo_reglib quasimodo_massreglib quasimodo_model quasimodo_core quasimodo_myhull quasimodo_Mesher quasimodo_weightlib quasimodo_modelmask
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


##############################################
##################### INCLUDES ###############
##############################################

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Opencv stuff
link_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/include/)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(src)
include_directories(include)

#ROS INDIGO STUFF
include_directories(/opt/ros/indigo/include)
link_directories(/opt/ros/indigo/lib)

#g2o
link_directories(/usr/local/lib/)
include_directories(/usr/local/include/)

#Ceres
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})


add_library(quasimodo_SICP src/registration/ICP.cpp)
target_link_libraries(quasimodo_SICP ${catkin_LIBRARIES})

add_library(quasimodo_weightlib src/weightfunctions/DistanceWeightFunction2.cpp src/weightfunctions/DistanceWeightFunction2PPR.cpp src/weightfunctions/DistanceWeightFunction2PPR2.cpp)
target_link_libraries(quasimodo_weightlib quasimodo_SICP ceres glog ${catkin_LIBRARIES})

add_library(quasimodo_myhull src/registration/myhull.cpp)
target_link_libraries(quasimodo_myhull ${catkin_LIBRARIES})

add_library(quasimodo_reglib src/registration/Registration.cpp src/registration/RegistrationSICP.cpp src/registration/RegistrationPPR.cpp src/registration/RegistrationRandom.cpp src/registration/RegistrationRefinement.cpp src/registration/RegistrationRefinementColor.cpp)
target_link_libraries(quasimodo_reglib quasimodo_myhull quasimodo_SICP quasimodo_weightlib ceres glog ${catkin_LIBRARIES})

add_library(quasimodo_core src/core/Util.cpp src/core/Camera.cpp src/core/RGBDFrame.cpp)
target_link_libraries(quasimodo_core ${CERES_LIBRARIES} quasimodo_SICP ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})



add_library(quasimodo_massreglib src/registration/MassRegistration.cpp src/registration/MassRegistrationPPR.cpp src/registration/MassRegistrationPPRColor.cpp)
target_link_libraries(quasimodo_massreglib quasimodo_reglib quasimodo_core)# g2o_cli g2o_interactive g2o_solver_cholmod g2o_solver_slam2d_linear g2o_types_sba g2o_types_slam3d_addons g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sclam2d g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sim3 g2o_viewer g2o_ext_freeglut_minimal g2o_parser g2o_solver_eigen g2o_types_data g2o_types_slam2d_addons g2o_incremental g2o_simulator g2o_solver_pcg g2o_types_icp g2o_types_slam2d)

add_library(quasimodo_modelmask src/model/ModelMask.cpp)
target_link_libraries(quasimodo_modelmask quasimodo_core quasimodo_reglib ${catkin_LIBRARIES})

add_library(quasimodo_model src/model/Model.cpp)
target_link_libraries(quasimodo_model quasimodo_modelmask quasimodo_core quasimodo_reglib ${catkin_LIBRARIES})

add_library(quasimodo_Mesher src/mesher/Mesh.cpp)
target_link_libraries(quasimodo_Mesher quasimodo_model ${catkin_LIBRARIES})


add_library(quasimodo_ModelUpdater src/modelupdater/ModelUpdater.cpp src/modelupdater/ModelUpdaterBasic.cpp src/modelupdater/ModelUpdaterBasicFuse.cpp)
target_link_libraries(quasimodo_ModelUpdater quasimodo_model quasimodo_reglib quasimodo_massreglib ${catkin_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html



# Mark executables and/or libraries for installation
install(TARGETS quasimodo_SICP quasimodo_weightlib quasimodo_myhull quasimodo_reglib quasimodo_core quasimodo_massreglib quasimodo_modelmask quasimodo_model quasimodo_Mesher quasimodo_ModelUpdater
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
