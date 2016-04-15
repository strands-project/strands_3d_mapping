cmake_minimum_required(VERSION 2.8.3)
project(observation_registration_services)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_msgs geometry_msgs message_generation)

add_service_files(
   FILES
   ObservationRegistrationService.srv
   AdditionalViewRegistrationService.srv
   ObjectAdditionalViewRegistrationService.srv
)

generate_messages(
  DEPENDENCIES  
  std_msgs
  sensor_msgs	
  geometry_msgs 
)


catkin_package(
   INCLUDE_DIRS 
   CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs
   DEPENDS  
   LIBRARIES 
)




include_directories(${catkin_INCLUDE_DIRS}
                    )


