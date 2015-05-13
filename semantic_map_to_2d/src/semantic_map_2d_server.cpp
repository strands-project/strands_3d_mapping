#include <ros/ros.h>
#include <semantic_map_to_2d/SemanticMap2dServer.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "semtic_map_2d_server");

  SemanticMap2DServer server;
  ros::spinOnce();


  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("semantic_map_2d_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
