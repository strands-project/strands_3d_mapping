#include "object_manager.h"

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "object_manager_node");
    ros::NodeHandle n;

    ros::NodeHandle aRosNode("~");

    ObjectManager<pcl::PointXYZRGB> aObjectManager(aRosNode);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
