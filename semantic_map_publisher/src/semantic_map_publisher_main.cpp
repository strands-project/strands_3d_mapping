#include "semantic_map_publisher.h"

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "Semantic_map_publisher_node");
    ros::NodeHandle n;

    ros::NodeHandle aRosNode("~");

    SemanticMapPublisher<pcl::PointXYZRGB> aSemanticMapPublisherNode(aRosNode);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
