#include "semantic_map/semantic_map_node.h"

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "Semantic_map_node");
    ros::NodeHandle n;

    ros::NodeHandle aRosNode("~");

    SemanticMapNode<pcl::PointXYZRGB> aSemanticMapNode(aRosNode);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
