#include "cloud_merge_node.h"

void callback(const std_msgs::String& controlString)
{
    ROS_INFO_STREAM("Received control string "<<controlString);
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "Cloud_merge_node");
    ros::NodeHandle n;

    ros::NodeHandle aRosNode("~");

    CloudMergeNode<pcl::PointXYZRGB> aCloudMergeNode(aRosNode);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
