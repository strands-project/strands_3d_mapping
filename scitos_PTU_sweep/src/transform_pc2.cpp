#include "ros/ros.h"
#include <ros/time.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

using namespace std;

ros::Publisher pub;
tf::TransformListener *listener;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("Called callback");
  sensor_msgs::PointCloud2 pc_out;
  pcl_ros::transformPointCloud("/base_link", *msg, pc_out, *listener);
  pub.publish(pc_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_pc2");
  ROS_INFO("INIT");
  ros::NodeHandle n;

  listener = new tf::TransformListener();

  string pc_sub_topic;
  string pc_pub_topic;

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("pc2_input", pc_sub_topic, string("/ptu_sweep/depth/points"));
  ros::Subscriber sub = n.subscribe("/ptu_sweep/depth/points", 10, callback);
  private_node_handle_.param("pc2_output", pc_pub_topic, string("/transform_pc2/depth/points"));
  pub = n.advertise<sensor_msgs::PointCloud2>(pc_pub_topic.c_str(), 10);

  ros::spin();

  return 0;
}
