#include "ros/ros.h"
#include <ros/time.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

using namespace std;

ros::Publisher pub;
tf::TransformListener *listener;
tf::MessageFilter<sensor_msgs::PointCloud2> *tf_filter;

void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
{
  //ROS_INFO("Called callback");
  sensor_msgs::PointCloud2 pc_out;
  listener->waitForTransform(msg->header.frame_id.c_str(), "/base_link", ros::Time(), ros::Duration(3.0));
  pcl_ros::transformPointCloud("/base_link", *msg, pc_out, *listener);
  pub.publish(pc_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_pc2");

  ros::NodeHandle n;

  listener = new tf::TransformListener();

  string pc_sub_topic;
  string pc_pub_topic;

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("pc2_input", pc_sub_topic, string("/ptu_sweep/depth/points"));
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub;
  sub.subscribe(private_node_handle_, pc_sub_topic.c_str(), 10);
  tf_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(sub, *listener, "/base_link", 10);
  tf_filter->registerCallback(boost::bind(&callback, _1));
  private_node_handle_.param("pc2_output", pc_pub_topic, string("/transform_pc2/depth/points"));
  pub = n.advertise<sensor_msgs::PointCloud2>(pc_pub_topic.c_str(), 10);

  ros::spin();

  return 0;
}
