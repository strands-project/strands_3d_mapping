#ifndef QUASIMODO_CONVERSIONS_H
#define QUASIMODO_CONVERSIONS_H

#include <soma_llsd_msgs/Segment.h>
#include <soma_llsd_msgs/Observation.h>
#include <soma_llsd_msgs/Scene.h>
#include <quasimodo_msgs/model.h>
#include <quasimodo_msgs/rgbd_frame.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace quasimodo_conversions {

void model_to_soma_segment(ros::NodeHandle& n, const quasimodo_msgs::model& model, soma_llsd_msgs::Segment& segment);
void soma_segment_to_model(ros::NodeHandle& n, const soma_llsd_msgs::Segment& segment, quasimodo_msgs::model& model);
void soma_observation_to_frame(ros::NodeHandle& n, const soma_llsd_msgs::Observation& obs, quasimodo_msgs::rgbd_frame& frame);
void frame_to_soma_observation(ros::NodeHandle& n, const quasimodo_msgs::rgbd_frame& frame, soma_llsd_msgs::Observation& obs);
void raw_frames_to_soma_scene(const cv::Mat& rgb, const cv::Mat& depth,
                              const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                              const Eigen::Matrix4d& pose, const Eigen::Matrix3d& K,
                              const std::string& waypoint, const std::string episode_id,
                              soma_llsd_msgs::Scene& scene);

} // namespace quasimodo_conversions

#endif // QUASIMODO_CONVERSIONS_H
