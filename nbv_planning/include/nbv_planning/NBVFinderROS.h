//
// Created by chris on 24/11/15.
//

#ifndef NBV_PLANNING_NBVFINDERROS_H
#define NBV_PLANNING_NBVFINDERROS_H

#include "NBVFinder.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>

using namespace visualization_msgs;

namespace nbv_planning {
    /**
     * ROS Specific version of NBVFinder. This adds ROS service handles to functions, and accepts ROS standard types
     */
    class NBVFinderROS : public NBVFinder {

    public:
        NBVFinderROS(const SensorModel &m_sensor_model, ros::NodeHandle node_handle);

        void publish_volume_marker();

        void publish_octomap();

        using NBVFinder::update_current_volume;

        bool update_current_volume(const sensor_msgs::PointCloud2 &cloud, const Eigen::Affine3d &sensor_origin);

        void publish_views();

    private:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_volume_marker_publisher;
        ros::Publisher m_octomap_publisher;
        ros::Publisher m_view_marker_publisher;
    };
}

#endif //NBV_PLANNING_NBVFINDERROS_H
