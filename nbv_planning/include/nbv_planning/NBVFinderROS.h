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
#include <boost/shared_ptr.hpp>

using namespace visualization_msgs;

namespace nbv_planning {
    /**
     * ROS Specific version of NBVFinder. This adds ROS service handles to functions, accepts ROS standard types and
     * publishes visualisations.
     */
    class NBVFinderROS : public NBVFinder {
    public:
        typedef boost::shared_ptr<NBVFinderROS> Ptr;


        /**
         * Takes the ROS node handle in addition to the NBVFinder signature.
         */
        NBVFinderROS(const SensorModel &m_sensor_model, ros::NodeHandle node_handle);

        /**
         * Publish a ROS Visualisation marker (line array) to show the target volume.  The topic is "target_volume"
         * under the namespace of the node handle.
         */
        void publish_volume_marker();

        /**
         * Publish the current internal octomap for visualisation in RViz. Topic is "octomap" under the node's namespace.
         */
        void publish_octomap();

        using NBVFinder::update_current_volume;
        /**
         * Update the current volume using a ROS sensor_msgs PointCloud2 message.
         */
        bool update_current_volume(const sensor_msgs::PointCloud2 &cloud, const Eigen::Affine3d &sensor_origin);

        /**
         * Publish the view candidates' frustrum's as line arrays. Topic is "views" under the node's namespace.
         */
        void publish_views();

    private:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_volume_marker_publisher;
        ros::Publisher m_octomap_publisher;
        ros::Publisher m_view_marker_publisher;
    };
}

#endif //NBV_PLANNING_NBVFINDERROS_H
