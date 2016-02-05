//
// Created by chris on 24/11/15.
//

#include "nbv_planning/NBVFinderROS.h"
#include <algorithm>

#include <octomap_ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>

void nbv_planning::NBVFinderROS::publish_volume_marker() {
    Marker marker;
    visualization_msgs::Marker points, line_strip, line_list;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_volume";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = Marker::LINE_LIST;
    marker.scale.x = 0.05;
//    marker.scale.y = 0.1;
//    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    // Create the vertices for the points and lines
    Eigen::Vector3f origin = m_target_volume.get_origin();
    Eigen::Vector3f extents = m_target_volume.get_extents();

    geometry_msgs::Point p1, p2;
    Eigen::Array3f direction(1,1,1);
    int offset_indices[] = {0,1,2};
    do {
        for(int j=-1;j<2;j += 2) {
            for (int k = -1; k < 2; k += 2) {
                Eigen::Array3f a_offset;
                a_offset[offset_indices[0]]=j;
                a_offset[offset_indices[1]]= k;
                a_offset[offset_indices[2]]= 1;
                Eigen::Array3f b_offset;
                b_offset[offset_indices[0]]=j;
                b_offset[offset_indices[1]]= k;
                b_offset[offset_indices[2]]= -1;

                Eigen::Vector3f a,b;
                for (uint i=0; i < 3; i++) {
                    a[i] = origin[i] + a_offset[i]*extents[i];
                    b[i] = origin[i] + b_offset[i]*extents[i];
                }
                // = origin + a_offset * extents.array();
                //Eigen::Vector3f b = origin + b_offset * extents.array();

                p1.x = a[0];
                p1.y = a[1];
                p1.z = a[2];

                p2.x = b[0];
                p2.y = b[1];
                p2.z = b[2];

                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }
    } while ( std::next_permutation(offset_indices,offset_indices+3) );

    m_volume_marker_publisher.publish(marker);
}

void nbv_planning::NBVFinderROS::publish_octomap() {
    if (m_octomap_publisher.getNumSubscribers() > 0) {
        octomap_msgs::Octomap msg;
        octomap_msgs::binaryMapToMsg(*m_octree, msg);
        msg.header.frame_id="/map";
        m_octomap_publisher.publish(msg);
    }
}

bool nbv_planning::NBVFinderROS::update_current_volume(const sensor_msgs::PointCloud2 &cloud,
                                                       const Eigen::Affine3d &sensor_origin) {
    nbv_planning::NBVFinder::CloudPtr pcl_cloud(new nbv_planning::NBVFinder::Cloud());
    pcl::fromROSMsg(cloud,*pcl_cloud);
    std::cout << "Number of points in pcl cloud: " << pcl_cloud->points.size() << "\n";
    return update_current_volume(pcl_cloud, sensor_origin);
}

inline geometry_msgs::Point convert_vector_to_point(const Eigen::Vector3d &vector){
    geometry_msgs::Point point;
    point.x=vector(0);
    point.y=vector(1);
    point.z=vector(2);
    return point;
}

void nbv_planning::NBVFinderROS::publish_views() {
    MarkerArray cameras;
    uint id=0;
    for (std::vector<Eigen::Affine3d>::iterator view = m_candidate_views.begin(); view < m_candidate_views.end(); view++) {
        Marker camera;
        camera.id = id++;
        camera.header.frame_id = "/map";
        camera.type = Marker::LINE_LIST;
        camera.action = Marker::ADD;
        camera.scale.x = 0.01;
        camera.ns = "views";
        camera.color.a=1;
        camera.color.g=1;

        std::vector<Eigen::Vector3d> frustum = m_sensor_model.get_frustum_vertices(m_sensor_model.get_min_range()+0.3);
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(*view, pose);
        camera.pose = pose;

        camera.points.push_back(convert_vector_to_point(frustum[0]));
        camera.points.push_back(convert_vector_to_point(frustum[1]));

        camera.points.push_back(convert_vector_to_point(frustum[2]));
        camera.points.push_back(convert_vector_to_point(frustum[3]));

        camera.points.push_back(convert_vector_to_point(frustum[0]));
        camera.points.push_back(convert_vector_to_point(frustum[2]));

        camera.points.push_back(convert_vector_to_point(frustum[1]));
        camera.points.push_back(convert_vector_to_point(frustum[3]));

        camera.points.push_back(convert_vector_to_point(frustum[4]));
        camera.points.push_back(convert_vector_to_point(frustum[5]));

        camera.points.push_back(convert_vector_to_point(frustum[6]));
        camera.points.push_back(convert_vector_to_point(frustum[7]));

        camera.points.push_back(convert_vector_to_point(frustum[4]));
        camera.points.push_back(convert_vector_to_point(frustum[6]));

        camera.points.push_back(convert_vector_to_point(frustum[5]));
        camera.points.push_back(convert_vector_to_point(frustum[7]));

        camera.points.push_back(convert_vector_to_point(frustum[0]));
        camera.points.push_back(convert_vector_to_point(frustum[4]));

        camera.points.push_back(convert_vector_to_point(frustum[2]));
        camera.points.push_back(convert_vector_to_point(frustum[6]));

        camera.points.push_back(convert_vector_to_point(frustum[1]));
        camera.points.push_back(convert_vector_to_point(frustum[5]));

        camera.points.push_back(convert_vector_to_point(frustum[3]));
        camera.points.push_back(convert_vector_to_point(frustum[7]));

        cameras.markers.push_back(camera);

//        geometry_msgs::Pose pose;
//        tf::poseEigenToMsg(*view, pose);
//        camera.type = visualization_msgs::Marker::SPHERE;
//        camera.mesh_resource = "package://nbv_planning/meshes/camera.dae";
//        camera.scale.x= camera.scale.y =camera.scale.z=0.3;
//        camera.pose = pose;
//        camera.header.frame_id="/map";
//        camera.ns = "views";
//        camera.color.a=1;
//        camera.color.g=1;
//        camera.id = id++;
//        camera.action = visualization_msgs::Marker::ADD;
//        cameras.markers.push_back(camera);
    }
//    Eigen::Vector3d min,max;
//    calculate_viewed_volume(min,max);
//    Marker viewed_area;
//    viewed_area.id = id++;
//    viewed_area.header.frame_id = "/map";
//    viewed_area.type = Marker::LINE_LIST;
//    viewed_area.action = Marker::ADD;
//    viewed_area.scale.x = 0.05;//max(0)-min(0);
////    viewed_area.scale.y = max(1)-min(1);
////    viewed_area.scale.z = max(2)-min(2);
////    viewed_area.pose.position.x=(min(0)+max(0))/2.0;
////    viewed_area.pose.position.y=(min(1)+max(1))/2.0;
////    viewed_area.pose.position.z=(min(2)+max(2))/2.0;
//    viewed_area.points.push_back(convert_vector_to_point());
//    viewed_area.ns = "views";
//    viewed_area.color.a=1;
//    viewed_area.color.g=1;
//    cameras.markers.push_back(viewed_area);
    m_view_marker_publisher.publish(cameras);
}

nbv_planning::NBVFinderROS::NBVFinderROS(const SensorModel &m_sensor_model, ros::NodeHandle node_handle)
        : NBVFinder(m_sensor_model),
          m_node_handle(node_handle) {
    m_volume_marker_publisher = m_node_handle.advertise<Marker>("target_volume", 10, true);
    m_octomap_publisher = m_node_handle.advertise<octomap_msgs::Octomap>("octomap", 10, true);
    m_view_marker_publisher = m_node_handle.advertise<MarkerArray>("views", 10, true);

    std::cout << "Creating a ROS wrapped NBV planner in namespace " << m_node_handle.getNamespace() <<
    std::endl;
}