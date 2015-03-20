#ifndef __SEMANTIC_MAP_PUBLISHER__H
#define __SEMANTIC_MAP_PUBLISHER__H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>


// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

// Services
#include <object_manager/DynamicObjectsService.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/registration/distances.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "load_utilities.h"

#include <geometry_msgs/Point.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/frustum_culling.h>

#include <semantic_map/room_xml_parser.h>

template <class PointType>
class ObjectManager {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    typedef typename object_manager::DynamicObjectsService::Request DynamicObjectsServiceRequest;
    typedef typename object_manager::DynamicObjectsService::Response DynamicObjectsServiceResponse;

    struct ObjStruct {
        std::string id;
        CloudPtr cloud;
    };




    ObjectManager(ros::NodeHandle nh);
    ~ObjectManager();

    bool dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res);
    std::vector<ObjectManager<PointType>::ObjStruct>  loadDynamicObjectsFromObservation(std::string obs_file);
    void returnObjectMask(std::string waypoint, std::string object_id, std::string observation_xml);

    ros::Publisher                                                              m_PublisherDynamicClusters;
    ros::ServiceServer                                                          m_DynamicObjectsServiceServer;

    static CloudPtr filterGroundClusters(CloudPtr dynamic, double min_height)
    {
        CloudPtr filtered(new Cloud());


        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (dynamic);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-1.0, min_height);
        pass.setFilterLimitsNegative (true);
        pass.filter (*filtered);

        return filtered;
    }


private:
    ros::NodeHandle                                                             m_NodeHandle;
    std::string                                                                 m_dataFolder;
    std::map<std::string, std::vector<ObjStruct>>                               m_waypointToObjMap;
    std::map<std::string, std::string>                                          m_waypointToSweepFileMap;
};

template <class PointType>
ObjectManager<PointType>::ObjectManager(ros::NodeHandle nh)
{
    ROS_INFO_STREAM("ObjectManager node initialized");

    m_NodeHandle = nh;

    passwd* pw = getpwuid(getuid());
    std::string default_folder(pw->pw_dir);
    default_folder+="/.semanticMap/";

    m_NodeHandle.param<std::string>("object_folder",m_dataFolder,default_folder);
    ROS_INFO_STREAM("Reading dynamic object from "<<m_dataFolder);

    m_PublisherDynamicClusters = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/object_manager/objects", 1, true);

    m_DynamicObjectsServiceServer = m_NodeHandle.advertiseService("ObjectManager/DynamicObjectsService", &ObjectManager::dynamicObjectsServiceCallback, this);
}

template <class PointType>
ObjectManager<PointType>::~ObjectManager()
{

}

template <class PointType>
bool ObjectManager<PointType>::dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res)
{
    ROS_INFO_STREAM("Received a dynamic clusters request for waypoint "<<req.waypoint_id);

    using namespace std;
    std::vector<std::string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(m_dataFolder, req.waypoint_id);
    if (matchingObservations.size() == 0)
    {
        ROS_INFO_STREAM("No observations for this waypoint "<<req.waypoint_id);
        return false;
    }

    sort(matchingObservations.begin(), matchingObservations.end());
    reverse(matchingObservations.begin(), matchingObservations.end());
    string latest = matchingObservations[0];
    std::vector<ObjStruct> currentObjects;

    ROS_INFO_STREAM("Latest observation "<<latest);

    auto it = m_waypointToSweepFileMap.find(req.waypoint_id);
    if (it == m_waypointToSweepFileMap.end())
    {
        // no dynamic clusters loaded for this waypoint
        // -> load them
        std::vector<ObjStruct> dynamicObjects = loadDynamicObjectsFromObservation(latest);
        if (dynamicObjects.size() == 0)
        {
            ROS_INFO_STREAM("No objects detected after clustering.");
            return true;
        }

        m_waypointToSweepFileMap[req.waypoint_id] = latest;
        m_waypointToObjMap[req.waypoint_id] = dynamicObjects;

        currentObjects = m_waypointToObjMap[req.waypoint_id];
    } else {
        if (m_waypointToSweepFileMap[req.waypoint_id] != latest)
        {
            ROS_INFO_STREAM("Older point cloud loaded in memory. Loading objects from latest observation ...");
            std::vector<ObjStruct> dynamicObjects = loadDynamicObjectsFromObservation(latest);
            if (dynamicObjects.size() == 0)
            {
                ROS_INFO_STREAM("No objects detected after clustering.");
                return true;
            }

            m_waypointToSweepFileMap[req.waypoint_id] = latest;
            m_waypointToObjMap[req.waypoint_id] = dynamicObjects;

            currentObjects = m_waypointToObjMap[req.waypoint_id];
        } else {
            ROS_INFO_STREAM("Objects loaded in memory");
            auto it2 =  m_waypointToObjMap.find(req.waypoint_id);
            if (it2 == m_waypointToObjMap.end())
            {
                ROS_ERROR_STREAM("Object map is empty. Reload.");
                std::vector<ObjStruct> dynamicObjects = loadDynamicObjectsFromObservation(latest);
                if (dynamicObjects.size() == 0)
                {
                    ROS_INFO_STREAM("No objects detected after clustering.");
                    return true;
                }

                m_waypointToSweepFileMap[req.waypoint_id] = latest;
                m_waypointToObjMap[req.waypoint_id] = dynamicObjects;

                currentObjects = m_waypointToObjMap[req.waypoint_id];
            } else {
                currentObjects = m_waypointToObjMap[req.waypoint_id];
            }
        }
    }

    ROS_INFO_STREAM("Found "<<currentObjects.size() <<" objects at "<<req.waypoint_id);
    // publish objects
    CloudPtr allObjects(new Cloud());

    for (auto cloudObj : currentObjects)
    {
        *allObjects += *(cloudObj.cloud);
    }

    sensor_msgs::PointCloud2 msg_objects;
    pcl::toROSMsg(*allObjects, msg_objects);
    msg_objects.header.frame_id="/map";

    m_PublisherDynamicClusters.publish(msg_objects);

    std::vector<sensor_msgs::PointCloud2> clouds;
    std::vector<std::string> id;
    std::vector<geometry_msgs::Point> centroids;

    for (auto cloudObj : currentObjects)
    {
        sensor_msgs::PointCloud2 msg_objects;
        pcl::toROSMsg(*cloudObj.cloud, msg_objects);
        msg_objects.header.frame_id="/map";

        clouds.push_back(msg_objects);
        id.push_back(cloudObj.id);


        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloudObj.cloud, centroid);

        geometry_msgs::Point ros_centroid; ros_centroid.x = centroid[0];ros_centroid.y = centroid[1];ros_centroid.z = centroid[2];
        centroids.push_back(ros_centroid);
    }

    res.objects = clouds;
    res.object_id = id;
    res.centroids = centroids;

    returnObjectMask(req.waypoint_id, "object_1",latest);

    return true;


}

template <class PointType>
std::vector<typename ObjectManager<PointType>::ObjStruct>  ObjectManager<PointType>::loadDynamicObjectsFromObservation(std::string obs_file)
{
    std::vector<ObjStruct> dynamicObjects;
    auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(obs_file, std::vector<std::string>{"RoomDynamicClusters"},false);
    if (sweep.dynamicClusterCloud->points.size() == 0)
    {
        // no clusters in the observation
        return dynamicObjects;
    }

    sweep.dynamicClusterCloud = filterGroundClusters(sweep.dynamicClusterCloud, 0.2);

    double tolerance = 0.03;
    int min_cluster_size = 800;
    int max_cluster_size = 10000;
    // split into clusters
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (sweep.dynamicClusterCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (sweep.dynamicClusterCloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        CloudPtr cloud_cluster (new Cloud());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (sweep.dynamicClusterCloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::stringstream ss;ss<<"object_";ss<<j;
        ObjStruct object;
        object.cloud = cloud_cluster;
        object.id = ss.str();

        dynamicObjects.push_back(object);

        j++;
    }

    return dynamicObjects;
}

template <class PointType>
void ObjectManager<PointType>::returnObjectMask(std::string waypoint, std::string object_id, std::string observation_xml)
{
    auto it =  m_waypointToObjMap.find(waypoint);
    if (it == m_waypointToObjMap.end() )
    {
        ROS_ERROR_STREAM("No objects loaded for waypoint "+waypoint);
        return;
    }

    std::vector<ObjStruct> objects = m_waypointToObjMap[waypoint];
    ObjStruct object;
    bool found = false;
    for (auto objectStruct : objects)
    {
        if (objectStruct.id == object_id)
        {
            found = true;
            object = objectStruct;
            break;
        }
    }

    if (!found)
    {
        ROS_ERROR_STREAM("Cannot find object "+object_id+" at waypoint "+waypoint);
        return;
    }


    // load observation and individual clouds
    SemanticRoom<PointType> observation = SemanticRoomXMLParser<PointType>::loadRoomFromXML(observation_xml,true);

    int argc = 0;
    char** argv;
    pcl::visualization::PCLVisualizer* p = new pcl::visualization::PCLVisualizer (argc, argv, "Metaroom consistency");
    p->addCoordinateSystem();
    CloudPtr object_cloud(new Cloud());
    *object_cloud = *object.cloud;

    // transform into the camera frame of ref (to compare with the individual clusters)
    Eigen::Matrix4f roomTransform = observation.getRoomTransform();
    if (observation.getIntermediateCloudTransforms().size() ==0)
    {
        ROS_ERROR_STREAM("Cannot get transform to origin.");
        return;
    }

    tf::StampedTransform transformToOrigin = observation.getIntermediateCloudTransforms()[0];

//    pcl::transformPointCloud (*object_cloud, *object_cloud, roomTransform.inverse());
//    pcl_ros::transformPointCloud(*object_cloud, *object_cloud,transformToOrigin.inverse());




    std::vector<tf::StampedTransform> allTransforms = observation.getIntermediateCloudTransformsRegistered();
    std::vector<CloudPtr> allClouds = observation.getIntermediateClouds();
    // frustrum for centroid projection

    pcl::FrustumCulling<PointType> fc;
    fc.setInputCloud (object_cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (15);
    fc.setNearPlaneDistance (.5);
    fc.setFarPlaneDistance (3.5);

    int max_overlap = 0;
    int best_index = -1;
    Eigen::Matrix4f best_transform;

    for (int j=0; j<allTransforms.size();j++)
    {

        Eigen::Matrix4f eigen_combined;
        pcl_ros::transformAsMatrix (transformToOrigin*allTransforms[j], eigen_combined);

        Eigen::Matrix4f cam2robot;
        cam2robot << 0, 0, 1, 0, 0,-1, 0, 0,1, 0, 0, 0, 0, 0, 0, 1;
        Eigen::Matrix4f pose_new = roomTransform * eigen_combined ;

        // set pose to frustum
        fc.setCameraPose (pose_new * cam2robot);
        pcl::PointCloud <PointType> target;
        fc.filter (target);

//        ROS_INFO_STREAM("Overlap "<<target.points.size() <<" max overlap"<<max_overlap);

        if (target.points.size() > max_overlap)
        {
//            ROS_INFO_STREAM("Intermediate cloud "<<j<<" contains the current dynamic cluster. No Points "<<target.points.size()<<"  total points "<<object_cloud->points.size());
            max_overlap = target.points.size();
            best_index = j;
            best_transform = pose_new;
        }

    }

    if (max_overlap > 0)
    {
        ROS_INFO_STREAM("Intermediate cloud "<<best_index<<" contains the current dynamic cluster. No Points "<<max_overlap<<"  total points "<<object_cloud->points.size());
        CloudPtr transformedCloud(new Cloud());
        pcl::transformPointCloud (*allClouds[best_index], *transformedCloud, best_transform);


        // filter out the cluster from the intermediate cloud
        CloudPtr filteredFromIntCloudCluster(new Cloud());
        std::vector<int> nn_indices (1);
        std::vector<float> nn_distances (1);
        std::vector<int> src_indices;
        pcl::SegmentDifferences<PointType> segment;
        typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (object_cloud);

          // Iterate through the source data set
          for (int i = 0; i < static_cast<int> (transformedCloud->points.size ()); ++i)
          {
            if (!isFinite (transformedCloud->points[i]))
              continue;
            // Search for the closest point in the target data set (number of neighbors to find = 1)
            if (!tree->nearestKSearch (transformedCloud->points[i], 1, nn_indices, nn_distances))
            {
              PCL_WARN ("No neighbor found for point %zu (%f %f %f)!\n", i, transformedCloud->points[i].x, transformedCloud->points[i].y, transformedCloud->points[i].z);
              continue;
            }

            if (nn_distances[0] < 0.001)
            {
                src_indices.push_back (i);
                filteredFromIntCloudCluster->push_back(transformedCloud->points[i]);
            }
          }




        ROS_INFO_STREAM("Filtered cluster from int cloud "<<filteredFromIntCloudCluster->points.size()<<"  inliers "<<src_indices.size());

        cv::Mat cluster_image = cv::Mat::zeros(480, 640, CV_8UC3);
        int top_y = -1, bottom_y = 640, top_x = -1, bottom_x = 640;
        for (int index : src_indices)
        {
            pcl::PointXYZRGB point = transformedCloud->points[index];
            int y = index / cluster_image.cols;
            int x = index % cluster_image.cols;
            cluster_image.at<cv::Vec3b>(y, x)[0] = point.b;
            cluster_image.at<cv::Vec3b>(y, x)[1] = point.g;
            cluster_image.at<cv::Vec3b>(y, x)[2] = point.r;

            if (y > top_y)
                top_y = y;
            if (x > top_x)
                top_x = x;
            if (y < bottom_y)
                bottom_y = y;
            if (x < bottom_x)
                bottom_x = x;

        }

        ROS_INFO_STREAM("Image ROI "<<bottom_y<<" "<<bottom_x<<" "<<top_y - bottom_y<<" "<<top_x - bottom_x);

//        if ((bottom_y < 0) || (bottom_x < 0) || (top_y - bottom_y < 0) || (top_x - bottom_x) < 0)
//        {
//        }

        cv::Rect rec(bottom_x,bottom_y,top_x - bottom_x, top_y - bottom_y);
        cv::Mat tmp = cluster_image(rec);



        cv::imwrite("image.jpg", cluster_image);


            cv::imshow( "Display window", cluster_image );                   // Show our image inside it.
            cv::waitKey(0);                                          // Wait for a keystroke in the window


            pcl::visualization::PointCloudColorHandlerCustom<PointType> cluster_handler (object_cloud, 0, 255, 0);
            pcl::visualization::PointCloudColorHandlerCustom<PointType> int_handler (transformedCloud, 255, 255, 0);
            p->addPointCloud(object_cloud,cluster_handler,"cluster");
            p->addPointCloud(transformedCloud,int_handler,"int");
            p->spin();
            p->removeAllPointClouds();

            // found a matching intermediate cloud
//            pcl::visualization::PointCloudColorHandlerCustom<PointType> int_cloud_handler (transformedCloud, 255, 0, 0);
//            pcl::visualization::PointCloudColorHandlerCustom<PointType> cluster_handler (c, 0, 255, 0);
//            pcl::visualization::PointCloudColorHandlerCustom<PointType> cluster_from_int_handler (filteredFromIntCloudCluster, 0, 0, 255);
//            p->addPointCloud (transformedCloud, int_cloud_handler, "int_cloud");
//            p->addPointCloud (c, cluster_handler, "cluster_cloud");
//            p->addPointCloud (filteredFromIntCloudCluster, cluster_from_int_handler, "cluster_from_int_cloud_cloud");
//            p->spin();
//            p->removeAllPointClouds();
    }

}

#endif
