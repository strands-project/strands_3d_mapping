#include <semantic_map/room_xml_parser.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
#include <observation_registration_services/ObjectAdditionalViewRegistrationService.h>
#include <observation_registration_services/AdditionalViewRegistrationService.h>
#include <metaroom_xml_parser/simple_dynamic_object_parser.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;

using namespace std;

CloudPtr rebuildCloud(std::vector<CloudPtr> intermediate_clouds, std::vector<tf::StampedTransform> intermediate_transforms);

int main(int argc, char** argv)
{
    string obs;
    string object_xml;


    if (argc > 2){
        obs = argv[1];
        object_xml = argv[2];
    } else {
        ROS_ERROR_STREAM("Please specify observation xml and object xml.");
        return -1;
    }

    // load object
    ObjectData object = semantic_map_load_utilties::loadDynamicObjectFromSingleSweep<PointType>(object_xml);
    ROS_INFO_STREAM("Object has "<<object.vAdditionalViews.size()<<" additional views");

    ros::init(argc, argv, "test_additional_view_registration_service");
    ros::NodeHandle n;

    /************************************************ TEST FIRST SERVICE ********************************************************/
    ros::ServiceClient client = n.serviceClient<observation_registration_services::ObjectAdditionalViewRegistrationService>("object_additional_view_registration_server");
    observation_registration_services::ObjectAdditionalViewRegistrationService srv;
    srv.request.observation_xml = obs;
    srv.request.object_xml = object_xml;

    vector<tf::Transform> registered_transforms;

    ROS_INFO_STREAM("Testing object_additional_view_registration_server service");
    if (client.call(srv))
    {
        int total_constraints = 0;
        std::for_each(srv.response.additional_view_correspondences.begin(), srv.response.additional_view_correspondences.end(), [&] (int n) {
            total_constraints += n;
        });

        ROS_INFO_STREAM("Registration done. Number of additional view registration constraints "<<total_constraints<<". Number of additional view transforms "<<srv.response.additional_view_transforms.size());
        if (total_constraints <= 0){
            ROS_INFO_STREAM("Additional view Registration unsuccessful due to insufficient constraints.");
            return -1;
        }
        for (auto tr : srv.response.additional_view_transforms){
            tf::Transform transform;
            tf::transformMsgToTF(tr, transform);
            registered_transforms.push_back(transform);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service object_additional_view_registration_server");
        return -1;
    }

    // check registration
    ROS_INFO_STREAM("Visualizing registration");
    pcl::visualization::PCLVisualizer* pg = new pcl::visualization::PCLVisualizer (argc, argv, "test_additional_view_registration");
    pg->addCoordinateSystem(1.0);

    for (size_t i=0; i<object.vAdditionalViews.size();i++){
        CloudPtr transformedCloud1(new Cloud);
        pcl_ros::transformPointCloud(*object.vAdditionalViews[i], *transformedCloud1,registered_transforms[i]);

        stringstream ss; ss<<"Cloud";ss<<i;
        pg->addPointCloud(transformedCloud1, ss.str());
    }

    pg->spin();
    pg->removeAllPointClouds();


    /************************************************ TEST SECOND SERVICE ********************************************************/
    ros::ServiceClient client2 = n.serviceClient<observation_registration_services::AdditionalViewRegistrationService>("additional_view_registration_server");
    observation_registration_services::AdditionalViewRegistrationService srv2;
    srv2.request.observation_xml = obs;
    // copy object data here
    for (size_t i=0; i<object.vAdditionalViews.size(); i++){
//
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*object.vAdditionalViews[i], cloud_msg);
        srv2.request.additional_views.push_back(cloud_msg);

        geometry_msgs::Transform transform_msg;
        tf::transformTFToMsg(object.vAdditionalViewsTransforms[i],transform_msg);
        srv2.request.additional_views_odometry_transforms.push_back(transform_msg);
    }

    registered_transforms.clear();

    ROS_INFO_STREAM("Testing additional_view_registration_server service");
    if (client2.call(srv2))
    {
        int total_constraints = 0;
        std::for_each(srv2.response.additional_view_correspondences.begin(), srv2.response.additional_view_correspondences.end(), [&] (int n) {
            total_constraints += n;
        });

        ROS_INFO_STREAM("Registration done. Number of additional view registration constraints "<<total_constraints<<". Number of additional view transforms "<<srv2.response.additional_view_transforms.size());
        if (total_constraints <= 0){
            ROS_INFO_STREAM("Additional view Registration unsuccessful due to insufficient constraints.");
            return -1;
        }
        for (auto tr : srv2.response.additional_view_transforms){
            tf::Transform transform;
            tf::transformMsgToTF(tr, transform);
            registered_transforms.push_back(transform);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service object_additional_view_registration_server");
        return -1;
    }

    // check registration
    ROS_INFO_STREAM("Visualizing registration");

    for (size_t i=0; i<object.vAdditionalViews.size();i++){
        CloudPtr transformedCloud1(new Cloud);
        pcl_ros::transformPointCloud(*object.vAdditionalViews[i], *transformedCloud1,registered_transforms[i]);

        stringstream ss; ss<<"Cloud";ss<<i;
        pg->addPointCloud(transformedCloud1, ss.str());
    }

    pg->spin();
    pg->removeAllPointClouds();

}


