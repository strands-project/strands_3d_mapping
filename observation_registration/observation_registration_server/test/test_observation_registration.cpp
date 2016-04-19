#include <semantic_map/room_xml_parser.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
#include <observation_registration_services/ObservationRegistrationService.h>
#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

using namespace std;

CloudPtr rebuildCloud(std::vector<CloudPtr> intermediate_clouds, std::vector<tf::StampedTransform> intermediate_transforms);

int main(int argc, char** argv)
{
    string obs_1;
    string obs_2;


    if (argc > 2){
        obs_1 = argv[1];
        obs_2 = argv[2];
    } else {
        cout<<"Please specify 2 observation xml files to register."<<endl;
        return -1;
    }

    ros::init(argc, argv, "test_observation_registration_service");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<observation_registration_services::ObservationRegistrationService>("observation_registration_server");
    observation_registration_services::ObservationRegistrationService srv;
    srv.request.source_observation_xml = obs_1;
    srv.request.target_observation_xml = obs_2;

    tf::Transform registered_transform;


    if (client.call(srv))
    {
        ROS_INFO_STREAM("Registration done. Number of constraints "<<srv.response.total_correspondences);
        if (srv.response.total_correspondences <= 0){
            ROS_INFO_STREAM("Registration unsuccessful due to insufficient constraints.");
            return -1;
        }
        tf::transformMsgToTF(srv.response.transform, registered_transform);
    }
    else
    {
        ROS_ERROR("Failed to call service observation_registration_server");
        return -1;
    }

    // check registration

    // first rebuild cloud
    SemanticRoomXMLParser<PointType> parser;
    auto obs1_data = parser.loadRoomFromXML(obs_1);
    auto obs2_data = parser.loadRoomFromXML(obs_2);

    CloudPtr obs1_cloud, obs2_cloud;

    if ((obs1_data.getIntermediateCloudTransformsRegistered().size() == 0) || (obs2_data.getIntermediateCloudTransformsRegistered().size() == 0)){
        ROS_INFO_STREAM("Using original transforms to build merged cloud");
        obs1_cloud = rebuildCloud(obs1_data.getIntermediateClouds(), obs1_data.getIntermediateCloudTransforms());
        obs2_cloud = rebuildCloud(obs2_data.getIntermediateClouds(), obs2_data.getIntermediateCloudTransforms());
    } else {
        ROS_INFO_STREAM("Using registered transforms to build merged cloud");
        obs1_cloud = rebuildCloud(obs1_data.getIntermediateClouds(), obs1_data.getIntermediateCloudTransformsRegistered());
        pcl_ros::transformPointCloud(*obs1_cloud, *obs1_cloud,obs1_data.getIntermediateCloudTransforms()[0]);
        obs2_cloud = rebuildCloud(obs2_data.getIntermediateClouds(), obs2_data.getIntermediateCloudTransformsRegistered());
        pcl_ros::transformPointCloud(*obs2_cloud, *obs2_cloud,obs2_data.getIntermediateCloudTransforms()[0]);
    }

    CloudPtr obs1_registered_cloud(new Cloud);
    pcl_ros::transformPointCloud(*obs1_cloud, *obs1_registered_cloud,registered_transform);
    // visualize: left - original, right - registered
    pcl::visualization::PCLVisualizer* pg = new pcl::visualization::PCLVisualizer (argc, argv, "test_observation_registration");
    int v1, v2;
    pg->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    pg->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    pg->addCoordinateSystem(1.0, v1);
    pg->addCoordinateSystem(1.0, v2);

    pcl::visualization::PointCloudColorHandlerCustom<PointType> obs1_handler_original (obs1_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> obs2_handler_original (obs2_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> obs1_handler_registered (obs1_registered_cloud, 255, 0, 0);
    pg->addPointCloud (obs1_cloud,obs1_handler_original,"obs1_cloud", v1);
    pg->addPointCloud (obs2_cloud,obs2_handler_original,"obs2_cloud", v1);

    pg->addPointCloud (obs1_registered_cloud,obs1_handler_registered,"obs1_cloud_registered", v2);
    pg->addPointCloud (obs2_cloud,obs2_handler_original,"obs2_cloud_2", v2);
    pg->spin();
    pg->removeAllPointClouds();




}


CloudPtr rebuildCloud(std::vector<CloudPtr> intermediate_clouds, std::vector<tf::StampedTransform> intermediate_transforms){
    CloudPtr mergedCloud(new Cloud);

    for (size_t i=0; i<intermediate_clouds.size(); i++)
    {
        Cloud transformed_cloud;
        pcl_ros::transformPointCloud(*intermediate_clouds[i], transformed_cloud,intermediate_transforms[i]);
        *mergedCloud+=transformed_cloud;
    }
    return mergedCloud;
}
