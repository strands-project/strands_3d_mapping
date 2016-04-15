#include <ros/ros.h>
#include <observation_registration_services/ObservationRegistrationService.h>
#include <semantic_map/room_xml_parser.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <geometry_msgs/Transform.h>

#include "observation_registration_server/observation_registration_optimizer.h"

typedef pcl::PointXYZRGB PointType;

using namespace std;

bool observation_registration_service(
        observation_registration_services::ObservationRegistrationService::Request  &req,
        observation_registration_services::ObservationRegistrationService::Response &res)
{
    ROS_INFO("Received an observation registration request");
    ROS_INFO_STREAM("Target " << req.target_observation_xml);
    ROS_INFO_STREAM("Source " << req.source_observation_xml);
    ROS_INFO_STREAM("Registering source to target.");

    // initialize registered transform with identity
    tf::Transform registered_transform;
    registered_transform.setIdentity();
    geometry_msgs::Transform registered_transform_msg;
    tf::transformTFToMsg(registered_transform, registered_transform_msg);
    res.transform = registered_transform_msg;
    res.total_correspondences = -1;

    // load observations
    SemanticRoomXMLParser<PointType> parser;
    auto source_observation = parser.loadRoomFromXML(req.source_observation_xml);
    auto target_observation = parser.loadRoomFromXML(req.target_observation_xml);

    // get observation data
    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> source_observation_intermediate_clouds = source_observation.getIntermediateClouds();
    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> target_observation_intermediate_clouds = target_observation.getIntermediateClouds();
    std::vector<tf::StampedTransform> source_observation_transforms = source_observation.getIntermediateCloudTransforms();
    std::vector<tf::StampedTransform> target_observation_transforms = target_observation.getIntermediateCloudTransforms();
    std::vector<tf::StampedTransform> source_observation_transforms_registered = source_observation.getIntermediateCloudTransformsRegistered();
    std::vector<tf::StampedTransform> target_observation_transforms_registered = target_observation.getIntermediateCloudTransformsRegistered();

    // check intermediate clouds
    if ((source_observation_intermediate_clouds.size() == 0) || (target_observation_intermediate_clouds.size() == 0) ||
            (source_observation_transforms.size() == 0) || (target_observation_transforms.size() == 0)){
        // no intermediate clouds -> cannot register!
        ROS_ERROR_STREAM("observation_registration_service: the observations provided contain 0 intermediate clouds or 0 intermediate transforms. Cannot register.");
        throw std::runtime_error(
                    std::string("observation_registration_service: the observations "
                                "provided contain 0 intermediate clouds or 0 intermediate transforms. Cannot register.\n"));
        return true;
    }

    // registration data
    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_clouds_source, all_clouds_target;
    vector<tf::StampedTransform> all_initial_poses_source, all_initial_poses_target;
    tf::StampedTransform origin_source, origin_target;

    // find intermediate clouds to register
//    SweepParameters registration_sweep_parameters(-160, 40, 160, -30, -30, -30);
    SweepParameters registration_sweep_parameters = source_observation.m_SweepParameters;
    SweepParameters source_observation_parameters = source_observation.m_SweepParameters;
    SweepParameters target_observation_parameters = target_observation.m_SweepParameters;

//    for (size_t i=0; i<source_observation_intermediate_clouds.size(); i++){
    for (size_t i=0; i<registration_sweep_parameters.getNumberOfIntermediatePositions(); i++){
        int corresponding_source_index;
        registration_sweep_parameters.findCorrespondingPosition(source_observation_parameters,i,corresponding_source_index);
        int corresponding_target_index;
        registration_sweep_parameters.findCorrespondingPosition(target_observation_parameters, i, corresponding_target_index);

        if ((corresponding_source_index == -1) || (corresponding_target_index == -1)){ // could not find corresponding cloud -> sweeps not compatible (different ptu configurations maybe?)
            throw std::runtime_error(
                        std::string("observation_registration_service: the observations "
                                    "provided are not compatible for registration (different ptu configurations?). Cannot register.\n"));
            return true;
        }

        // get intermediate clouds
        boost::shared_ptr<pcl::PointCloud<PointType>> source_cloud ( new pcl::PointCloud<PointType>);
        *source_cloud = *source_observation_intermediate_clouds[corresponding_source_index];
        boost::shared_ptr<pcl::PointCloud<PointType>> target_cloud ( new pcl::PointCloud<PointType>);
        *target_cloud = *target_observation_intermediate_clouds[corresponding_target_index];
        all_clouds_source.push_back(source_cloud);
        all_clouds_target.push_back(target_cloud);

        // get intermediate cloud transforms
        if((source_observation_transforms_registered.size() == 0) || (target_observation_transforms_registered.size() == 0)){
            // at least one of the observations doesn't have registered transforms set -> use original (odometry) transforms
            all_initial_poses_source.push_back(source_observation_transforms[corresponding_source_index]);
            all_initial_poses_target.push_back(target_observation_transforms[corresponding_target_index]);
        } else {
            all_initial_poses_source.push_back(source_observation_transforms_registered[corresponding_source_index]);
            all_initial_poses_target.push_back(target_observation_transforms_registered[corresponding_target_index]);
        }
    }

    // set observation origin
    if((source_observation_transforms_registered.size() == 0) || (target_observation_transforms_registered.size() == 0)){
        // at least one of the observations doesn't have registered transforms set -> the transform to the map frame is already contained in the intermediate transforms
        ROS_INFO_STREAM("Using original (odometry) intermediate cloud transforms");
        origin_source.setIdentity();
        origin_target.setIdentity();
    } else {
        ROS_INFO_STREAM("Using registered intermediate cloud transforms");
        origin_source = source_observation_transforms[0];
        origin_target = target_observation_transforms[0];
    }

    // optimize
    std::vector<int> number_of_constraints;
    bool verbose = true;
    ObservationRegistrationOptimizer optimizer(verbose);
    registered_transform = optimizer.registerObservation(all_clouds_source, all_initial_poses_source,
                                                         all_clouds_target, all_initial_poses_target,
                                                         number_of_constraints,
                                                         origin_source,
                                                         origin_target);

    if (number_of_constraints.size()){
        int total_constraints = 0;
        std::for_each(number_of_constraints.begin(), number_of_constraints.end(), [&] (int n) {
            total_constraints += n;
        });
        res.total_correspondences = total_constraints;
        tf::transformTFToMsg(registered_transform, registered_transform_msg);
        res.transform = registered_transform_msg;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "observation_registration_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("observation_registration_server", observation_registration_service);
    ROS_INFO("observation_registration_server started.");
    ros::spin();

    return 0;
}
