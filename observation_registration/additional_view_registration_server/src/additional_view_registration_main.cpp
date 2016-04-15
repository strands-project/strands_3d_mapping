#include <ros/ros.h>
#include <observation_registration_services/AdditionalViewRegistrationService.h>
#include <observation_registration_services/ObjectAdditionalViewRegistrationService.h>
#include <semantic_map/room_xml_parser.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <metaroom_xml_parser/simple_dynamic_object_parser.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <geometry_msgs/Transform.h>
#include "additional_view_registration_server/additional_view_registration_optimizer.h"

typedef pcl::PointXYZRGB PointType;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;

using namespace std;

bool optimize(const vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& additional_views,
              const vector<tf::StampedTransform>& additional_views_odometry_transforms,
              const vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& observation_intermediate_clouds,
              const vector<tf::StampedTransform>& observation_intermediate_clouds_transforms,
              const tf::StampedTransform& observation_origin_transform,
              std::vector<int>& additional_view_constraints,
              vector<tf::StampedTransform>& additional_view_registered_transforms,
              int& observation_constraints,
              tf::StampedTransform& observation_transform);

bool additional_view_registration_service(
        observation_registration_services::AdditionalViewRegistrationService::Request  &req,
        observation_registration_services::AdditionalViewRegistrationService::Response &res)
{
    ROS_INFO("Received an additonal view registration request");
    ROS_INFO_STREAM("Observation " << req.observation_xml);
    ROS_INFO_STREAM("Number of AV: "<<req.additional_views.size());
    ROS_INFO_STREAM("Number of AV odometry transforms: "<<req.additional_views_odometry_transforms.size());

    // check additional view data
    if ((!req.additional_views.size())){
        // no additional views, cannot register
        ROS_ERROR_STREAM("additional_view_registration_service: no additional views provided. Cannot register.");
        return true;
    }

    if ((req.additional_views_odometry_transforms.size() != 0) && (req.additional_views_odometry_transforms.size() != req.additional_views.size())){
        ROS_ERROR_STREAM("additional_view_registration_service: the number of additional views provided differs from the number of odometry transforms. Will not use the odometry transforms");
        return true;
    }


    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> observation_clouds;
    std::vector<tf::StampedTransform> observation_transforms;
    tf::StampedTransform observation_origin_transform;

    // load observation
    if (req.observation_xml != ""){
        SemanticRoomXMLParser<PointType> parser;
        auto source_observation = parser.loadRoomFromXML(req.observation_xml);
        observation_clouds = source_observation.getIntermediateClouds();
        observation_transforms = source_observation.getIntermediateCloudTransformsRegistered();

        if (!source_observation.getIntermediateCloudTransforms().size()){
            ROS_ERROR_STREAM("object_additional_view_registration_service: the observations provided contains 0 registered transforms. Cannot register to observation -> will register only the additional views.");
            observation_origin_transform.setIdentity();
            observation_clouds.clear();
            observation_transforms.clear();
        } else {
            observation_origin_transform = source_observation.getIntermediateCloudTransforms()[0];
        }

        // check intermediate clouds
        if ((observation_clouds.size() == 0) || (observation_transforms.size() == 0)){
            // no intermediate clouds -> cannot register!
            ROS_ERROR_STREAM("additional_view_registration_service: the observations provided contain 0 intermediate clouds or 0 registered intermediate transforms. Cannot register to observation -> will register only the additional views.");
        }
    }

    // set up optimization data
    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> additional_views;
    vector<tf::StampedTransform> additional_views_odometry_transforms;
    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> observation_intermediate_clouds;
    vector<tf::StampedTransform> observation_intermediate_clouds_transforms;

    // object optimization data
    for (auto additional_view_msg : req.additional_views){
        boost::shared_ptr<pcl::PointCloud<PointType>> additional_view ( new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(additional_view_msg, *additional_view);
        additional_views.push_back(additional_view);
    }

    if ((req.additional_views_odometry_transforms.size() != 0) && (req.additional_views_odometry_transforms.size() == req.additional_views.size())){
        // get odometry transforms
        for (auto odometry : req.additional_views_odometry_transforms){
            tf::Transform view_transform;
            tf::transformMsgToTF(odometry, view_transform);
            tf::StampedTransform view_transform_stamped(view_transform, ros::Time::now(),"",""); // dummy values, not important
            additional_views_odometry_transforms.push_back(view_transform_stamped);
        }
    }

    // observation optimization data
    if ((observation_clouds.size() !=0) && (observation_transforms.size() !=0 ) &&
            (observation_clouds.size() == observation_transforms.size())){
        observation_intermediate_clouds.assign(observation_clouds.begin(), observation_clouds.end());
        observation_intermediate_clouds_transforms.assign(observation_transforms.begin(), observation_transforms.end());
    }

    // pass data to the optimizer
    std::vector<int> additional_view_constraints;
    vector<tf::StampedTransform> additional_view_registered_transforms;
    int observation_constraints;
    tf::StampedTransform observation_transform;

    ROS_INFO_STREAM("Registering additional views ...");
    optimize(additional_views, additional_views_odometry_transforms,
             observation_intermediate_clouds, observation_intermediate_clouds_transforms, observation_origin_transform,
             additional_view_constraints, additional_view_registered_transforms,
             observation_constraints, observation_transform);

    // return data
    // constraint number
    res.additional_view_correspondences.assign(additional_view_constraints.begin(), additional_view_constraints.end());
    res.observation_correspondences = observation_constraints;

    // additional view registered transforms
    for (auto reg_tf : additional_view_registered_transforms){
        geometry_msgs::Transform registered_transform_msg;
        tf::transformTFToMsg(reg_tf, registered_transform_msg);
        res.additional_view_transforms.push_back(registered_transform_msg);
    }

    // transform to observation
    geometry_msgs::Transform observation_transform_msg;
    tf::transformTFToMsg(observation_transform, observation_transform_msg);
    res.observation_transform = observation_transform_msg;

    return true;
}

bool object_additional_view_registration_service(
        observation_registration_services::ObjectAdditionalViewRegistrationService::Request  &req,
        observation_registration_services::ObjectAdditionalViewRegistrationService::Response &res)
{
    ROS_INFO("Received an object additional view registration request");
    ROS_INFO_STREAM("Observation " << req.observation_xml);
    ROS_INFO_STREAM("Object " << req.object_xml);

    // load object
    ObjectData object = semantic_map_load_utilties::loadDynamicObjectFromSingleSweep<PointType>(req.object_xml);

    // check that the object was found
    if (!object.objectCloud->points.size()){
        // could not load object
        ROS_ERROR_STREAM("object_additional_view_registration_service: could not load object from " << req.object_xml);
        throw std::runtime_error(
                    std::string("object_additional_view_registration_service: could not load object. \n"));
    }

    // check that the object has additional views
    if (!object.vAdditionalViews.size()){
        ROS_ERROR_STREAM("object_additional_view_registration_service: object from " << req.object_xml<<" has no additional views to register.");
        throw std::runtime_error(
                    std::string("object_additional_view_registration_service: object has no additional views. \n"));
    }

    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> observation_clouds;
    std::vector<tf::StampedTransform> observation_transforms;
    tf::StampedTransform observation_origin_transform;

    // load observation
    if (req.observation_xml != ""){
        SemanticRoomXMLParser<PointType> parser;
        auto source_observation = parser.loadRoomFromXML(req.observation_xml);
        observation_clouds = source_observation.getIntermediateClouds();
        observation_transforms = source_observation.getIntermediateCloudTransformsRegistered();

        if (!source_observation.getIntermediateCloudTransforms().size()){
            ROS_ERROR_STREAM("object_additional_view_registration_service: the observations provided contains 0 registered transforms. Cannot register to observation -> will register only the additional views.");
            observation_origin_transform.setIdentity();
            observation_clouds.clear();
            observation_transforms.clear();
        } else {
            observation_origin_transform = source_observation.getIntermediateCloudTransforms()[0];
        }

        // check intermediate clouds
        if ((observation_clouds.size() == 0) || (observation_transforms.size() == 0)){
            // no intermediate clouds -> cannot register!
            ROS_ERROR_STREAM("object_additional_view_registration_service: the observations provided contains 0 intermediate clouds or 0 registered intermediate transforms. Cannot register to observation -> will register only the additional views.");
        }
    }

    // set up optimization data
    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> additional_views;
    vector<tf::StampedTransform> additional_views_odometry_transforms;
    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> observation_intermediate_clouds;
    vector<tf::StampedTransform> observation_intermediate_clouds_transforms;

    // object optimization data
    additional_views.assign(object.vAdditionalViews.begin(), object.vAdditionalViews.end());
    additional_views_odometry_transforms.assign(object.vAdditionalViewsTransforms.begin(), object.vAdditionalViewsTransforms.end());
    // observation optimization data
    if ((observation_clouds.size() != 0) && (observation_transforms.size() != 0) &&
            (observation_clouds.size() == observation_transforms.size())){
        observation_intermediate_clouds.assign(observation_clouds.begin(), observation_clouds.end());
        observation_intermediate_clouds_transforms.assign(observation_transforms.begin(), observation_transforms.end());
    }

    // pass data to the optimizer
    std::vector<int> additional_view_constraints;
    vector<tf::StampedTransform> additional_view_registered_transforms;
    int observation_constraints;
    tf::StampedTransform observation_transform;

    ROS_INFO_STREAM("Registering additional views ...");
    optimize(additional_views, additional_views_odometry_transforms,
             observation_intermediate_clouds, observation_intermediate_clouds_transforms, observation_origin_transform,
             additional_view_constraints, additional_view_registered_transforms,
             observation_constraints, observation_transform);

    // return data
    // constraint number
    res.additional_view_correspondences.assign(additional_view_constraints.begin(), additional_view_constraints.end());
    res.observation_correspondences = observation_constraints;

    // additional view registered transforms
    for (auto reg_tf : additional_view_registered_transforms){
        geometry_msgs::Transform registered_transform_msg;
        tf::transformTFToMsg(reg_tf, registered_transform_msg);
        res.additional_view_transforms.push_back(registered_transform_msg);
    }

    // transform to observation
    geometry_msgs::Transform observation_transform_msg;
    tf::transformTFToMsg(observation_transform, observation_transform_msg);
    res.observation_transform = observation_transform_msg;


    return true;
}

bool optimize(const vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& additional_views,
              const vector<tf::StampedTransform>& additional_views_odometry_transforms,
              const vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& observation_intermediate_clouds,
              const vector<tf::StampedTransform>& observation_intermediate_clouds_transforms,
              const tf::StampedTransform& observation_origin_transform,
              std::vector<int>& additional_view_constraints,
              vector<tf::StampedTransform>& additional_view_registered_transforms,
              int& observation_constraints,
              tf::StampedTransform& observation_transform){

    bool register_to_observation = false;
    if((observation_intermediate_clouds.size()!=0) && (observation_intermediate_clouds.size() == observation_intermediate_clouds_transforms.size())
            /*&& (additional_views_odometry_transforms.size() != 0)*/){
        register_to_observation = true;
    }


    bool verbose = true;
    vector<tf::Transform> registered_transforms;
    tf::Transform transform_to_observation;
    AdditionalViewRegistrationOptimizer optimizer(verbose);
    optimizer.registerViews<PointType>(additional_views,additional_views_odometry_transforms,
                                       observation_intermediate_clouds, observation_intermediate_clouds_transforms, observation_origin_transform,
                                       additional_view_constraints,registered_transforms,
                                       observation_constraints,
                                       transform_to_observation,
                                       register_to_observation);

    for (auto tf : registered_transforms){
        tf::StampedTransform tf_stamped(tf, ros::Time::now(),"", "");
        additional_view_registered_transforms.push_back(tf_stamped);
    }

    observation_transform = tf::StampedTransform(transform_to_observation, ros::Time::now(),"","");

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "additional_view_registration_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("additional_view_registration_server", additional_view_registration_service);
    ros::ServiceServer service2 = n.advertiseService("object_additional_view_registration_server", object_additional_view_registration_service);
    ROS_INFO("additional_view_registration_server started.");
    ros::spin();

    return 0;
}
