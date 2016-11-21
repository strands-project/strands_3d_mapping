#include <quasimodo_conversions/conversions.h>
#include <soma_llsd/GetScene.h>
#include <soma_llsd/InsertScene.h>

#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace quasimodo_conversions {

void convert_to_img_msg(const cv::Mat& cv_image, sensor_msgs::Image& ros_image)
{
    cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
    cv_pub_ptr->image = cv_image;
    cv_pub_ptr->encoding = "bgr8";
    ros_image = *cv_pub_ptr->toImageMsg();
}

void convert_to_depth_msg(const cv::Mat& cv_image, sensor_msgs::Image& ros_image)
{
    cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
    cv_pub_ptr->image = cv_image;
    cv_pub_ptr->encoding = "mono16";
    ros_image = *cv_pub_ptr->toImageMsg();
}

void convert_to_mask_msg(const cv::Mat& cv_image, sensor_msgs::Image& ros_image)
{
    cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
    cv_pub_ptr->image = cv_image;
    cv_pub_ptr->encoding = "mono8";
    ros_image = *cv_pub_ptr->toImageMsg();
}

void model_to_soma_segment(ros::NodeHandle& n, const quasimodo_msgs::model& model, soma_llsd_msgs::Segment& segment)
{
    ros::ServiceClient client = n.serviceClient<soma_llsd::InsertScene>("/soma_llsd/insert_scene");
    ROS_INFO("Waiting for /soma_llsd/insert_scene service...");
    if (!client.waitForExistence(ros::Duration(1.0))) {
        ROS_INFO("Failed to get /soma_llsd/insert_scene service!");
        return;
    }
    ROS_INFO("Got /soma_llsd/insert_scene service");

    segment.id = std::to_string(model.model_id);

    size_t counter = 0;
    for (const quasimodo_msgs::rgbd_frame& frame : model.frames) {
        soma_llsd::InsertScene scene;
        scene.request.rgb_img = frame.rgb;
        scene.request.depth_img = frame.depth;
        scene.request.camera_info = frame.camera;
        scene.request.robot_pose = frame.pose; // not good actually but whatevs
        scene.request.cloud = model.clouds[counter];

        if (!client.call(scene)) {
            ROS_ERROR("Failed to call service /soma_llsd/insert_scene");
            return;
        }

        soma_llsd_msgs::Observation obs;
        obs.scene_id = scene.response.response.id;
        obs.camera_cloud = model.clouds[counter];
        obs.image_mask = model.masks[counter];
        //frame.camera = // see above
        //frame.frame_id = // see
        obs.pose = model.local_poses[counter]; //frame.pose;
        //obs.pose = model.local_poses[counter];
        obs.id = std::to_string(frame.frame_id);
        obs.timestamp = ros::Time::now().nsec;
        segment.observations.push_back(obs);
        ++counter;
    }

}

void soma_segment_to_model(ros::NodeHandle& n, const soma_llsd_msgs::Segment& segment, quasimodo_msgs::model& model)
{
    ros::ServiceClient client = n.serviceClient<soma_llsd::GetScene>("/soma_llsd/get_scene");
    ROS_INFO("Waiting for /soma_llsd/get_scene service...");
    if (!client.waitForExistence(ros::Duration(1.0))) {
        ROS_INFO("Failed to get /soma_llsd/get_scene service!");
        return;
    }
    ROS_INFO("Got /soma_llsd/get_scene service");

    // segment.scene_id; // this can be used to fetch the scene from mongodb, from which we can get the camera parameters

    model.model_id = std::stoi(segment.id);

    for (const soma_llsd_msgs::Observation& obs : segment.observations) {
        soma_llsd::GetScene srv;
        srv.request.scene_id = obs.scene_id;
        if (!client.call(srv)) {
            ROS_ERROR("Failed to call service /soma_llsd/get_scene");
            return;
        }
        model.clouds.push_back(obs.camera_cloud);
        quasimodo_msgs::rgbd_frame frame;
        frame.depth = srv.response.response.depth_img;
        frame.rgb = srv.response.response.rgb_img;
        model.masks.push_back(obs.image_mask);
        //frame.camera = // see above
        frame.frame_id = std::stoi(obs.id);
        //frame.pose = obs.pose;
        model.local_poses.push_back(obs.pose);
        model.frames.push_back(frame);
        model.local_poses.push_back(obs.pose);
        //model.global_pose = // could probably get this from scene also?
        //model.masks
    }
}

void frame_to_soma_observation(ros::NodeHandle& n, const quasimodo_msgs::rgbd_frame& frame, soma_llsd_msgs::Observation& obs)
{
    ros::ServiceClient client = n.serviceClient<soma_llsd::InsertScene>("/soma_llsd/insert_scene");
    ROS_INFO("Waiting for /soma_llsd/insert_scene service...");
    if (!client.waitForExistence(ros::Duration(1.0))) {
        ROS_INFO("Failed to get /soma_llsd/insert_scene service!");
        return;
    }
    ROS_INFO("Got /soma_llsd/insert_scene service");

    soma_llsd::InsertScene scene;
    scene.request.rgb_img = frame.rgb;
    scene.request.depth_img = frame.depth;
    scene.request.camera_info = frame.camera;
    scene.request.robot_pose = frame.pose; // not good actually but whatevs

    if (!client.call(scene)) {
        ROS_ERROR("Failed to call service /soma_llsd/insert_scene");
        return;
    }

    obs.scene_id = scene.response.response.id;
    obs.id = std::to_string(frame.frame_id);
    obs.pose = frame.pose;
}

void soma_observation_to_frame(ros::NodeHandle& n, const soma_llsd_msgs::Observation& obs, quasimodo_msgs::rgbd_frame& frame)
{
    ros::ServiceClient client = n.serviceClient<soma_llsd::GetScene>("/soma_llsd/get_scene");
    ROS_INFO("Waiting for /soma_llsd/get_scene service...");
    if (!client.waitForExistence(ros::Duration(1.0))) {
        ROS_INFO("Failed to get /soma_llsd/get_scene service!");
        return;
    }
    ROS_INFO("Got /soma_llsd/get_scene service");

    soma_llsd::GetScene srv;
    srv.request.scene_id = obs.scene_id;
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service /soma_llsd/get_scene");
        return;
    }

    frame.depth = srv.response.response.depth_img;
    frame.rgb = srv.response.response.rgb_img;
    //frame.camera = // see above
    frame.frame_id = std::stoi(obs.id);
    frame.pose = obs.pose; // not good actually but whatevs
}

//string episode_id
//string waypoint
//string meta_data
//uint32 timestamp

//tf/tfMessage transform
//sensor_msgs/PointCloud2 cloud
//sensor_msgs/Image rgb_img
//sensor_msgs/Image depth_img
//sensor_msgs/CameraInfo camera_info
//geometry_msgs/Pose robot_pose
//---
//bool result
//soma_llsd_msgs/Scene response

// ### response has id field! ###

void raw_frames_to_soma_scene(const cv::Mat& rgb, const cv::Mat& depth,
                              const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                              const Eigen::Matrix4d& pose, const Eigen::Matrix3d& K,
                              const std::string& waypoint, const std::string episode_id,
                              soma_llsd_msgs::Scene& scene)
{
    convert_to_img_msg(rgb, scene.rgb_img);
    convert_to_depth_msg(depth, scene.depth_img);
    pcl::toROSMsg(*cloud, scene.cloud);
    tf::poseEigenToMsg(Eigen::Affine3d(pose), scene.robot_pose);
    sensor_msgs::CameraInfo info;
    info.K.at(0) = K(0, 0);
    info.K.at(1) = K(0, 1);
    info.K.at(2) = K(0, 2);
    info.K.at(3) = K(1, 0);
    info.K.at(4) = K(1, 1);
    info.K.at(5) = K(1, 2);
    info.K.at(8) = K(2, 2);

    info.P.at(0) = K(0, 0);
    info.P.at(1) = K(0, 1);
    info.P.at(2) = K(0, 2);
    info.P.at(4) = K(1, 0);
    info.P.at(5) = K(1, 1);
    info.P.at(6) = K(1, 2);
    info.P.at(10) = K(2, 2);
    info.height = rgb.rows;
    info.width = rgb.cols;
    scene.waypoint = waypoint;
    scene.episode_id = episode_id;
}

} // namespace quasimodo_conversions
