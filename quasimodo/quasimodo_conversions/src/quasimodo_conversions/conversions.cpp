#include <quasimodo_conversions/conversions.h>
#include <soma_llsd/GetScene.h>
#include <soma_llsd/InsertScene.h>

namespace quasimodo_conversions {

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

} // namespace quasimodo_conversions
