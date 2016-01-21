#include <ros/ros.h>
#include <nbv_planning/NBVFinderROS.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/common/eigen.h>
#include <math.h>
#include <pcl/common/time.h>

/**
 * Helper class to wait for a single message on a given ROS topic, and return that message and unsubscribe
 */
template<class MessageType>
class WaitForMessage {
public:
    static MessageType get(const std::string &topic_name) {
        WaitForMessage waiter(topic_name);
        ROS_INFO_STREAM("Waiting for message on " << topic_name);
        while (!waiter.got_it_) {
            ros::spinOnce();
        }
        ROS_INFO("Got it.");
        return waiter.message_;
    }

private:
    WaitForMessage(const std::string &topic_name) : got_it_(false) {
        subscriber_ = node_handle_.subscribe(topic_name, 1, &WaitForMessage::callback, this);
    }

    ros::NodeHandle node_handle_;
    ros::Subscriber subscriber_;
    MessageType message_;
    bool got_it_;

    void callback(MessageType message) {
        subscriber_.shutdown();
        message_ = message;
        got_it_ = true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "nbv_planner");
    ros::NodeHandle n;

    tf::TransformListener tf_listener(ros::Duration(90));
    ros::Duration(4).sleep();
    // Read the parameters
    std::string camera_info_topic("/head_xtion/depth/camera_info");
    std::string camera_topic("/head_xtion/depth/points");

    // Get a sensor_msgs::CameraInfo message and use it to construct the sensor model
    sensor_msgs::CameraInfo camera_info = WaitForMessage<sensor_msgs::CameraInfo>::get(camera_info_topic);
    nbv_planning::SensorModel::ProjectionMatrix P(camera_info.P.data());
    nbv_planning::SensorModel sensor_model(camera_info.height, camera_info.width, P, 4, 0.3);

    ROS_INFO_STREAM("" << camera_info);

    nbv_planning::NBVFinderROS planner(sensor_model, n);

    tf::StampedTransform transform;
    ros::Time time = ros::Time::now();
    tf_listener.waitForTransform("/map","/base_link",time,ros::Duration(10));
    try {
        tf_listener.lookupTransform("/map", "/base_link", time, transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    nbv_planning::TargetVolume volume(0.05, Eigen::Vector3f(transform.getOrigin().x(),
                                                           transform.getOrigin().y(), 1.7), Eigen::Vector3f(0.5,0.5,0.3));
    planner.set_target_volume(volume);
    planner.publish_volume_marker();


    ROS_INFO("Getting cloud...");
    sensor_msgs::PointCloud2 cloud = WaitForMessage<sensor_msgs::PointCloud2>::get(camera_topic);
    Eigen::Affine3d origin;

    try {
        tf_listener.lookupTransform("/map", cloud.header.frame_id, cloud.header.stamp, transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    tf::transformTFToEigen(transform,origin);

    ROS_INFO("Updating map");
    planner.update_current_volume(cloud, origin);
    ROS_INFO("Publishing map");
    planner.publish_octomap();

    // Set some points to sample
    const unsigned int number_of_views = 20;
    const float robot_height = 1.7;
    std::vector<Eigen::Affine3d> views;
    Eigen::Affine3d view;
    Eigen::Vector3f extents = planner.get_target_volume().get_extents();
    Eigen::Vector3f volume_origin =   planner.get_target_volume().get_origin();
    std::cout << "Volume origin: " << volume_origin << std::endl;
    double distance_from_target = sqrt(extents[0]*extents[0] + extents[1]*extents[1])+0.5;
    Eigen::Affine3d shift;
    pcl::getTransformation(volume_origin[0], volume_origin[1], 0, 0, 0, 0, shift);
    pcl::getTransformation(0, distance_from_target, robot_height, M_PI_2, 0, 0, view);
    Eigen::Affine3d rotator;
    for (unsigned int i=0; i< number_of_views; i++) {
        pcl::getTransformation(0, 0, 0, 0, 0, i * (2.0*M_PI / (float)number_of_views), rotator);
        // Rotate the target view and append it to the view set
        views.push_back(shift*rotator*view);
        { pcl::ScopeTime timeit("evaluate_view");
                planner.evaluate_view(views.back());
        };
    }
    planner.set_candidate_views(views);
    planner.publish_views();

    std::cout << "Created planner, now spinning..." << std::endl;
    // This is a comment for no good reason

    ROS_INFO("Publishing map");
    planner.publish_octomap();
    try {
        ros::spin();
    } catch (std::runtime_error &e) {
        ROS_ERROR("nbv_planner_server exception: %s", e.what());
        return -1;
    }

    return 0;
}
