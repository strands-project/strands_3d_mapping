#include <ros/ros.h>
#include <nbv_planning/NBVFinderROS.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "nbv_planning/SetTarget.h"
#include "nbv_planning/SetViews.h"
#include "nbv_planning/Update.h"
#include "nbv_planning/SelectNextView.h"
#include <eigen_conversions/eigen_msg.h>

/**
 * Helper class to wait for a single message on a given ROS topic, and return that message and unsubscribe
 */
template<class MessageType>
class WaitForMessage {
public:
    static MessageType get(const std::string &topic_name) {
        WaitForMessage waiter(topic_name);
//        ROS_INFO_STREAM("Waiting for message on " << topic_name);
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

class NBVFinderROSServer {
public:
    NBVFinderROSServer(ros::NodeHandle &node) : m_node_handle(node) {
        // Read the parameters
        std::string camera_info_topic, camera_topic;

        m_node_handle.param("camera_info_topic", camera_info_topic, std::string("/head_xtion/depth/camera_info"));
        m_node_handle.param("camera_topic", camera_topic, std::string("/points/depth/camera_info"));

        // Get a sensor_msgs::CameraInfo message and use it to construct the sensor model
        ROS_INFO_STREAM("Waiting for camera info on " << camera_info_topic );
        sensor_msgs::CameraInfo camera_info = WaitForMessage<sensor_msgs::CameraInfo>::get(camera_info_topic);
        nbv_planning::SensorModel::ProjectionMatrix P(camera_info.P.data());
        nbv_planning::SensorModel sensor_model(camera_info.height, camera_info.width, P, 4, 0.3);

        // Create the planner
        m_planner = nbv_planning::NBVFinderROS::Ptr(new nbv_planning::NBVFinderROS(sensor_model, m_node_handle));

        // Advertise some ROS services
        m_set_target_volume_srv = m_node_handle.advertiseService("set_target_volume",
                                                                 &NBVFinderROSServer::set_target_volume, this);
        m_select_next_view_srv = m_node_handle.advertiseService("select_next_view",
                                                                &NBVFinderROSServer::select_next_view, this);
        m_update_srv = m_node_handle.advertiseService("update", &NBVFinderROSServer::update, this);
        m_set_views_srv = m_node_handle.advertiseService("set_views", &NBVFinderROSServer::set_views, this);

    }

    void shutdown() {
        m_set_views_srv.shutdown();
        m_select_next_view_srv.shutdown();
        m_update_srv.shutdown();
        m_set_target_volume_srv.shutdown();
    }
private:
    bool set_target_volume(nbv_planning::SetTargetRequest &req, nbv_planning::SetTargetResponse &resp) {
        ROS_INFO_STREAM("Setting target volume.");
        nbv_planning::TargetVolume volume(0.05, Eigen::Vector3f(req.target_centroid.x, req.target_centroid.y,
                                                                req.target_centroid.z),
                                          Eigen::Vector3f(req.target_extents.x, req.target_extents.y,
                                                          req.target_extents.z));
        m_planner->set_target_volume(volume);
        m_planner->publish_volume_marker();
        resp.success = true;
        return true;
    }

    bool select_next_view(nbv_planning::SelectNextViewRequest &req, nbv_planning::SelectNextViewResponse &resp) {
        unsigned int view_index;
        double score;
        if (!m_planner->choose_next_view(req.disable_view, view_index, score)) {
            resp.success=false;
            return true;
        }
        resp.selected_view_index = view_index;
        resp.view_score = score;
        resp.success=true;

        return true;
    }

    bool update(nbv_planning::UpdateRequest &req, nbv_planning::UpdateResponse &resp) {
        Eigen::Affine3d sensor_origin;
        tf::poseMsgToEigen(req.view_pose, sensor_origin);
        m_planner->update_current_volume(req.view, sensor_origin);
        m_planner->publish_octomap();
        resp.success = true;
        return true;
    }

    bool set_views(nbv_planning::SetViewsRequest &req, nbv_planning::SetViewsResponse &resp) {
        std::vector<Eigen::Affine3d> views;
        for (unsigned int i=0; i< req.candidate_views.size(); i++) {
            Eigen::Affine3d view;
            tf::poseMsgToEigen(req.candidate_views[i], view);
            views.push_back(view);

        }
        m_planner->set_candidate_views(views);
        m_planner->publish_views();
        resp.success = true;
        return true;
    }


    ros::NodeHandle m_node_handle;
    nbv_planning::NBVFinderROS::Ptr m_planner;
    ros::ServiceServer m_set_target_volume_srv, m_select_next_view_srv, m_update_srv, m_set_views_srv;
};



int main(int argc, char **argv) {
    ros::init(argc, argv, "nbv_planner");
    ros::NodeHandle n("~");

    NBVFinderROSServer server(n);

    std::cout << "Planner server active, now spinning..." << std::endl;

    try {
        ros::spin();
    } catch (std::runtime_error &e) {
        ROS_ERROR("nbv_planner_server exception: %s", e.what());
        return -1;
    }
    std::cout << "Shutting down..." << std::endl;
    server.shutdown();

    return 0;
}
