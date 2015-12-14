#include "ros/ros.h"
#include "quasimodo_msgs/query_cloud.h"
#include "quasimodo_msgs/visualize_query.h"
#include "quasimodo_msgs/retrieval_query_result.h"

#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;

class verify_server {
public:
    ros::NodeHandle n;
    ros::Subscriber sub;
    string topic_input;

    verify_server(const std::string& name)
    {
        ros::NodeHandle pn("~");

        pn.param<std::string>("topic_input", topic_input, std::string("/retrieval_result"));

        sub = n.subscribe(topic_input, 5, &verify_server::callback, this);
    }

    void show_images(const sensor_msgs::Image& image,
                     const sensor_msgs::Image& depth,
                     const sensor_msgs::Image& mask)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(-1);
        }

        cv_bridge::CvImagePtr cv_depth_ptr;
        try {
            cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(-1);
        }

        cv_bridge::CvImagePtr cv_mask_ptr;
        try {
            cv_mask_ptr = cv_bridge::toCvCopy(mask, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(-1);
        }

        cv::imshow("Image", cv_ptr->image);
        cv::imshow("Depth", cv_depth_ptr->image);
        cv::imshow("Mask", cv_mask_ptr->image);
        cv::waitKey();
    }

    void show_images(const quasimodo_msgs::retrieval_query& query,
                     const quasimodo_msgs::retrieval_result& result)
    {
        show_images(query.image, query.depth, query.mask);
        for (int i = 0; i < result.retrieved_images.size(); ++i) {
            for (int j = 0; j < result.retrieved_images[i].images.size(); ++j) {
                show_images(result.retrieved_images[i].images[j], result.retrieved_depths[i].images[j], result.retrieved_masks[i].images[j]);
            }
        }
    }

    void callback(const quasimodo_msgs::retrieval_query_result& res)
    {
        show_images(res.query, res.result);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_verify_images");

    verify_server vs(ros::this_node::getName());

    ros::spin();

    return 0;
}
