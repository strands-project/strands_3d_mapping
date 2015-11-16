#include "ros/ros.h"
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <dynamic_object_retrieval/visualize.h>
#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_visualization.h>
#include "quasimodo_msgs/query_cloud.h"
#include "quasimodo_msgs/visualize_query.h"
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;

class visualization_server {
public:
    ros::NodeHandle n;
    ros::ServiceServer service;
    ros::Publisher image_pub;
    string image_output;

    visualization_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        pn.param<std::string>("image_output", image_output, std::string("visualization_image"));

        image_pub = n.advertise<sensor_msgs::Image>(image_output, 1);

        service = n.advertiseService(name, &visualization_server::service_callback, this);
    }

    bool service_callback(quasimodo_msgs::visualize_query::Request& req,
                          quasimodo_msgs::visualize_query::Response& res)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }

        Eigen::Affine3d e;
        tf::transformMsgToEigen(req.room_transform, e);
        Eigen::Matrix4f T = e.matrix().cast<float>();
        T.col(3) << 0.0f, 0.0f, 0.0f, 1.0f;

        vector<CloudT::Ptr> retrieved_clouds;
        for (const sensor_msgs::PointCloud2& cloud : req.result.retrieved_clouds) {
            retrieved_clouds.push_back(CloudT::Ptr(new CloudT));
            pcl::fromROSMsg(cloud, *retrieved_clouds.back());
        }

        string query_label = "Query Image";
        vector<string> dummy_labels;
        for (int i = 0; i < req.result.retrieved_clouds.size(); ++i) {
            dummy_labels.push_back(string("result") + to_string(i));
        }
        cv::Mat visualization = benchmark_retrieval::make_visualization_image(cv_ptr->image, query_label, retrieved_clouds, dummy_labels, T);

        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->image = visualization;
        cv_pub_ptr->encoding = "bgr8";

        res.image = *cv_pub_ptr->toImageMsg();
        image_pub.publish(res.image);

        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_visualization_service");

    visualization_server vs(ros::this_node::getName());

    ros::spin();

    return 0;
}
