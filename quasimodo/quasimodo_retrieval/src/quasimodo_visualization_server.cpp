#include "ros/ros.h"
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <dynamic_object_retrieval/visualize.h>
#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_visualization.h>
#include "quasimodo_msgs/query_cloud.h"
#include "quasimodo_msgs/visualize_query.h"
#include "quasimodo_msgs/retrieval_query_result.h"
#include "quasimodo_msgs/string_array.h"
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
    ros::Publisher label_pub;
    ros::Subscriber sub;
    string image_output;
    string topic_input;

    visualization_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        pn.param<std::string>("image_output", image_output, std::string("/quasimodo_retrieval/visualization"));
        pn.param<std::string>("topic_input", topic_input, std::string("/retrieval_result"));

        image_pub = n.advertise<sensor_msgs::Image>(image_output, 1, true);
        label_pub = n.advertise<quasimodo_msgs::string_array>("/retrieved_labels", 1);

        service = n.advertiseService(name, &visualization_server::service_callback, this);

        sub = n.subscribe(topic_input, 1, &visualization_server::callback, this);
    }

    //sensor_msgs/PointCloud2 cloud
    //sensor_msgs/Image image
    //sensor_msgs/Image depth
    //sensor_msgs/Image mask
    //sensor_msgs/CameraInfo camera
    //int32 number_query


    //sensor_msgs/Image image
    //sensor_msgs/CameraInfo camera
    //geometry_msgs/Transform room_transform
    quasimodo_msgs::visualize_query::Response vis_img_from_msgs(const quasimodo_msgs::retrieval_query& query,
                                                                const quasimodo_msgs::retrieval_result& result)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(query.image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(-1);
        }

        cv_bridge::CvImagePtr cv_mask_ptr;
        try {
            cv_mask_ptr = cv_bridge::toCvCopy(query.mask, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(-1);
        }

        cv::Mat inverted_mask;
        cv::bitwise_not(cv_mask_ptr->image, inverted_mask);
        cv_ptr->image.setTo(cv::Scalar(255, 255, 255), inverted_mask);

        Eigen::Affine3d e;
        tf::transformMsgToEigen(query.room_transform, e);
        Eigen::Matrix4f T = e.matrix().cast<float>();
        T.col(3) << 0.0f, 0.0f, 0.0f, 1.0f;

        vector<CloudT::Ptr> retrieved_clouds;
        for (const sensor_msgs::PointCloud2& cloud : result.retrieved_clouds) {
            retrieved_clouds.push_back(CloudT::Ptr(new CloudT));
            pcl::fromROSMsg(cloud, *retrieved_clouds.back());
        }

        string query_label = "Query Image";
        vector<string> dummy_labels;
        quasimodo_msgs::string_array label_msg;
        vector<boost::filesystem::path> sweep_paths;
        int i = 0;
        for (const auto& s : result.retrieved_image_paths) {
            boost::filesystem::path image_path(s.strings[0]);
            boost::filesystem::path sweep_path = image_path.parent_path();
            boost::filesystem::path annotation_path = sweep_path / "annotation.txt";
            if (boost::filesystem::exists(annotation_path)) {
                ifstream f(annotation_path.string());
                string label;
                f >> label;
                int index;
                f >> index;
                f.close();
                cout << "Found annotated index: " << index << endl;
                cout << "Comparing to retrieved: " << result.segment_indices[i].ints[0] << endl;
                if (index == result.segment_indices[i].ints[0]) {
                    dummy_labels.push_back(label);
                }
                else {
                    dummy_labels.push_back(string("result") + to_string(i));
                }
            }
            else {
                dummy_labels.push_back(string("result") + to_string(i));
            }
            label_msg.strings.push_back(dummy_labels.back());
            sweep_paths.push_back(sweep_path / "room.xml");
            ++i;
        }

        label_pub.publish(label_msg);

        vector<cv::Mat> individual_images;
        cv::Mat visualization = benchmark_retrieval::make_visualization_image(cv_ptr->image, query_label, retrieved_clouds,
                                                                              sweep_paths, dummy_labels, T, individual_images);



        quasimodo_msgs::visualize_query::Response resp;
        {
            cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
            cv_pub_ptr->image = visualization;
            cv_pub_ptr->encoding = "bgr8";
            resp.image = *cv_pub_ptr->toImageMsg();
        }
        for (cv::Mat& im : individual_images) {
            cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
            cv_pub_ptr->image = im;
            cv_pub_ptr->encoding = "bgr8";
            resp.images.images.push_back(*cv_pub_ptr->toImageMsg());
        }

        return resp;
    }

    bool service_callback(quasimodo_msgs::visualize_query::Request& req,
                          quasimodo_msgs::visualize_query::Response& res)
    {
        res = vis_img_from_msgs(req.query, req.result);

        image_pub.publish(res.image);

        return true;
    }

    void callback(const quasimodo_msgs::retrieval_query_result& res)
    {
        quasimodo_msgs::visualize_query::Response resp = vis_img_from_msgs(res.query, res.result);

        image_pub.publish(resp.image);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_visualization_service");

    visualization_server vs(ros::this_node::getName());

    ros::spin();

    return 0;
}
