#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <object_3d_benchmark/benchmark_retrieval.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <dynamic_object_retrieval/definitions.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>

#include <k_means_tree/k_means_tree.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>

#include "ros/ros.h"
#include "quasimodo_msgs/retrieval_query_result.h"
#include "quasimodo_msgs/model.h"
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

class retrieval_cloud_visualizer {
public:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pubs[10];

    string input;

    retrieval_cloud_visualizer(const std::string& name)
    {
        ros::NodeHandle pn("~");

        pn.param<std::string>("input", input, std::string("/retrieval_result"));

        sub = n.subscribe(input, 1, &retrieval_cloud_visualizer::callback, this);

        for (size_t i = 0; i < 10; ++i) {
            pubs[i] = n.advertise<sensor_msgs::PointCloud2>(string("/retrieval_raw_cloud/") + to_string(i), 1, true);
        }
    }

    CloudT::Ptr construct_cloud(cv::Mat& rgb, cv::Mat& depth, cv::Mat& mask, const Eigen::Matrix3f& K, const Eigen::Matrix4f& T)
    {
        int height = rgb.rows;
        int width = rgb.cols;

        CloudT::Ptr cloud(new CloudT);
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                if (mask.at<uchar>(i, j) == 0) {
                    continue;
                }
                float d = 0.001f*float(depth.at<uint16_t>(i, j));
                Eigen::Vector3f ep(float(j), float(i), 1.0f);
                ep = K.inverse()*ep;
                ep = d/ep(2)*ep;
                PointT p;
                p.getVector3fMap() = ep;
                cv::Vec3b colors = rgb.at<cv::Vec3b>(i, j);
                p.r = colors[2]; p.g = colors[1]; p.b = colors[0];
                cloud->push_back(p);
            }
        }
        CloudT::Ptr temp_cloud(new CloudT);
        pcl::transformPointCloud(*cloud, *temp_cloud, T);
        return temp_cloud;
    }

    void callback(const quasimodo_msgs::retrieval_query_result& result)
    {
        const int m = 10;//result.result.retrieved_clouds.size(); // 10!

        for (int j = 0; j < m; ++j) {
            int n = result.result.retrieved_images[j].images.size();
            CloudT::Ptr cloud(new CloudT);

            for (int i = 0; i < n; ++i) {
                Eigen::Matrix3f K; // = Eigen::Map<Eigen::Matrix3d>(cvK.val).cast<float>();
                K << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f, 0.0f, 0.0f, 1.0f;

                Eigen::Affine3d e;
                tf::poseMsgToEigen(result.result.retrieved_initial_poses[j].poses[i], e);
                Eigen::Matrix4f T = e.matrix().cast<float>();

                cv_bridge::CvImagePtr cv_ptr;
                try {
                    cv_ptr = cv_bridge::toCvCopy(result.result.retrieved_images[j].images[i], sensor_msgs::image_encodings::BGR8);
                }
                catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    exit(-1);
                }

                cv_bridge::CvImagePtr cv_depth_ptr;
                try {
                    cv_depth_ptr = cv_bridge::toCvCopy(result.result.retrieved_depths[j].images[i], sensor_msgs::image_encodings::MONO16);
                }
                catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    exit(-1);
                }

                cv_bridge::CvImagePtr cv_mask_ptr;
                try {
                    cv_mask_ptr = cv_bridge::toCvCopy(result.result.retrieved_masks[j].images[i], sensor_msgs::image_encodings::MONO8);
                }
                catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    exit(-1);
                }

                CloudT::Ptr frame_cloud = construct_cloud(cv_ptr->image, cv_depth_ptr->image, cv_mask_ptr->image, K, T);
                *cloud += *frame_cloud;
            }

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*cloud, msg_cloud);
            msg_cloud.header.stamp = ros::Time::now();
            msg_cloud.header.frame_id = "/map";
            //tf::transformTFToMsg(room_transform, res.query.room_transform);
            pubs[j].publish(msg_cloud);

        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_visualize_retrieval_cloud");

    retrieval_cloud_visualizer vrc(ros::this_node::getName());

    ros::spin();

    return 0;
}

