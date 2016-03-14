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

class model_visualizer {
public:
    ros::NodeHandle n;
    ros::Subscriber sub;

    string input;

    model_visualizer(const std::string& name)
    {
        ros::NodeHandle pn("~");

        pn.param<std::string>("input", input, std::string("/models/new"));

        sub = n.subscribe(input, 1, &model_visualizer::callback, this);
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
                float d = 0.0002f*float(depth.at<uint16_t>(i, j));
                Eigen::Vector3f ep(float(j), float(i), 1.0f);
                ep = K.inverse()*ep;
                ep = d/ep(2)*ep;
                PointT p;
                p.getVector3fMap() = ep;
                cv::Vec3b colors = rgb.at<cv::Vec3b>(i, j);
                p.r = colors[0]; p.g = colors[1]; p.b = colors[2];
                cloud->push_back(p);
            }
        }
        CloudT::Ptr temp_cloud(new CloudT);
        pcl::transformPointCloud(*cloud, *temp_cloud, T);
        return temp_cloud;
    }

    void callback(const quasimodo_msgs::modelConstPtr& model)
    {
        int n = model->frames.size();
        CloudT::Ptr cloud(new CloudT);

        for (int i = 0; i < n; ++i) {
            image_geometry::PinholeCameraModel cam_model;
            cam_model.fromCameraInfo(model->frames[i].camera);
            cv::Matx33d cvK = cam_model.intrinsicMatrix();
            Eigen::Matrix3f K = Eigen::Map<Eigen::Matrix3d>(cvK.val).cast<float>();
            K << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f, 0.0f, 0.0f, 1.0f;

            Eigen::Affine3d e;
            tf::poseMsgToEigen(model->local_poses[i], e);
            Eigen::Matrix4f T = e.matrix().cast<float>();

            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(model->frames[i].rgb, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                exit(-1);
            }

            cv_bridge::CvImagePtr cv_depth_ptr;
            try {
                cv_depth_ptr = cv_bridge::toCvCopy(model->frames[i].depth, sensor_msgs::image_encodings::MONO16);
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                exit(-1);
            }

            cv_bridge::CvImagePtr cv_mask_ptr;
            try {
                cv_mask_ptr = cv_bridge::toCvCopy(model->masks[i], sensor_msgs::image_encodings::MONO8);
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                exit(-1);
            }

            CloudT::Ptr frame_cloud = construct_cloud(cv_ptr->image, cv_depth_ptr->image, cv_mask_ptr->image, K, T);
            *cloud += *frame_cloud;
        }

        /*
        pcl::toROSMsg(*query_cloud, res.query.cloud);
        convert_to_img_msg(query_image, res.query.image);
        convert_to_depth_msg(query_depth, res.query.depth);
        convert_to_mask_msg(query_mask, res.query.mask);
        res.query.camera = camera_info;
        res.query.number_query = number_query;
        tf::transformTFToMsg(room_transform, res.query.room_transform);
        */

        dynamic_object_retrieval::visualize(cloud);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_visualize_model");

    model_visualizer mv(ros::this_node::getName());

    ros::spin();

    return 0;
}

