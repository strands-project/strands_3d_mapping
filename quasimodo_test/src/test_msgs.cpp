#include "ros/ros.h"
#include <quasimodo_msgs/query_cloud.h>
#include <quasimodo_msgs/visualize_query.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

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

cv::Mat sweep_get_depth_at(const boost::filesystem::path& sweep_xml, size_t scan_index)
{
    stringstream ss;
    ss << "depth_" << std::setw(4) << std::setfill('0') << scan_index;
    boost::filesystem::path depth_path = sweep_xml.parent_path() / (ss.str() + ".png");
    cv::Mat depth_image = cv::imread(depth_path.string());
    return depth_image;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_test_node");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    string data_path;
    pn.param<std::string>("data_path", data_path, std::string(""));

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path, true);

    for (const string& sweep_xml : folder_xmls) {
		printf("doing sweep\n");
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);
        semantic_map_load_utilties::IntermediateCloudCompleteData<PointT> data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(sweep_xml);

        geometry_msgs::Transform T;
        tf::transformTFToMsg(labels.transformToGlobal, T);
        image_geometry::PinholeCameraModel cm = data.vIntermediateRoomCloudCamParams[0];

        for (int i = 0; i < labels.objectClouds.size(); ++i) {
			printf("doing label: %s\n",labels.objectLabels[i].c_str());
            quasimodo_msgs::query_cloud srv;

            pcl::toROSMsg(*labels.objectClouds[i], srv.request.query.cloud);

            convert_to_img_msg(labels.objectImages[i], srv.request.query.image);
            convert_to_depth_msg(sweep_get_depth_at(sweep_xml, labels.objectScanIndices[i]), srv.request.query.depth);
            convert_to_mask_msg(labels.objectMasks[i], srv.request.query.mask);

            srv.request.query.camera = cm.cameraInfo();
            srv.request.query.number_query = 10;

            srv.request.query.room_transform = T;

            ros::ServiceClient client = n.serviceClient<quasimodo_msgs::query_cloud>("quasimodo_retrieval_service");
            if (!client.call(srv)) {
                ROS_ERROR("Failed to call service quasimodo_retrieval_service");
                return -1;
            }

            quasimodo_msgs::visualize_query vis_srv;

            cv::Mat inverted_mask;
            cv::bitwise_not(labels.objectMasks[i], inverted_mask);
            cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
            cv_pub_ptr->image = labels.objectImages[i];
            cv_pub_ptr->image.setTo(cv::Scalar(255, 255, 255), inverted_mask);
            cv_pub_ptr->encoding = "bgr8";

            vis_srv.request.query = srv.request.query;
            vis_srv.request.result = srv.response.result;
            vis_srv.request.query.image = *cv_pub_ptr->toImageMsg();

            ros::ServiceClient vis_client = n.serviceClient<quasimodo_msgs::visualize_query>("quasimodo_visualization_service");
            if (!vis_client.call(vis_srv)) {
                ROS_ERROR("Failed to call service quasimodo_visualization_service");
                return -1;
            }
        }
    }

    return 0;
}
