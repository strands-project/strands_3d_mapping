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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_test_node");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    string data_path;
    pn.param<std::string>("data_path", data_path, std::string(""));

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path, true);

    for (const string& sweep_xml : folder_xmls) {
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);
        semantic_map_load_utilties::IntermediateCloudCompleteData<PointT> data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(sweep_xml);

        geometry_msgs::Transform T;
        tf::transformTFToMsg(labels.transformToGlobal, T);
        image_geometry::PinholeCameraModel cm = data.vIntermediateRoomCloudCamParams[0];

        for (int i = 0; i < labels.objectClouds.size(); ++i) {
            quasimodo_msgs::query_cloud srv;

            pcl::toROSMsg(*labels.objectClouds[i], srv.request.cloud);

            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            labels.objectImages[i].copyTo(cv_ptr->image);
            cv_ptr->encoding = "bgr8";
            srv.request.image = *cv_ptr->toImageMsg();

            srv.request.camera = cm.cameraInfo();

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
            vis_srv.request.image = *cv_pub_ptr->toImageMsg();

            vis_srv.request.camera = cm.cameraInfo();
            vis_srv.request.room_transform = T;
            vis_srv.request.result = srv.response.result;

            ros::ServiceClient vis_client = n.serviceClient<quasimodo_msgs::visualize_query>("quasimodo_visualization_service");
            if (!vis_client.call(vis_srv)) {
                ROS_ERROR("Failed to call service quasimodo_visualization_service");
                return -1;
            }
        }
    }

    return 0;
}
