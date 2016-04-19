#include <object_3d_retrieval/supervoxel_segmentation.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/surfel_type.h>
#include <dynamic_object_retrieval/summary_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <semantic_map/RoomObservation.h>
#include <dynamic_object_retrieval/definitions.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#define VISUALIZE 0

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using Graph = supervoxel_segmentation::Graph;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

ros::Publisher room_pub;
ros::Publisher string_pub;
bool bypass_surfelize;
double threshold;
vector<string> sweep_xmls;
int sweep_ind;

void callback(const std_msgs::String::ConstPtr& msg)
{
    if (sweep_ind >= sweep_xmls.size()) {
        exit(0);
    }

    if (bypass_surfelize) {
        std_msgs::String sweep_msg;
        sweep_msg.data = sweep_xmls[sweep_ind++];
        string_pub.publish(sweep_msg);
    }
    else {
        semantic_map::RoomObservation sweep_msg;
        sweep_msg.xml_file_name = sweep_xmls[sweep_ind++];
        room_pub.publish(sweep_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "retrieval_simulate_observations");
    ros::NodeHandle n;

    ros::NodeHandle pn("~");
    string data_path;
    pn.param<string>("data_path", data_path, "");

    pn.param<bool>("bypass_surfelize", bypass_surfelize, true);

    sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path, true);
    sweep_ind = 0;

    if (bypass_surfelize) {
        string_pub = n.advertise<std_msgs::String>("/surfelization_done", 1);
    }
    else {
        room_pub = n.advertise<semantic_map::RoomObservation>("/local_metric_map/room_observations", 1);
    }
    ros::Subscriber sub = n.subscribe("/vocabulary_done", 1, callback);

    ros::Duration(1.0).sleep();

    if (bypass_surfelize) {
        std_msgs::String sweep_msg;
        sweep_msg.data = sweep_xmls[sweep_ind++];
        string_pub.publish(sweep_msg);
    }
    else {
        semantic_map::RoomObservation sweep_msg;
        sweep_msg.xml_file_name = sweep_xmls[sweep_ind++];
        room_pub.publish(sweep_msg);
    }

    ros::spin();

    return 0;
}
