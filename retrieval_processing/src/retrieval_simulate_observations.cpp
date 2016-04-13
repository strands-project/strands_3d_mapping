#include <object_3d_retrieval/supervoxel_segmentation.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/surfel_type.h>
#include <dynamic_object_retrieval/summary_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <metaroom_xml_parser/load_utilities.h>
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

ros::Publisher pub;
double threshold;
vector<string> sweep_xmls;
int sweep_ind;

void callback(const std_msgs::String::ConstPtr& msg)
{
    if (sweep_ind >= sweep_xmls.size()) {
        exit(0);
    }
    std_msgs::String sweep_msg;
    sweep_msg.data = sweep_xmls[sweep_ind++];
    pub.publish(sweep_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "retrieval_simulate_observations");
    ros::NodeHandle n;

    ros::NodeHandle pn("~");
    string data_path;
    pn.param<string>("data_path", data_path, "");

    sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path, true);
    sweep_ind = 0;

    pub = n.advertise<std_msgs::String>("/surfelization_done", 1);
    ros::Subscriber sub = n.subscribe("/vocabulary_done", 1, callback);

    std_msgs::String sweep_msg;
    sweep_msg.data = sweep_xmls[sweep_ind++];
    ros::Duration(1.0).sleep();
    pub.publish(sweep_msg);

    ros::spin();

    return 0;
}
