#include <object_3d_retrieval/supervoxel_segmentation.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/surfel_type.h>
#include <dynamic_object_retrieval/summary_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <dynamic_object_retrieval/definitions.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#define VISUALIZE 1

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

int colormap[][3] = {
    {166,206,227},
    {31,120,180},
    {178,223,138},
    {51,160,44},
    {251,154,153},
    {227,26,28},
    {253,191,111},
    {255,127,0},
    {202,178,214},
    {106,61,154},
    {255,255,153},
    {177,89,40},
    {141,211,199},
    {255,255,179},
    {190,186,218},
    {251,128,114},
    {128,177,211},
    {253,180,98},
    {179,222,105},
    {252,205,229},
    {217,217,217},
    {188,128,189},
    {204,235,197},
    {255,237,111}
};

ros::Publisher pub;
ros::Publisher vis_cloud_pub;
double threshold;
dynamic_object_retrieval::data_summary data_summary;
boost::filesystem::path data_path;

void segmentation_callback(const std_msgs::String::ConstPtr& msg)
{
    data_summary.load(data_path);

    boost::filesystem::path sweep_xml(msg->data);
    boost::filesystem::path surfel_path = sweep_xml.parent_path() / "surfel_map.pcd";

    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    pcl::io::loadPCDFile(surfel_path.string(), *surfel_cloud);

    CloudT::Ptr cloud(new CloudT);
    NormalCloudT::Ptr normals(new NormalCloudT);
    cloud->reserve(surfel_cloud->size());
    normals->reserve(surfel_cloud->size());
    for (const SurfelT& s : surfel_cloud->points) {
        if (s.confidence < threshold) {
            continue;
        }
        PointT p;
        p.getVector3fMap() = s.getVector3fMap();
        p.rgba = s.rgba;
        NormalT n;
        n.getNormalVector3fMap() = s.getNormalVector3fMap();
        cloud->push_back(p);
        normals->push_back(n);
    }

    // we might want to save the map and normals here
    boost::filesystem::path cloud_path = sweep_xml.parent_path() / "cloud.pcd";
    boost::filesystem::path normals_path = sweep_xml.parent_path() / "normals.pcd";
    pcl::io::savePCDFileBinary(cloud_path.string(), *cloud);
    pcl::io::savePCDFileBinary(normals_path.string(), *normals);

    supervoxel_segmentation ss(0.02f, 0.2f, 0.4f, false); // do not filter
    Graph* g;
    Graph* convex_g;
    vector<CloudT::Ptr> supervoxels;
    vector<CloudT::Ptr> convex_segments;
    map<size_t, size_t> indices;
    std::tie(g, convex_g, supervoxels, convex_segments, indices) = ss.compute_convex_oversegmentation(cloud, normals, false);

#if VISUALIZE
    CloudT::Ptr colored_segments(new CloudT);
    colored_segments->reserve(cloud->size());
    int counter = 0;
    for (CloudT::Ptr& c : convex_segments) {
        for (PointT p : c->points) {
            p.r = colormap[counter%24][0];
            p.g = colormap[counter%24][1];
            p.b = colormap[counter%24][2];
            colored_segments->push_back(p);
        }
        ++counter;
    }
    //dynamic_object_retrieval::visualize(colored_segments);
    sensor_msgs::PointCloud2 vis_msg;
    pcl::toROSMsg(*colored_segments, vis_msg);
    vis_msg.header.frame_id = "/map";
    vis_cloud_pub.publish(vis_msg);
#endif

    boost::filesystem::path segments_path = sweep_xml.parent_path() / "convex_segments";
    boost::filesystem::create_directory(segments_path);
    ss.save_graph(*convex_g, (segments_path / "graph.cereal").string());

    delete g;
    delete convex_g;

    dynamic_object_retrieval::sweep_summary summary;
    summary.nbr_segments = convex_segments.size();

    int i = 0;
    vector<string> segment_paths;
    for (CloudT::Ptr& c : convex_segments) {
        std::stringstream ss;
        ss << "segment" << std::setw(4) << std::setfill('0') << i;
        boost::filesystem::path segment_path = segments_path / (ss.str() + ".pcd");
        segment_paths.push_back(segment_path.string());
        pcl::io::savePCDFileBinary(segment_path.string(), *c);
        summary.segment_indices.push_back(i); // counter
        ++i;
    }

    summary.save(segments_path);

    std_msgs::String done_msg;
    done_msg.data = msg->data;
    pub.publish(done_msg);

    data_summary.nbr_sweeps++;
    data_summary.nbr_convex_segments += convex_segments.size();
    data_summary.index_convex_segment_paths.insert(data_summary.index_convex_segment_paths.end(),
                                                   segment_paths.begin(), segment_paths.end());

    data_summary.save(data_path);
}

void bypass_callback(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String done_msg;
    done_msg.data = msg->data;
    pub.publish(done_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "retrieval_segmenter");
    ros::NodeHandle n;

    ros::NodeHandle pn("~");
    pn.param<double>("threshold", threshold, 0.4);
    bool bypass;
    pn.param<bool>("bypass", bypass, 0);
    string temp_path;
    pn.param<string>("data_path", temp_path, "~/.semanticMap");
    data_path = boost::filesystem::path(temp_path);

    if (boost::filesystem::exists(data_path / "segments_summary.json")) {
        data_summary.load(data_path);
    }
    else {
        data_summary = dynamic_object_retrieval::data_summary();
        data_summary.nbr_sweeps = 0;
        data_summary.nbr_convex_segments = 0;
        data_summary.nbr_subsegments = 0;
        data_summary.subsegment_type = "convex_segment";
        data_summary.save(data_path);
    }

    pub = n.advertise<std_msgs::String>("/segmentation_done", 1);
    vis_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/retrieval_processing/segmentation_cloud", 1);

    ros::Subscriber sub;
    if (bypass) {
        sub = n.subscribe("/surfelization_done", 1, bypass_callback);
    }
    else {
        sub = n.subscribe("/surfelization_done", 1, segmentation_callback);
    }

    ros::spin();

    return 0;
}
