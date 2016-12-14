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
#include <cv_bridge/cv_bridge.h>

#include <quasimodo_msgs/mask_pointclouds.h>

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
ros::ServiceServer service;
double threshold;
dynamic_object_retrieval::data_summary data_summary;
boost::filesystem::path data_path;

bool segmentation_service(quasimodo_msgs::mask_pointclouds::Request& req, quasimodo_msgs::mask_pointclouds::Response& resp)
{
    for (size_t i = 0; i < req.clouds.size(); ++i) {
        CloudT::Ptr cloud(new CloudT);
        pcl::fromROSMsg(req.clouds[i], *cloud);
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(req.masks[i], sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(-1);
        }
        cv::Mat mask;
        cv::cvtColor(cv_ptr->image, mask, CV_BGR2GRAY);

        for (size_t y = 0; y < mask.rows; ++y) {
            for (size_t x = 0; x < mask.cols; ++x) {
                size_t index = y*mask.cols + x;
                if (int(mask.at<uchar>(y, x)) != 255) {
                    cloud->points[index].x = std::numeric_limits<float>::infinity();
                    cloud->points[index].y = std::numeric_limits<float>::infinity();
                    cloud->points[index].z = std::numeric_limits<float>::infinity();
                }
            }
        }

        sensor_msgs::PointCloud2 masked_msg;
        pcl::toROSMsg(*cloud, masked_msg);
        resp.clouds.push_back(masked_msg);
    }

    return true;
}

bool maybe_append(const boost::filesystem::path& segments_path)
{
    {
        std::stringstream ss;
        ss << "segment" << std::setw(4) << std::setfill('0') << 0;
        boost::filesystem::path segment_path = segments_path / (ss.str() + ".pcd");
        if (std::find(data_summary.index_convex_segment_paths.begin(),
                      data_summary.index_convex_segment_paths.end(),
                      segment_path.string()) !=
                data_summary.index_convex_segment_paths.end()) {
            return false;
        }
    }

    dynamic_object_retrieval::sweep_summary summary;
    summary.load(segments_path);

    for (int i : summary.segment_indices) {
        std::stringstream ss;
        ss << "segment" << std::setw(4) << std::setfill('0') << i;
        boost::filesystem::path segment_path = segments_path / (ss.str() + ".pcd");
        data_summary.index_convex_segment_paths.push_back(segment_path.string());
    }

    summary.save(segments_path);

    return true;
}

void segmentation_callback(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String done_msg;
    done_msg.data = msg->data;

    data_summary.load(data_path);

    boost::filesystem::path sweep_xml(msg->data);
    boost::filesystem::path surfel_path = sweep_xml.parent_path() / "surfel_map.pcd";
    boost::filesystem::path segments_path = sweep_xml.parent_path() / "convex_segments";
    if (boost::filesystem::exists(segments_path)) {
        cout << "Convex segments " << segments_path.string() << " already exist, finishing sweep " << msg->data << "..." << endl;
        maybe_append(segments_path);
        pub.publish(done_msg);
        return;
    }

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
    service = n.advertiseService("/retrieval_segmentation_service", segmentation_service);

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
