#include <object_3d_retrieval/supervoxel_segmentation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <convex_segmentation/CloudArray.h>

#define VISUALIZE 0

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using PointNormalT = pcl::PointXYZRGBNormal;
using PointNormalCloudT = pcl::PointCloud<PointNormalT>;
using NormalCloudT = pcl::PointCloud<NormalT>;
using Graph = supervoxel_segmentation::Graph;

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

void segmentation_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    PointNormalCloudT::Ptr normal_cloud(new PointNormalCloudT);
    pcl::fromROSMsg(*msg, *normal_cloud);

    CloudT::Ptr cloud(new CloudT);
    NormalCloudT::Ptr normals(new NormalCloudT);
    cloud->reserve(normal_cloud->size());
    normals->reserve(normal_cloud->size());
    for (const PointNormalT& pn : normal_cloud->points) {
        PointT p;
        p.getVector3fMap() = pn.getVector3fMap();
        p.rgba = pn.rgba;
        NormalT n;
        n.getNormalVector3fMap() = pn.getNormalVector3fMap();
        cloud->push_back(p);
        normals->push_back(n);
    }

    supervoxel_segmentation ss(0.02f, 0.2f, 0.4f, false); // do not filter
    Graph* g;
    Graph* convex_g;
    vector<CloudT::Ptr> supervoxels;
    vector<CloudT::Ptr> convex_segments;
    map<size_t, size_t> indices;
    std::tie(g, convex_g, supervoxels, convex_segments, indices) = ss.compute_convex_oversegmentation(cloud, normals, false);

#if VISUALIZE
    CloudT::Ptr colored_segments(new CloudT);
    *colored_segments += *cloud;
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
    dynamic_object_retrieval::visualize(colored_segments);
#endif

    delete g;
    delete convex_g;

    convex_segmentation::CloudArray out_msg;
    out_msg.clouds.resize(convex_segments.size());

    for (int i = 0; i < convex_segments.size(); ++i) {
        pcl::toROSMsg(*convex_segments[i], out_msg.clouds[i]);
    }

    pub.publish(out_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convex_segmentation_node");
    ros::NodeHandle n;

    //ros::NodeHandle pn("~");
    //pn.param<double>("threshold", threshold, 0.4);

    pub = n.advertise<convex_segmentation::CloudArray>("/convex_segments", 1);

    ros::Subscriber sub = n.subscribe("/cloud_pcd", 1, segmentation_callback);

    ros::spin();

    return 0;
}
