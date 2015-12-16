#include "object_3d_retrieval/supervoxel_segmentation.h"
#include "object_3d_retrieval/pfhrgb_estimation.h"
#include "object_3d_retrieval/shot_estimation.h"

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <dynamic_object_retrieval/definitions.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using Graph = supervoxel_segmentation::Graph;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;

class PointChooser {
public:
    PointChooser() : final_points(new CloudT), vis_src_(new pcl::visualization::PCLVisualizer ("Class Viewer", true)) {
        vis_src_->registerPointPickingCallback(&PointChooser::pp_callback, *this);
        vis_src_->registerKeyboardCallback(&PointChooser::keyboard_callback, *this);
    }
    void setInputCloud(CloudT::Ptr& subsegment_keypoints, CloudT::Ptr& cloud, vector<CloudT::Ptr> segments)
    {
        xyz_ = subsegment_keypoints;
        full_cloud = cloud;
        subsegments = segments;
        kdtree.setInputCloud(full_cloud);
        counter = 0;
    }
    void simpleVis()
    {
        vis_src_->setBackgroundColor(1, 1, 1);
        vis_src_->initCameraParameters();
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(xyz_);
        vis_src_->addPointCloud<PointT>(xyz_, rgb, "cloud");
        vis_src_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "cloud");
        while (!vis_src_->wasStopped()) {
            vis_src_->spinOnce(100);
        }
        vis_src_->close();
    }
    CloudT::Ptr get_points()
    {
        return final_points;
    }
protected:
    void pp_callback (const pcl::visualization::PointPickingEvent& event, void*)
    {
        if (event.getPointIndex() == -1) {
            return;
        }
        event.getPoint(picked_point.x, picked_point.y, picked_point.z);
        std::cout << "Temp point: " << picked_point.getVector3fMap().transpose() << std::endl;
    }
    void keyboard_callback(const pcl::visualization::KeyboardEvent &event, void*)
    {
        if (event.getKeySym () == "k" && event.keyDown ())
        {
            std::cout << "Picked point: " << picked_point.getVector3fMap().transpose() << std::endl;
            final_points->push_back(picked_point);

            for (CloudT::Ptr& c : subsegments) {
                auto it = std::find_if(c->points.begin(), c->points.end(), [this](const PointT& p) {
                    return (p.getVector3fMap() - picked_point.getVector3fMap()).norm() < 0.02;
                });
                if (it != c->points.end()) {
                    CloudT::Ptr resulting_cloud(new CloudT);
                    for (const PointT& p : c->points) {
                        if (!pcl::isFinite(p)) {
                            continue;
                        }
                        vector<int> indices;
                        vector<float> distances;
                        kdtree.radiusSearchT(p, 0.05, indices, distances);
                        for (int i : indices) {
                            resulting_cloud->push_back(full_cloud->points[i]);
                        }
                    }
                    cout << "Resulting cloud size: " << resulting_cloud->size() << endl;
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(resulting_cloud);
                    vis_src_->addPointCloud<PointT>(resulting_cloud, rgb, string("cloud") + to_string(counter));
                    vis_src_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                               1, string("cloud") + to_string(counter));
                    vis_src_->spinOnce(10);
                    ++counter;
                    break;
                }
            }
        }
    }
private:
    PointT picked_point;
    CloudT::Ptr final_points;
    // Point cloud data
    CloudT::Ptr xyz_;
    vector<CloudT::Ptr> subsegments;
    CloudT::Ptr full_cloud;
    pcl::KdTreeFLANN<PointT> kdtree;
    int counter;
    // The visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_src_;
};

void visualize_cloud(CloudT::Ptr& cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void visualize_feature_segmentation(CloudT::Ptr& segment_keypoints, CloudT::Ptr& cloud)
{
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(segment_keypoints);

    CloudT::Ptr resulting_cloud(new CloudT);
    for (const PointT& p : cloud->points) {
        if (!pcl::isFinite(p)) {
            continue;
        }
        vector<int> indices;
        vector<float> distances;
        kdtree.nearestKSearchT(p, 1, indices, distances);
        if (distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        float dist = sqrt(distances[0]);
        if (dist < 0.05) {
            resulting_cloud->push_back(p);
        }
    }

    visualize_cloud(resulting_cloud);
}

void pick_segments(CloudT::Ptr& subsegment_keypoints, CloudT::Ptr& cloud, vector<CloudT::Ptr> subsegments)
{

    // now click the object that we want in the vísualizer and find the segment corresponding to that
    PointChooser my_viewer;
    my_viewer.setInputCloud(subsegment_keypoints, cloud, subsegments); // A pointer to a cloud
    my_viewer.simpleVis();

    CloudT::Ptr picked = my_viewer.get_points();
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply a point cloud .pcd to segment..." << endl;
        return -1;
    }

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

    string cloud_file(argv[1]);

    CloudT::Ptr cloud(new CloudT);
    pcl::io::loadPCDFile(cloud_file, *cloud);

    supervoxel_segmentation ss;
    Graph* g;
    Graph* convex_g;
    vector<CloudT::Ptr> supervoxels;
    vector<CloudT::Ptr> convex_segments;
    map<size_t, size_t> indices;
    std::tie(g, convex_g, supervoxels, convex_segments, indices) = ss.compute_convex_oversegmentation(cloud, false);

    CloudT::Ptr subsegment_keypoints(new CloudT);
    vector<CloudT::Ptr> subsegments;
    int counter = 0;
    for (CloudT::Ptr& c : convex_segments) {
        // first, extract keypoints
        HistCloudT::Ptr features(new HistCloudT);
        CloudT::Ptr keypoints(new CloudT);
        pfhrgb_estimation::compute_features(features, keypoints, c, false);
        //shot_estimation::compute_features(features, keypoints, c, false);

        // then, cluster them
        vector<HistCloudT::Ptr> split_features;
        vector<CloudT::Ptr> split_keypoints;
        pfhrgb_estimation::split_descriptor_points(split_features, split_keypoints, features, keypoints, 30);
        //shot_estimation::split_descriptor_points(split_features, split_keypoints, features, keypoints, 30);

        // finally, visualize the keypoints
        for (CloudT::Ptr& k : split_keypoints) {
            subsegments.push_back(CloudT::Ptr(new CloudT));
            for (PointT p : k->points) {
                p.r = colormap[counter%24][0];
                p.g = colormap[counter%24][1];
                p.b = colormap[counter%24][2];
                subsegment_keypoints->push_back(p);
                subsegments.back()->push_back(p);
            }
            ++counter;
        }
    }

    visualize_cloud(subsegment_keypoints);

    pick_segments(subsegment_keypoints, cloud, subsegments);

    delete g;
    delete convex_g;

    return 0;
}
