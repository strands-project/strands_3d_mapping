#include <object_3d_retrieval/supervoxel_segmentation.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/surfel_type.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <dynamic_object_retrieval/definitions.h>

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

/**
 * Some thoughts on were we need to look to improve the convex segmentation:
 *
 * We need to cut more often, e.g. the sofa in the first part of this sequence should not be attached to the wall
 *
 * We need to make sure that the PCL supervoxel segmentation returns the whole scene
 *
 */

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

    boost::filesystem::path cloud_path(argv[1]);

    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    pcl::io::loadPCDFile(cloud_path.string(), *surfel_cloud);

    CloudT::Ptr cloud(new CloudT);
    NormalCloudT::Ptr normals(new NormalCloudT);
    cloud->reserve(surfel_cloud->size());
    normals->reserve(surfel_cloud->size());
    for (const SurfelT& s : surfel_cloud->points) {
        PointT p;
        p.getVector3fMap() = s.getVector3fMap();
        p.rgba = s.rgba;
        NormalT n;
        n.getNormalVector3fMap() = s.getNormalVector3fMap();
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

    delete g;
    delete convex_g;

    return 0;
}

