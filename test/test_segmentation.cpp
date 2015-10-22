#include <object_3d_retrieval/supervoxel_segmentation.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/visualize.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <metaroom_xml_parser/load_utilities.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using Graph = supervoxel_segmentation::Graph;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

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

    boost::filesystem::path data_path(argv[1]);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    for (const string& xml : folder_xmls) {
        CloudT::Ptr cloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointT>(xml);

        cout << "Cloud size: " << cloud->size() << endl;

        supervoxel_segmentation ss;
        Graph* g;
        vector<CloudT::Ptr> supervoxels;
        vector<CloudT::Ptr> convex_segments;
        map<size_t, size_t> indices;
        std::tie(g, supervoxels, convex_segments, indices) = ss.compute_convex_oversegmentation(cloud, true);

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
    }

    return 0;
}
