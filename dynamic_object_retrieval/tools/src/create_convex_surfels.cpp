#include "dynamic_object_retrieval/summary_iterators.h"
#include <metaroom_xml_parser/load_utilities.h>
#include "retrieval_tools/surfel_type.h"
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <stdlib.h>
#include <time.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

using namespace std;

int main(int argc, char** argv)
{
    const int colormap[][3] = {
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
        {255,237,111},
        {255, 179, 0},
        {128, 62, 117},
        {255, 104, 0},
        {166, 189, 215},
        {193, 0, 32},
        {206, 162, 98},
        {0, 125, 52},
        {246, 118, 142},
        {0, 83, 138},
        {255, 122, 92},
        {83, 55, 122},
        {255, 142, 0},
        {179, 40, 81},
        {244, 200, 0},
        {127, 24, 13},
        {147, 170, 0},
        {89, 51, 21},
        {241, 58, 19},
        {35, 44, 22}
    };

    if (argc < 2) {
        cout << "Please provide the path to the sweep..." << endl;
        return 0;
    }
    boost::filesystem::path sweep_path = boost::filesystem::path(argv[1]);
    boost::filesystem::path surfel_path = sweep_path / "surfel_map.pcd";

    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    pcl::io::loadPCDFile(surfel_path.string(), *surfel_cloud);

    cout << "Creating iterators..." << endl;

    // get all convex segments for this sweep
    dynamic_object_retrieval::sweep_convex_segment_cloud_map segments(sweep_path);
    dynamic_object_retrieval::sweep_convex_segment_index_map indices(sweep_path);

    pcl::KdTreeFLANN<SurfelT> kdtree;
    kdtree.setInputCloud(surfel_cloud);

    cout << "Analyzing convex segments..." << endl;

    srand(time(NULL));
    size_t modulo = rand() % 44;

    SurfelCloudT::Ptr colored_cloud(new SurfelCloudT);
    for (auto tup : dynamic_object_retrieval::zip(segments, indices)) {
        // here it seems like we're gonna have to wrap our own solution by wrapping two octrees (or voxelgrids?)
        CloudT::Ptr c;
        size_t index;
        tie(c, index) = tup;
        index += modulo;

        // now, associate each point in segment with a surfel in the surfel cloud!
        for (const PointT& p : c->points) {
            if (!pcl::isFinite(p)) {
                continue;
            }
            vector<int> indices;
            vector<float> distances;
            SurfelT s; s.x = p.x; s.y = p.y; s.z = p.z;
            kdtree.nearestKSearchT(s, 1, indices, distances);
            if (distances.empty()) {
                cout << "Distances empty, wtf??" << endl;
                exit(0);
            }
            SurfelT q = surfel_cloud->at(indices[0]);
            uint8_t* rgb = (uint8_t*)(&q.rgba);
            rgb[2] = colormap[index % 44][0];
            rgb[1] = colormap[index % 44][1];
            rgb[0] = colormap[index % 44][2];
            colored_cloud->push_back(q);
        }
    }

    cout << "Cloud size: " << colored_cloud->size() << endl;

    boost::filesystem::path convex_surfel_path = sweep_path / "convex_surfels.pcd";
    pcl::io::savePCDFileBinary(convex_surfel_path.string(), *colored_cloud);

    return 0;
}
