#include "retrieval_tools/surfel_type.h"
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide the path to the cloud and the path to the surfels..." << endl;
        return 0;
    }
    boost::filesystem::path cloud_path(argv[1]);
    boost::filesystem::path surfel_path(argv[2]);

    CloudT::Ptr cloud(new CloudT);
    pcl::io::loadPCDFile(cloud_path.string(), *cloud);

    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    pcl::io::loadPCDFile(surfel_path.string(), *surfel_cloud);

    pcl::KdTreeFLANN<SurfelT> kdtree;
    kdtree.setInputCloud(surfel_cloud);

    cout << "Analyzing convex segments..." << endl;

    set<int> already_inserted;

    SurfelCloudT::Ptr overlap_cloud(new SurfelCloudT);
    // now, associate each point in segment with a surfel in the surfel cloud!
    for (const PointT& p : cloud->points) {
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
        if (already_inserted.count(indices[0]) == 0) {
            SurfelT q = surfel_cloud->at(indices[0]);
            q.rgba = p.rgba;
            overlap_cloud->push_back(q);
            already_inserted.insert(indices[0]);
        }
    }

    boost::filesystem::path overlap_surfel_path = cloud_path.parent_path() / (cloud_path.stem().string() + "_surfel.pcd");
    pcl::io::savePCDFileBinary(overlap_surfel_path.string(), *overlap_cloud);

    return 0;
}

