#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide the path to the mask and the path to the complete cloud..." << endl;
        return 0;
    }
    boost::filesystem::path mask_path(argv[1]);
    boost::filesystem::path complete_path(argv[2]);

    CloudT::Ptr mask(new CloudT);
    pcl::io::loadPCDFile(mask_path.string(), *mask);

    CloudT::Ptr complete(new CloudT);
    pcl::io::loadPCDFile(complete_path.string(), *complete);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(complete);

    cout << "Analyzing convex segments..." << endl;

    set<int> already_inserted;
    CloudT::Ptr overlap_cloud(new CloudT);
    // now, associate each point in segment with a surfel in the surfel cloud!
    for (PointT p : mask->points) {
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
        for (int i : indices) {
            if (already_inserted.count(i) != 0) {
                continue;
            }
            already_inserted.insert(i);
            //overlap_cloud->push_back(complete->at(i));
            p.rgba = complete->at(i).rgba;
            overlap_cloud->push_back(p);
        }
    }

    boost::filesystem::path overlap_path = complete_path.parent_path() / (complete_path.stem().string() + "_overlap.pcd");
    pcl::io::savePCDFileBinary(overlap_path.string(), *overlap_cloud);

    return 0;
}
