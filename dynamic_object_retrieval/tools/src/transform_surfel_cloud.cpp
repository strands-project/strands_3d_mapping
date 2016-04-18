#include "retrieval_tools/surfel_type.h"
#include <pcl/io/pcd_io.h>

using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 6) {
        cout << "Please provide the path to the cloud and a quaternion w x y z..." << endl;
        return 0;
    }
    boost::filesystem::path cloud_path(argv[1]);
    float w = atof(argv[2]);
    float x = atof(argv[3]);
    float y = atof(argv[4]);
    float z = atof(argv[5]);

    Eigen::Quaternionf quat(w, x, y, z);
    Eigen::Matrix3f R = quat.matrix();

    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    pcl::io::loadPCDFile(cloud_path.string(), *surfel_cloud);

    // now, associate each point in segment with a surfel in the surfel cloud!
    for (SurfelT& p : surfel_cloud->points) {
        if (!pcl::isFinite(p)) {
            continue;
        }
        p.getVector3fMap() = R*p.getVector3fMap();
        p.getNormalVector3fMap() = R*p.getNormalVector3fMap();
    }

    boost::filesystem::path rotated_surfel_path = cloud_path.parent_path() / (cloud_path.stem().string() + "_rotated.pcd");
    pcl::io::savePCDFileBinary(rotated_surfel_path.string(), *surfel_cloud);

    return 0;
}

