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
        cout << "Please provide paths of point clouds to line up..." << endl;
        return 0;
    }

    vector<SurfelCloudT::Ptr> surfel_clouds;
    for (int i = 1; i < argc; ++i) {
        boost::filesystem::path surfel_path(argv[i]);
        surfel_clouds.push_back(SurfelCloudT::Ptr(new SurfelCloudT));
        pcl::io::loadPCDFile(surfel_path.string(), *surfel_clouds.back());
    }

    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);

    Eigen::Vector3f zz(0.0f, 1.0f, 0.0f);

    int offset = -int(surfel_clouds.size()/2);
    for (SurfelCloudT::Ptr& c : surfel_clouds) {
        Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
        for (const SurfelT& s : c->points) {
            center += s.getVector3fMap();
        }
        center *= 1.0f/float(c->size());


        /*
        Eigen::Vector3f x = center;
        x.normalize();
        Eigen::Vector3f z = x.cross(zz);
        z.normalize();
        Eigen::Vector3f y = z.cross(x);
        y.normalize();
        Eigen::Matrix3f R;
        R << x, y, z;
        */
        Eigen::Vector3f x, y, z;
        z = center;
        z.normalize();
        x = Eigen::Vector3f(0.0f, 0.0f, -1.0f).cross(z);
        y = z.cross(x);

        Eigen::Matrix3f R;
        R.row(0) = x.transpose();
        R.row(1) = y.transpose();
        R.row(2) = z.transpose();

        for (SurfelT& s : c->points) {
            s.getVector3fMap() -= center;
            s.getVector3fMap() = R*s.getVector3fMap();
            s.getNormalVector3fMap() = R*s.getNormalVector3fMap();
            s.x += 0.5f*float(offset);
            s.radius *= 1.5;
        }

        *surfel_cloud += *c;

        ++offset;
    }

    pcl::io::savePCDFileBinary("lined_surfels.pcd", *surfel_cloud);

    return 0;
}


