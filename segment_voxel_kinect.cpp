#include "types.h"
#include "io_utils_kinect.h"
#include "utils.h"
#include "convex_voxel_segmentation.h"
#include "segment_features.h"

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

void transform_back(int frameid, Cloud_t::Ptr& cloud)
{
    string line;
    ifstream file("/home/nbore/Workspace/objectness_score/transformations10.txt");
    for (int i = 0; i < frameid; ++i) {
       getline(file, line);
    }
    getline(file, line);
    std::istringstream is(line.c_str());
    std::istream_iterator<float> start(is), end;
    vector<float> res(start, end);
    Eigen::Vector3f m(res[0], res[1], res[2]);
    Eigen::Quaternionf q(res[3], res[4], res[5], res[6]);
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    transform.matrix().block<3, 3>(0, 0) = q.matrix();
    transform.matrix().block<3, 1>(0, 3) = m;
    Cloud_t::Ptr result(new Cloud_t);
    pcl::transformPointCloud(*cloud, *result, transform.inverse());
    *cloud = *result;
}

// Take a scene id and segment the scene to its object hypotheses
//  outputs .ply files for object hypotheses and measures.eig.txt file that contains the objectness measures
int main(int argc, char** argv) {
    srand(1);

    assert(argc > 1);
    int sceneid= atoi(argv[1]);
    int frameid = atoi(argv[1]+2);
    string pcdfile = getScenePath(sceneid);
    if(!file_exists(pcdfile)) {
        cout << "EEEeee, file " << pcdfile << " does not exist. Quitting." << endl;
        return(0);
    }

    Cloud_t::Ptr cloud(new Cloud_t);
    pcl::PointCloud<pcl::PointXYZL>::Ptr result(new pcl::PointCloud<pcl::PointXYZL>);
    cout << "Loading file " << pcdfile << endl;
    pcl::io::loadPCDFile(pcdfile, *cloud);
    transform_back(frameid, cloud);

    std::vector<Cloud_t::Ptr> segments;
    convex_voxel_segmentation cvs(true);
    cvs.segment_objects(segments, cloud);

    for (Cloud_t::Ptr& segment : segments) {
        // for each segment, create features
        segment_features sf;
        Eigen::VectorXf feature;
        float th1 = 0.1;
        float th2 = 0.005;
        sf.calculate_features(feature, segment);
        if (feature(0) < th1) {
            std::cout << "Too thin: " << feature(0) << std::endl;
            continue;
        }
        else if (feature(1) < th2) {
            std::cout << "Too flat: " << feature(1) << std::endl;
            continue;
        }
        else if (feature(2) < 200) {
            std::cout << "Too few points: " << feature(2) << std::endl;
            continue;
        }

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segment);
        viewer->addPointCloud<pcl::PointXYZRGB> (segment, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
        }
    }

}
