#include "types.h"
#include "io_utils_kinect.h"
#include "utils.h"
#include "convex_voxel_segmentation.h"
#include "segment_features.h"
#include "k_means_tree/k_means_tree.h"

//#define PCL_NO_PRECOMPILE
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

/*POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::Histogram<33>,
        (float, histogram, histogram)
)*/

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
    /*srand(1);

    assert(argc > 1);
    int sceneid= atoi(argv[1]);
    int frameid = atoi(argv[1]+2);*/

    using PointT = pcl::Histogram<33>;
    using HistCloudT = pcl::PointCloud<PointT>;
    using CloudT = pcl::PointCloud<pcl::PointXYZRGB>;

    std::vector<int> feature_inds;
    HistCloudT::Ptr all_local_features(new HistCloudT);
    std::vector<Cloud_t::Ptr> all_segments;
    size_t counter = 0;
    size_t temp = 0;
    size_t segment_id = 191;
    for (int sceneid = 1000; sceneid <= 1006; ++sceneid) {
        int frameid = sceneid - 1000;
        string pcdfile = getScenePath(sceneid);
        if(!file_exists(pcdfile)) {
            cout << "EEEeee, file " << pcdfile << " does not exist. Quitting." << endl;
            return(0);
        }

        Cloud_t::Ptr cloud(new Cloud_t);
        cout << "Loading file " << pcdfile << endl;
        pcl::io::loadPCDFile(pcdfile, *cloud);
        transform_back(frameid, cloud);

        std::vector<Cloud_t::Ptr> segments;
        std::vector<NormalCloud_t::Ptr> segment_normals;
        std::vector<Cloud_t::Ptr> full_segments;
        convex_voxel_segmentation cvs(false);
        cvs.segment_objects(segments, segment_normals, full_segments, cloud);

        std::vector<HistCloudT::Ptr> local_features(full_segments.size());
        std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > global_features(full_segments.size());
        for (size_t i = 0; i < full_segments.size(); ++i) {
            all_segments.push_back(full_segments[i]);
            // for each segment, create features
            segment_features sf(false);
            float th1 = 0.1;
            float th2 = 0.005;
            local_features[i] = HistCloudT::Ptr(new HistCloudT);
            // let's save all features to begin with
            sf.calculate_features(global_features[i], local_features[i], segments[i], segment_normals[i], full_segments[i]);
            if (temp == segment_id) {
                boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                viewer->setBackgroundColor (0, 0, 0);
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(full_segments[i]);
                viewer->addPointCloud<pcl::PointXYZRGB> (full_segments[i], rgb, "sample cloud");
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
                viewer->addCoordinateSystem (1.0);
                viewer->initCameraParameters ();
                while (!viewer->wasStopped ())
                {
                    viewer->spinOnce(100);
                }
            }
            if (global_features[i](0) < th1) {
                std::cout << "Too thin: " << global_features[i](0) << std::endl;
                ++temp;
                continue;
            }
            else if (global_features[i](1) < th2) {
                std::cout << "Too flat: " << global_features[i](1) << std::endl;
                ++temp;
                continue;
            }
            else if (global_features[i](2) < 800) { // should do this on the downsampled segments instead
                std::cout << "Too few points: " << global_features[i](2) << std::endl;
                ++temp;
                continue;
            }

            std::cout << "Showing segment number " << temp << std::endl;
            ++temp;
        }

        for (HistCloudT::Ptr& c : local_features) {
            size_t last_size = all_local_features->size();
            *all_local_features += *c;
            feature_inds.resize(all_local_features->size());
            for (size_t j = last_size; j < all_local_features->size(); ++j) {
                feature_inds[j] = counter;
            }
            ++counter;
        }
    }

    k_means_tree<pcl::Histogram<33>, 8> kmt;
    kmt.set_input_cloud(all_local_features);
    kmt.add_points_from_input_cloud();

    std::cout << __FILE__ << ", " << __LINE__ << std::endl;

    // OK, now we have some kind of representation, let's query!!!

    std::cout << __FILE__ << ", " << __LINE__ << std::endl;
    std::cout << "feature_inds: " << feature_inds.size() << std::endl;

    std::vector<int> segment_inds;
    // first, find all features belonging to this segment ID?
    for (std::vector<int>::iterator i = feature_inds.begin(); i != feature_inds.end(); i = std::find(i, feature_inds.end(), segment_id)) {
        if (*i != segment_id) {
            continue;
        }
        std::cout << "*i: " << *i << std::endl;
        segment_inds.push_back(std::distance(feature_inds.begin(), i));
        ++i;
    }

    std::cout << __FILE__ << ", " << __LINE__ << std::endl;

    using tree_node = k_means_tree<PointT, 8>::node;
    using tree_leaf = k_means_tree<PointT, 8>::leaf;
    using map_pair = std::map<int, int>::value_type;

    std::map<int, int> index_counts;
    // Now, get the path for all of these features
    for (int ind : segment_inds) {
        PointT hist = all_local_features->at(ind);
        std::vector<tree_node*> path;
        std::cout << __FILE__ << ", " << __LINE__ << std::endl;
        kmt.get_path_for_point(path, hist);
        tree_leaf* leaf = static_cast<tree_leaf*>(path.back());
        std::cout << __FILE__ << ", " << __LINE__ << std::endl;
        for (int i : leaf->inds) {
            int segment_ind = feature_inds[i];
            if (index_counts.count(segment_ind) == 1) {
                std::cout << __FILE__ << ", " << __LINE__ << std::endl;
                index_counts.at(segment_ind) += 1;
            }
            else {
                std::cout << __FILE__ << ", " << __LINE__ << std::endl;
                index_counts.insert(map_pair(segment_ind, 1));
            }
        }

        // count the index that has the most siblings in leaf nodes?
    }

    std::cout << __FILE__ << ", " << __LINE__ << std::endl;

    auto iter = std::max_element(index_counts.begin(), index_counts.end(), [](const map_pair& v1, const map_pair& v2) {
        return v1.second < v2.second;
    });

    std::cout << __FILE__ << ", " << __LINE__ << std::endl;

    int max_ind = iter->first;
    Cloud_t::Ptr max_segment = all_segments[max_ind];

    std::cout << __FILE__ << ", " << __LINE__ << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(max_segment);
    viewer->addPointCloud<pcl::PointXYZRGB> (max_segment, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
    }

}
