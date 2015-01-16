#include "types.h"
#include "io_utils_kinect.h"
#include "utils.h"
#include "convex_voxel_segmentation.h"
#include "segment_features.h"
//#include "k_means_tree/k_means_tree.h"
#include "vocabulary_tree/vocabulary_tree.h"

//#define PCL_NO_PRECOMPILE
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <algorithm>

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

    using HistT = pcl::Histogram<131>;
    using HistCloudT = pcl::PointCloud<HistT>;
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;

    Eigen::Matrix3f K;
    K << 0.5*1.0607072507083330e3, 0.0, 0.5*9.5635447181548398e2,
                0.0, 0.5*1.0586083263054650e3, 0.5*5.1897844298824486e2,
                0.0, 0.0, 1.0;

    std::vector<int> feature_inds;
    HistCloudT::Ptr all_local_features(new HistCloudT);
    std::vector<CloudT::Ptr> all_segments;
    size_t counter = 0;
    size_t temp = 0;
    size_t segment_id = 90;
    for (int sceneid = 1000; sceneid <= 1009; ++sceneid) {
        int frameid = sceneid - 1000;
        string pcdfile = getScenePath(sceneid);
        if(!file_exists(pcdfile)) {
            cout << "EEEeee, file " << pcdfile << " does not exist. Quitting." << endl;
            return(0);
        }

        CloudT::Ptr cloud(new CloudT);
        cout << "Loading file " << pcdfile << endl;
        pcl::io::loadPCDFile(pcdfile, *cloud);
        transform_back(frameid, cloud);

        std::vector<CloudT::Ptr> segments;
        std::vector<NormalCloud_t::Ptr> segment_normals;
        std::vector<CloudT::Ptr> full_segments;
        convex_voxel_segmentation cvs(true, 0.008);
        cvs.segment_objects(segments, segment_normals, full_segments, cloud);

        std::vector<HistCloudT::Ptr> local_features(full_segments.size());
        std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > global_features(full_segments.size());
        for (size_t i = 0; i < full_segments.size(); ++i) {
            all_segments.push_back(full_segments[i]);
            // for each segment, create features
            segment_features sf(K, false);
            float th1 = 0.1;
            float th2 = 0.005;
            local_features[i] = HistCloudT::Ptr(new HistCloudT);
            // let's save all features to begin with
            sf.calculate_features(global_features[i], local_features[i], segments[i], segment_normals[i], full_segments[i]);
            for (HistT& h : local_features[i]->points) {
                for (float& v : h.histogram) {
                    std::cout << v << ", ";
                }
                std::cout << std::endl;
            }
            if (temp == segment_id) {
                boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                viewer->setBackgroundColor (0, 0, 0);
                pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(full_segments[i]);
                viewer->addPointCloud<PointT> (full_segments[i], rgb, "sample cloud");
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

    vocabulary_tree<HistT, 8> kmt;
    kmt.set_input_cloud(all_local_features, feature_inds);
    kmt.add_points_from_input_cloud();

    HistCloudT::Ptr query_cloud(new HistCloudT);
    // first, find all features belonging to this segment ID?
    for (std::vector<int>::iterator i = feature_inds.begin(); i != feature_inds.end(); i = std::find(i, feature_inds.end(), segment_id)) {
        if (*i != segment_id) {
            continue;
        }
        std::cout << "*i: " << *i << std::endl;
        size_t ind = std::distance(feature_inds.begin(), i);
        query_cloud->push_back(all_local_features->at(ind));
        ++i;
    }

    using index_score = vocabulary_tree<HistT, 8>::cloud_idx_score;
    std::vector<index_score> max_ind_scores;
    kmt.top_similarities(max_ind_scores, query_cloud);

    auto comp = [](const index_score& v1, const index_score& v2)
    {
        return v1.first < v2.first;
    };

    std::cout << "Query cloud size: " << query_cloud->size() << std::endl;
    std::cout << "Number of images: " << all_segments.size() << std::endl;
    std::cout << "Database cloud size: " << all_local_features->size() << std::endl;
    std::cout << "Source cloud size: " << feature_inds.size() << std::endl;
    std::cout << "Min index: " << std::min_element(max_ind_scores.begin(), max_ind_scores.end(), comp)->first << std::endl;
    std::cout << "Max index: " << std::max_element(max_ind_scores.begin(), max_ind_scores.end(), comp)->first << std::endl;

    for (size_t i = 0; i < max_ind_scores.size(); ++i) {
        CloudT::Ptr max_segment = all_segments[max_ind_scores[i].first];

        std::cout << "Distance of result: " << max_ind_scores[i].second << std::endl;

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(max_segment);
        viewer->addPointCloud<PointT> (max_segment, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce(100);
        }
    }

}
