#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "dynamic_object_retrieval/definitions.h"
#include "iss/iss_conf.h"
#include "iss/iss_conf.hpp"
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d_omp.h>
#include <dynamic_object_retrieval/visualize.h>
#include "density_learning/surfel_type.h"
#include <pcl/kdtree/impl/kdtree_flann.hpp>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

void compute_density_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud,
                              NormalCloudT::Ptr& normals, bool visualize_features)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    if (normals->empty()) {
        // first, extract normals, if we don't use the lowres cloud
        pcl::NormalEstimationOMP<PointT, NormalT> ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.04); // 0.02
        ne.compute(*normals);
    }

    //float threshold = std::max(1.0-0.5*float(segment->size())/(0.3*480*640), 0.5);
    //
    //  ISS3D parameters
    //
    double iss_salient_radius_;
    double iss_non_max_radius_;
    double iss_normal_radius_;
    double iss_border_radius_;
    double iss_gamma_21_ (1.0); // 0.975 orig
    double iss_gamma_32_ (1.0); // 0.975 orig
    double iss_min_neighbors_ (5);
    int iss_threads_ (3);


    pcl::IndicesPtr indices(new std::vector<int>);

    // Fill in the model cloud
    double model_resolution = 0.005;

    // Compute model_resolution
    iss_salient_radius_ = 6 * model_resolution;
    iss_non_max_radius_ = 4 * model_resolution;
    iss_normal_radius_ = 4 * model_resolution;
    iss_border_radius_ = 0.5 * model_resolution; // 1

    //
    // Compute keypoints
    //
    pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
    iss_detector.setSearchMethod(tree);
    iss_detector.setSalientRadius(iss_salient_radius_);
    iss_detector.setNonMaxRadius(iss_non_max_radius_);

    iss_detector.setNormalRadius(iss_normal_radius_); // comment these two if not to use border removal
    iss_detector.setBorderRadius(iss_border_radius_); // comment these two if not to use border removal

    iss_detector.setThreshold21(iss_gamma_21_);
    iss_detector.setThreshold32(iss_gamma_32_);
    iss_detector.setMinNeighbors(iss_min_neighbors_);
    iss_detector.setNumberOfThreads(iss_threads_);
    iss_detector.setInputCloud(cloud);
    //iss_detector.setNormals(normals);
    iss_detector.compute(*keypoints);

    pcl::KdTreeFLANN<PointT> kdtree; // might be possible to just use the other tree here
    kdtree.setInputCloud(cloud);
    for (const PointT& k : keypoints->points) {
        std::vector<int> ind;
        std::vector<float> dist;
        kdtree.nearestKSearchT(k, 1, ind, dist);
        //cout << "Keypoint threshold: " << k.rgb << endl;
        indices->push_back(ind[0]);
    }

    if (visualize_features) {
        CloudT::Ptr vis_cloud(new CloudT());
        *vis_cloud += *cloud;
        for (PointT p : keypoints->points) {
            p.r = 255; p.g = 0; p.b = 0;
            vis_cloud->push_back(p);
        }
        dynamic_object_retrieval::visualize(vis_cloud);
    }
    // ISS3D

    // PFHRGB
    pcl::PFHRGBEstimation<PointT, NormalT> se;
    se.setSearchMethod(tree);
    //se.setKSearch(100);
    se.setIndices(indices); //keypoints
    se.setInputCloud(cloud);
    se.setInputNormals(normals);
    se.setRadiusSearch(0.04); //support 0.06 orig, 0.04 still seems too big, takes time

    pcl::PointCloud<pcl::PFHRGBSignature250> pfhrgb_cloud;
    se.compute(pfhrgb_cloud); //descriptors

    const int N = 250;
    features->resize(pfhrgb_cloud.size());
    for (size_t i = 0; i < pfhrgb_cloud.size(); ++i) {
        std::copy(pfhrgb_cloud.at(i).histogram, pfhrgb_cloud.at(i).histogram+N, features->at(i).histogram);
    }

    std::cout << "Number of features: " << pfhrgb_cloud.size() << std::endl;
}

NormalCloudT::Ptr compute_surfel_normals(SurfelCloudT::Ptr& surfel_cloud, CloudT::Ptr& segment)
{
    pcl::KdTreeFLANN<SurfelT> kdtree;
    kdtree.setInputCloud(surfel_cloud);

    NormalCloudT::Ptr normals(new NormalCloudT);
    for (const PointT& p : segment->points) {
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
        NormalT n; n.normal_x = q.normal_x; n.normal_y = q.normal_y; n.normal_z = q.normal_z;
        normals->push_back(n);

    }

    return normals;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the sweeps..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    dynamic_object_retrieval::convex_segment_cloud_map segment_clouds(data_path);
    dynamic_object_retrieval::convex_feature_map segment_features(data_path);
    dynamic_object_retrieval::convex_keypoint_map segment_keypoints(data_path);
    dynamic_object_retrieval::convex_segment_sweep_path_map segment_sweep_paths(data_path);

    boost::filesystem::path last_sweep_path;
    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    for (auto tup : dynamic_object_retrieval::zip(segment_clouds, segment_features,
                                                  segment_keypoints, segment_sweep_paths)) {
        CloudT::Ptr segment;
        boost::filesystem::path feature_path;
        boost::filesystem::path keypoint_path;
        boost::filesystem::path sweep_path;
        tie(segment, feature_path, keypoint_path, sweep_path) = tup;

        if (sweep_path != last_sweep_path) {
            boost::filesystem::path surfels_path = sweep_path / "surfel_map.pcd";
            surfel_cloud->clear();
            pcl::io::loadPCDFile(surfels_path.string(), *surfel_cloud);
        }

        NormalCloudT::Ptr normals = compute_surfel_normals(surfel_cloud, segment);

        HistCloudT::Ptr features(new HistCloudT);
        CloudT::Ptr keypoints(new CloudT);
        compute_density_features(features, keypoints, segment, normals, false);

        if (features->empty()) {
            HistT sp;
            for (int i = 0; i < N; ++i) {
                sp.histogram[i] = std::numeric_limits<float>::infinity();
            }
            features->push_back(sp);
            PointT p;
            p.x = p.y = p.z = std::numeric_limits<float>::infinity();
            keypoints->push_back(p);
        }

        feature_path = feature_path.parent_path() / (string("density_") + feature_path.filename().string());
        keypoint_path = keypoint_path.parent_path() / (string("density_") + keypoint_path.filename().string());

        cout << "Found cloud of size: " << segment->size() << endl;
        cout << "Found feature path: " << feature_path.string() << endl;

        pcl::io::savePCDFileBinary(feature_path.string(), *features);
        pcl::io::savePCDFileBinary(keypoint_path.string(), *keypoints);
    }

    return 0;
}
