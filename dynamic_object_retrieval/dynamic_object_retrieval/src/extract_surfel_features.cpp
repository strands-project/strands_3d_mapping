#include "dynamic_object_retrieval/extract_surfel_features.h"

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/octree/octree.h>

using namespace std;

namespace dynamic_object_retrieval {

const float saliency_threshold = 0.970; // from the keypoint density analysis
const float density_threshold_volume = 0.0405; // from the keypoint density analysis

void visualize_features(CloudT::Ptr& cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

float compute_cloud_volume_features(CloudT::Ptr& cloud)
{
    float resolution = 0.05f;
    pcl::octree::OctreePointCloud<PointT> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    std::vector<PointT, Eigen::aligned_allocator<PointT> > dummy;
    float centers = octree.getOccupiedVoxelCenters(dummy);
    return centers*resolution*resolution*resolution;
}

// there is no problem here, should just take the features, keypoints, cloud, and normals
void compute_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud,
                      NormalCloudT::Ptr& normals, bool do_visualize, bool is_query)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    //  ISS3D parameters
    double iss_salient_radius_;
    double iss_non_max_radius_;
    double iss_normal_radius_;
    double iss_border_radius_;
    double iss_gamma_21_ (saliency_threshold); // 0.975 orig
    double iss_gamma_32_ (saliency_threshold); // 0.975 orig
    double iss_min_neighbors_ (5);
    int iss_threads_ (3);

    pcl::IndicesPtr indices(new std::vector<int>);

    double volume = compute_cloud_volume_features(cloud);

    if (is_query || volume < density_threshold_volume) {

        // Fill in the model cloud
        double model_resolution = 0.007; // 0.003 before

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
        iss_detector.setNormals(normals);
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
    }
    else {
        pcl::PointCloud<int>::Ptr keypoints_ind(new pcl::PointCloud<int>);
        pcl::UniformSampling<PointT> us_detector;
        us_detector.setRadiusSearch(0.08);
        us_detector.setSearchMethod(tree);
        us_detector.setInputCloud(cloud);
        us_detector.compute(*keypoints_ind);

        for (int ind : keypoints_ind->points) {
            keypoints->push_back(cloud->at(ind));
            indices->push_back(ind);
        }
    }

    if (do_visualize) {
        CloudT::Ptr vis_cloud(new CloudT());
        *vis_cloud += *cloud;
        for (PointT p : keypoints->points) {
            p.r = 255; p.g = 0; p.b = 0;
            vis_cloud->push_back(p);
        }
        visualize_features(vis_cloud);
    }
    // ISS3D

    // PFHRGB
    pcl::PFHRGBEstimation<PointT, NormalT> se;
    se.setSearchMethod(tree);
    //se.setKSearch(100);
    se.setIndices(indices); //keypoints
    se.setInputCloud(cloud);
    se.setInputNormals(normals);
    se.setRadiusSearch(0.06); //support 0.06 orig, 0.04 still seems too big, takes time

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
    normals->reserve(segment->size());
    for (const PointT& p : segment->points) {
        if (!pcl::isFinite(p)) {
            NormalT crap; crap.normal_x = 0; crap.normal_y = 0; crap.normal_z = 0;
            normals->push_back(crap);
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

void compute_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints,
                      CloudT::Ptr& cloud, SurfelCloudT::Ptr& surfel_map, bool visualize_features)
{
    NormalCloudT::Ptr normals = compute_surfel_normals(surfel_map, cloud);
    compute_features(features, keypoints, cloud, normals, visualize_features);
}

void compute_query_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints,
                            CloudT::Ptr& cloud, SurfelCloudT::Ptr& surfel_map, bool visualize_features)
{
    NormalCloudT::Ptr normals = compute_surfel_normals(surfel_map, cloud);
    compute_features(features, keypoints, cloud, normals, visualize_features, true);
}

// this is only for the querying
/*
pair<CloudT::Ptr, NormalCloudT::Ptr>
cloud_normals_from_surfel_mask(SurfelCloudT::Ptr& surfel_cloud, const cv::Mat& mask,
                               const Eigen::Matrix4f& T, const Eigen::Matrix3f& K)
{
    int height = mask.rows;
    int width = mask.cols;

    //Eigen::Matrix4f inv = mask_transform.inverse();

    CloudT::Ptr cloud(new CloudT);
    NormalCloudT::Ptr normals(new NormalCloudT);
    for (const SurfelT& s : surfel_cloud->points) {
        //Eigen::Vector4f q = T*s.getVector4fMap();
        Eigen::Vector4f q = T*Eigen::Vector4f(s.x, s.y, s.z, 1.0f);
        Eigen::Vector3f r = K*q.head<3>();
        int x = int(r(0)/r(2));
        int y = int(r(1)/r(2));
        if (x >= width || x < 0 || y >= height || y < 0) {
            continue;
        }
        if (mask.at<uint8_t>(y, x) != 0) {
            PointT p; p.x = s.x; p.y = s.y; p.z = s.z; p.rgba = s.rgba;
            NormalT n; n.normal_x = s.normal_x; n.normal_y = s.normal_y; n.normal_z = s.normal_z;
            cloud->push_back(p);
            normals->push_back(n);
        }
    }

    return make_pair(cloud, normals);
}

SurfelCloudT::Ptr load_surfel_cloud_for_sweep(const string& xml)
{
    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    boost::filesystem::path surfels_path = boost::filesystem::path(xml).parent_path() / "surfel_map.pcd";
    pcl::io::loadPCDFile(surfels_path.string(), *surfel_cloud);
    return surfel_cloud;
}
*/

} // namespace dynamic_object_retrieval
