#include "density_learning/surfel_features.h"

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "iss/iss_conf.h"
#include "iss/iss_conf.hpp"

using namespace std;

namespace surfel_features {

void visualize(CloudT::Ptr& cloud)
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

void compute_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud,
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
        visualize(vis_cloud);
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

void compute_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud,
                      NormalCloudT::Ptr& normals, float threshold, bool visualize_features)
{
    HistCloudT::Ptr temp_features(new HistCloudT);
    CloudT::Ptr temp_keypoints(new CloudT);
    compute_features(temp_features, temp_keypoints, cloud, normals, visualize_features);
    for (int i = 0; i < temp_features->size(); ++i) {
        if (temp_keypoints->at(i).rgb < threshold) {
            features->push_back(temp_features->at(i));
            keypoints->push_back(temp_keypoints->at(i));
        }
    }
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

} // namespace surfel_features
