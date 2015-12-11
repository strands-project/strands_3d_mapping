#include "object_3d_retrieval/pfhrgb_estimation.h"

#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pfhrgb_estimation {

using namespace std;

void visualize_keypoints(CloudT::Ptr& cloud, CloudT::Ptr& keypoints)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    for (PointT& p : keypoints->points) {
        p.r = 255;
        p.g = 0;
        p.b = 0;
    }

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbk(keypoints);
    viewer->addPointCloud<PointT>(keypoints, rgbk, "keypoint cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoint cloud");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void compute_query_features(PfhRgbCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud, bool visualize_features)
{
    // first, extract normals, if we don't use the lowres cloud
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);
    NormalCloudT::Ptr normals(new NormalCloudT);
    ne.setRadiusSearch(0.04); // 0.02
    ne.compute(*normals);

    //float threshold = std::max(1.0-0.5*float(segment->size())/(0.3*480*640), 0.5);
    //
    //  ISS3D parameters
    //
    double iss_salient_radius_;
    double iss_non_max_radius_;
    double iss_normal_radius_;
    double iss_border_radius_;
    double iss_gamma_21_ (0.975); // 0.975 orig
    double iss_gamma_32_ (0.975); // 0.975 orig
    double iss_min_neighbors_ (5);
    int iss_threads_ (4);

    //CloudT::Ptr model_keypoints(new CloudT);
    pcl::IndicesPtr indices(new std::vector<int>);

    { // TODO: this used to be 0.1 !!!!!!!!!!!!!!!
        // Fill in the model cloud
        double model_resolution = std::min(0.006, 0.003 + 0.003*float(cloud->size())/(0.1*480*640));

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
        iss_detector.compute(*keypoints);

        pcl::KdTreeFLANN<PointT> kdtree; // might be possible to just use the other tree here
        kdtree.setInputCloud(cloud);
        for (const PointT& k : keypoints->points) {
            std::vector<int> ind;
            std::vector<float> dist;
            kdtree.nearestKSearchT(k, 1, ind, dist);
            indices->push_back(ind[0]);
        }
    }

    if (visualize_features) {
        visualize_keypoints(cloud, keypoints);
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

void compute_surfel_features(PfhRgbCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud, bool visualize_features)
{
    // first, extract normals, if we don't use the lowres cloud
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);
    NormalCloudT::Ptr normals(new NormalCloudT);
    ne.setRadiusSearch(0.04); // 0.02
    ne.compute(*normals);

    //float threshold = std::max(1.0-0.5*float(segment->size())/(0.3*480*640), 0.5);
    //
    //  ISS3D parameters
    //
    double iss_salient_radius_;
    double iss_non_max_radius_;
    double iss_normal_radius_;
    double iss_border_radius_;
    double iss_gamma_21_ (0.975); // 0.975 orig
    double iss_gamma_32_ (0.975); // 0.975 orig
    double iss_min_neighbors_ (5);
    int iss_threads_ (4);

    //CloudT::Ptr model_keypoints(new CloudT);
    pcl::PointCloud<int>::Ptr keypoints_ind(new pcl::PointCloud<int>);
    pcl::IndicesPtr indices(new std::vector<int>);

    if (cloud->size() < 0.015*480*640) { // TODO: this used to be 0.1 !!!!!!!!!!!!!!!
        // Fill in the model cloud
        double model_resolution = std::min(0.006, 0.003 + 0.003*float(cloud->size())/(0.015*480*640));

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
        iss_detector.compute(*keypoints);

        pcl::KdTreeFLANN<PointT> kdtree; // might be possible to just use the other tree here
        kdtree.setInputCloud(cloud);
        for (const PointT& k : keypoints->points) {
            std::vector<int> ind;
            std::vector<float> dist;
            kdtree.nearestKSearchT(k, 1, ind, dist);
            indices->push_back(ind[0]);
        }
    }
    else {
        pcl::UniformSampling<PointT> us_detector;
        us_detector.setRadiusSearch(0.1);
        us_detector.setSearchMethod(tree);
        us_detector.setInputCloud(cloud);
        us_detector.compute(*keypoints_ind); // this might actually be the indices directly

        for (int ind : keypoints_ind->points) {
            keypoints->push_back(cloud->at(ind));
            indices->push_back(ind);
        }
    }

    if (visualize_features) {
        visualize_keypoints(cloud, keypoints);
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

void compute_features(PfhRgbCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud, bool visualize_features)
{
    // first, extract normals, if we don't use the lowres cloud
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);
    NormalCloudT::Ptr normals(new NormalCloudT);
    ne.setRadiusSearch(0.02); // 0.02
    ne.compute(*normals);

    //float threshold = std::max(1.0-0.5*float(segment->size())/(0.3*480*640), 0.5);
    //
    //  ISS3D parameters
    //
    double iss_salient_radius_;
    double iss_non_max_radius_;
    double iss_normal_radius_;
    double iss_border_radius_;
    double iss_gamma_21_ (0.975); // 0.975 orig
    double iss_gamma_32_ (0.975); // 0.975 orig
    double iss_min_neighbors_ (5);
    int iss_threads_ (4);

    //CloudT::Ptr model_keypoints(new CloudT);
    pcl::PointCloud<int>::Ptr keypoints_ind(new pcl::PointCloud<int>);
    pcl::IndicesPtr indices(new std::vector<int>);

    if (cloud->size() < 0.3*480*640) { // TODO: this used to be 0.1 !!!!!!!!!!!!!!!
        // Fill in the model cloud
        double model_resolution = std::min(0.006, 0.003 + 0.003*float(cloud->size())/(0.1*480*640));

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
        iss_detector.compute(*keypoints);

        pcl::KdTreeFLANN<PointT> kdtree; // might be possible to just use the other tree here
        kdtree.setInputCloud(cloud);
        for (const PointT& k : keypoints->points) {
            std::vector<int> ind;
            std::vector<float> dist;
            kdtree.nearestKSearchT(k, 1, ind, dist);
            indices->push_back(ind[0]);
        }
    }
    else {
        pcl::UniformSampling<PointT> us_detector;
        us_detector.setRadiusSearch(0.1);
        us_detector.setSearchMethod(tree);
        us_detector.setInputCloud(cloud);
        us_detector.compute(*keypoints_ind); // this might actually be the indices directly

        for (int ind : keypoints_ind->points) {
            keypoints->push_back(cloud->at(ind));
            indices->push_back(ind);
        }
    }

    if (visualize_features) {
        visualize_keypoints(cloud, keypoints);
    }
    // ISS3D

    // PFHRGB
    pcl::PFHRGBEstimation<PointT, NormalT> se;
    se.setSearchMethod(tree);
    //se.setKSearch(100);
    se.setIndices(indices); //keypoints
    se.setInputCloud(cloud);
    se.setInputNormals(normals);
    se.setRadiusSearch(0.02); //support 0.06 orig, 0.04 still seems too big, takes time

    pcl::PointCloud<pcl::PFHRGBSignature250> pfhrgb_cloud;
    se.compute(pfhrgb_cloud); //descriptors

    const int N = 250;
    features->resize(pfhrgb_cloud.size());
    for (size_t i = 0; i < pfhrgb_cloud.size(); ++i) {
        std::copy(pfhrgb_cloud.at(i).histogram, pfhrgb_cloud.at(i).histogram+N, features->at(i).histogram);
    }

    std::cout << "Number of features: " << pfhrgb_cloud.size() << std::endl;
}

void visualize_split_keypoints(vector<CloudT::Ptr>& split_keypoints)
{
    int colormap[][3] = {
        {166,206,227},
        {31,120,180},
        {178,223,138},
        {51,160,44},
        {251,154,153},
        {227,26,28},
        {253,191,111},
        {255,127,0},
        {202,178,214},
        {106,61,154},
        {255,255,153},
        {177,89,40},
        {141,211,199},
        {255,255,179},
        {190,186,218},
        {251,128,114},
        {128,177,211},
        {253,180,98},
        {179,222,105},
        {252,205,229},
        {217,217,217},
        {188,128,189},
        {204,235,197},
        {255,237,111}
    };

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    int counter = 0;
    for (CloudT::Ptr& keypoints : split_keypoints) {
        CloudT::Ptr colored_keypoints(new CloudT);
        *colored_keypoints += *keypoints;
        for (PointT& p : colored_keypoints->points) {
            p.r = colormap[counter%24][2];
            p.g = colormap[counter%24][1];
            p.b = colormap[counter%24][0];
        }
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbk(colored_keypoints);
        string cloudname = "keypoint_cloud" + to_string(counter);
        viewer->addPointCloud<PointT>(colored_keypoints, rgbk, cloudname);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudname);
        ++counter;
    }

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void split_descriptor_points(vector<PfhRgbCloudT::Ptr>& split_features, vector<CloudT::Ptr>& split_keypoints,
                             PfhRgbCloudT::Ptr& features, CloudT::Ptr& keypoints, int expected_cluster_size, bool verbose)
{
    // first, decide how many clusters we want
    const int nbr_original_features = features->size();

    int expected_nbr_clusters = int(std::round(float(nbr_original_features) / float(expected_cluster_size)));

    if (expected_nbr_clusters == 1 || float(expected_cluster_size) > 1.7f*float(nbr_original_features)) {
        split_features.push_back(PfhRgbCloudT::Ptr(new PfhRgbCloudT));
        *split_features.back() += *features;
        split_keypoints.push_back(CloudT::Ptr(new CloudT));
        *split_keypoints.back() += *keypoints;
        return;
    }

    expected_cluster_size = int(float(keypoints->size()) / float(expected_nbr_clusters));

    Eigen::Matrix<float, 3, Eigen::Dynamic> centroids;
    centroids.resize(3, expected_nbr_clusters);
    vector<int> closest_centroids(keypoints->size());
    Eigen::Matrix<float, 1, Eigen::Dynamic> distances;
    distances.resize(1, expected_nbr_clusters);
    Eigen::Matrix<float, 3, Eigen::Dynamic> last_centroids;
    Eigen::VectorXi closest_counts;
    closest_counts.resize(expected_nbr_clusters);
    last_centroids.resize(3, expected_nbr_clusters);
    last_centroids.setZero();
    size_t counter;
    size_t iterations = 0;
    size_t max_iter = nbr_original_features*1000;
    int closest;
    while (iterations < max_iter && !centroids.isApprox(last_centroids, 1e-30f)) {
        counter = 0;
        for (const PointT& p : keypoints->points) {
            Eigen::Vector3f ep = p.getVector3fMap();
            distances = (centroids.colwise()-ep).colwise().norm();
            distances.minCoeff(&closest);
            closest_centroids[counter] = closest;
            ++counter;
        }
        closest_counts.setZero();
        last_centroids = centroids;
        centroids.setZero();
        counter = 0;
        for (int i : closest_centroids) {
            if (!pcl::isFinite(keypoints->at(counter))) {
                continue;
            }
            centroids.col(i) += keypoints->at(counter).getVector3fMap();
            closest_counts(i) += 1;
            ++counter;
        }
        for (size_t i = 0; i < expected_nbr_clusters; ++i) {
            if (closest_counts(i) == 0) {
                centroids.col(i) = keypoints->at(rand() % nbr_original_features).getVector3fMap();
            }
            else {
                centroids.col(i) *= 1.0f/float(closest_counts(i));
            }
        }
        ++iterations;
    }

    vector<int> groups(expected_nbr_clusters);
    Eigen::VectorXi group_size;
    group_size.resize(expected_nbr_clusters);
    group_size = closest_counts;
    for (int i = 0; i < expected_nbr_clusters; ++i) {
        groups[i] = i;
    }


    // after this initialization step, enforce group size
    // greedily move points in large groups to smaller groups
    // to the iteration until equalized, move one point at a time
    max_iter = 1000;
    int iter = 0;
    while (iter < max_iter) {

        for (size_t i = 0; i < expected_nbr_clusters; ++i) {
            bool large = closest_counts(i) >= expected_cluster_size;
            /*if (closest_counts(i) > expected_cluster_size) {
                continue;
            }*/
            // find the point closest to another cluster centre
            // if that cluster center is smaller, move it
            distances = (centroids.colwise()-centroids.col(i)).colwise().norm();

            for (size_t j = 0; j < expected_nbr_clusters; ++j) {
                bool should_move = closest_counts(j) <= closest_counts(i);
                if (large) {
                    should_move = !should_move;
                }
                if (should_move) {
                    distances(j) = 1000.0f;
                }
            }

            distances(i) = 1000.0f;
            distances.minCoeff(&closest);
            if (distances(closest) == 1000) {
                continue;
            }
            bool should_find_closest = closest_counts(closest) > closest_counts(i);
            if (large) {
                should_find_closest = !should_find_closest;
            }
            if (should_find_closest) {
                // find closest point in mincluster, assign it i
                counter = 0;
                Eigen::Vector3f pc;
                if (large) {
                    pc = centroids.col(closest);
                }
                else {
                    pc = centroids.col(i);
                }
                float mindist = 1000.0f; // very large
                int minind;
                int target_set;
                int source_set;
                if (large) {
                    target_set = i;
                    source_set = closest;
                }
                else {
                    target_set = closest;
                    source_set = i;
                }
                for (int j : closest_centroids) {
                    if (j != target_set || !pcl::isFinite(keypoints->at(counter))) {
                        ++counter;
                        continue;
                    }
                    float distance = (keypoints->at(counter).getVector3fMap() - pc).norm();
                    if (distance < mindist) {
                        mindist = distance;
                        minind = counter;
                    }
                    ++counter;
                }
                closest_centroids[minind] = source_set;
            }
        }

        // update cluster centers
        centroids.setZero();
        closest_counts.setZero();
        counter = 0;
        for (int i : closest_centroids) {
            if (!pcl::isFinite(keypoints->at(counter))) {
                continue;
            }
            centroids.col(i) += keypoints->at(counter).getVector3fMap();
            closest_counts(i) += 1;
            ++counter;
        }
        bool all_close = true;
        for (size_t i = 0; i < expected_nbr_clusters; ++i) {
            if (verbose) {
                cout << "Cluster nbr: " << i << endl;
                cout << "Expected nbr clusters: " << expected_nbr_clusters << endl;
                cout << "Cluster difference: " << std::abs(closest_counts(i) - expected_cluster_size) << endl;
                cout << "Cluster size: " << closest_counts(i) << endl;
                cout << "Expected cluster size: " <<  expected_cluster_size << endl;
            }
            if (std::abs(closest_counts(i) - expected_cluster_size) > 3) {
                all_close = false;
            }
            if (closest_counts(i) == 0) {
                centroids.col(i) = keypoints->at(rand() % nbr_original_features).getVector3fMap();
            }
            else {
                centroids.col(i) *= 1.0f/float(closest_counts(i));
            }
        }

        if (all_close) {
            break;
        }

        ++iter;
    }

    for (size_t i = 0; i < expected_nbr_clusters; ++i) {
        split_features.push_back(PfhRgbCloudT::Ptr(new PfhRgbCloudT));
        split_keypoints.push_back(CloudT::Ptr(new CloudT));
    }

    counter = 0;
    for (int i : closest_centroids) {
        split_features[i]->push_back(features->at(counter));
        split_keypoints[i]->push_back(keypoints->at(counter));
        ++counter;
    }

    if (verbose) {
        for (PfhRgbCloudT::Ptr& desc : split_features) {
            cout << "Cluster size: " << desc->size() << endl;
        }
    }

    //visualize_split_keypoints(split_keypoints);
}

}
