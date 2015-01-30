#include "object_3d_retrieval/segment_features.h"
#include "sift/sift.h"

#include <pcl/common/centroid.h>
//#include <pcl/common/common.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>

void segment_features::visualize_keypoints(PointCloudT::Ptr& cloud, PointCloudT::Ptr& keypoints) const
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

segment_features::segment_features(const Eigen::Matrix3f& K, bool visualize_features) : K(K), visualize_features(visualize_features)
{
}

/*Eigen::Map<Eigen::Matrix<float, 33, 1> > fpfheig(pcl::FPFHSignature33& v)
{
    return Eigen::Map<Eigen::Matrix<float, 33, 1> >(v.histogram);
}*/


// TODO: compare using just the lowres segments and the hd segments, lowres should be faster
// and already have the normals precomputed
void segment_features::compute_shot_features(HistCloudT::Ptr& local_features, PointCloudT::Ptr& segment) const
{
    // first, extract normals, if we don't use the lowres cloud
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setInputCloud(segment);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);
    NormalCloudT::Ptr normals(new NormalCloudT);
    ne.setRadiusSearch(0.01);
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
    PointCloudT::Ptr keypoints(new PointCloudT);
    pcl::PointCloud<int>::Ptr keypoints_ind(new pcl::PointCloud<int>);
    pcl::IndicesPtr indices(new std::vector<int>);

    if (segment->size() < 0.1*480*640) {
        // Fill in the model cloud
        double model_resolution = std::min(0.006, 0.003 + 0.003*float(segment->size())/(0.1*480*640));//0.01;

        // Compute model_resolution

        iss_salient_radius_ = 6 * model_resolution;
        iss_non_max_radius_ = 4 * model_resolution;
        iss_normal_radius_ = 4 * model_resolution;
        iss_border_radius_ = 1 * model_resolution;

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
        iss_detector.setInputCloud(segment);
        iss_detector.compute(*keypoints);

        /*PointCloudT::Ptr keypoints_reduced(new PointCloudT);
        int skip = std::max(int(keypoints->size()/100), 1);
        for (int i = 0; i < keypoints->size(); i += skip) {
            keypoints_reduced->push_back(keypoints->at(i));
        }*/

        pcl::KdTreeFLANN<PointT> kdtree; // might be possible to just use the other tree here
        kdtree.setInputCloud(segment);
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
        us_detector.setInputCloud(segment);
        us_detector.compute(*keypoints_ind); // this might actually be the indices directly

        for (int ind : keypoints_ind->points) {
            keypoints->push_back(segment->at(ind));
            indices->push_back(ind);
        }
    }

    //visualize_keypoints(segment, keypoints);
    // ISS3D

    // SHOTCOLOR

    pcl::PointCloud<pcl::SHOT1344>::Ptr shots(new pcl::PointCloud<pcl::SHOT1344>());

    pcl::SHOTColorEstimationOMP<PointT, NormalT> se;
    se.setSearchMethod(tree);
    se.setIndices(indices); //keypoints
    se.setInputCloud(segment);
    se.setInputNormals(normals);
    se.setRadiusSearch(0.04); //support 0.06 orig, 0.04 still seems too big, takes time
    se.compute(*shots); //descriptors

    // SHOTCOLOR

    local_features->resize(shots->size());
    for (size_t i = 0; i < shots->size(); ++i) {
        // define size in header as N
        std::copy(shots->at(i).descriptor, shots->at(i).descriptor+N, local_features->at(i).histogram);
        //std::cout << keypoints->at(i) << std::endl;
        //std::cout << shots->at(i) << std::endl;
    }

    std::cout << "Number of features: " << shots->size() << std::endl;
}

void segment_features::compute_sift_features(HistCloudT::Ptr& local_features, PointCloudT::Ptr& segment) const
{
    std::vector<int> xs, ys;
    std::vector<cv::Vec3b> colors;
    xs.reserve(segment->size());
    ys.reserve(segment->size());
    colors.reserve(segment->size());

    for (PointT& p : segment->points) {
        Eigen::Vector3f q = K*p.getVector3fMap();
        int x = int(q(0)/q(2) + 0.5f);
        int y = int(q(1)/q(2) + 0.5f);
        if (x >= 0 && y >= 0) {
            xs.push_back(x);
            ys.push_back(y);
            cv::Vec3b temp;
            temp[0] = p.b;
            temp[1] = p.g;
            temp[2] = p.r;
            colors.push_back(temp);
        }
    }

    std::vector<int>::iterator minx = std::min_element(xs.begin(), xs.end());
    std::vector<int>::iterator maxx = std::max_element(xs.begin(), xs.end());
    std::vector<int>::iterator miny = std::min_element(ys.begin(), ys.end());
    std::vector<int>::iterator maxy = std::max_element(ys.begin(), ys.end());

    int height = *maxy - *miny + 1;
    int width = *maxx - *minx + 1;
    cv::Mat mat = cv::Mat::zeros(height, width, CV_8UC3);

    for (size_t i = 0; i < xs.size(); ++i) {
        int y = ys[i] - *miny;
        int x = xs[i] - *minx;
        mat.at<cv::Vec3b>(y, x) = colors[i];
    }

    // SIFT FEATURE DESCRIPTORS

    // initialize the keypoints with the above data
    std::vector<cv::KeyPoint> keypoints_1;
    cv::SIFT::DetectorParams detector_params;
    detector_params.edgeThreshold = 15.0; // 10.0 default
    detector_params.threshold = 0.04; // 0.04 default
    cv::SiftFeatureDetector detector(detector_params);
    //cv::StarFeatureDetector detector;
    //cv::MserFeatureDetector detector;
    detector.detect(mat, keypoints_1);

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::Mat descriptors;
    // the length of the descriptors is 128
    // the 3D sift keypoint detectors are probably very unreliable
    cv::SIFT::DescriptorParams descriptor_params;
    descriptor_params.isNormalize = true; // always true, shouldn't matter
    descriptor_params.magnification = 3.0; // 3.0 default
    descriptor_params.recalculateAngles = true; // true default

    cv::SiftDescriptorExtractor extractor;
    //cv::BriefDescriptorExtractor extractor;
    //sift.compute( img_1, keypoints_1, descriptors_1 );
    extractor.compute(mat, keypoints_1, descriptors);

    local_features->resize(descriptors.rows);
    for (size_t j = 0; j < descriptors.rows; ++j) {
        cv::Point2f kp = keypoints_1[j].pt;
        int x = kp.x;
        int y = kp.y;
        cv::Vec3f cumc;
        float pixels = 0;
        for (int i = x - 10; i <= x + 10; ++i) {
            if (i < 0 || i >= mat.cols) {
                continue;
            }
            for (int j = y - 10; j <= y + 10; ++j) {
                if( j < 0 || j >= mat.rows) {
                    continue;
                }
                cumc += mat.at<cv::Vec3b>(j, i);
                ++pixels;
            }
        }
        cumc /= pixels;
        for (size_t k = 0; k < 128; ++k) {
            local_features->at(j).histogram[k] = descriptors.at<float>(j, k);
        }
        for (size_t k = 128; k < 131; ++k) {
            local_features->at(j).histogram[k] = color_weight*cumc[k-128];
        }
    }

    // SIFT FEATURE DESCRIPTORS

    /*if (visualize_features) {
        for (PointT& p : keypoints->points) {
            Eigen::Vector3f q = K*p.getVector3fMap();
            int x = int(q(0)/q(2)) - *minx;
            int y = int(q(1)/q(2)) - *miny;
            mat.at<cv::Vec3b>(y, x)[0] = 0;
            mat.at<cv::Vec3b>(y, x)[1] = 0;
            mat.at<cv::Vec3b>(y, x)[2] = 255;
        }

        cv::imshow("Matrix yo!", mat);
        cv::waitKey(0);
    }*/
}

void segment_features::calculate_features(Eigen::VectorXf& global_features, HistCloudT::Ptr &local_features, PointCloudT::Ptr& segment,
                                          NormalCloudT::Ptr& segment_normals, PointCloudT::Ptr& full_segment) const
{
    Eigen::Matrix<float, 4, 1> centroid;
    Eigen::Matrix3f covariance_matrix;
    //pcl::compute3DCentroid(*segment, centroid);
    //pcl::computeCovarianceMatrixNormalized(*segment, centroid, covariance_matrix);
    pcl::computeMeanAndCovarianceMatrix(*segment, covariance_matrix, centroid);
    std::cout << covariance_matrix << std::endl;
    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
    //std::cout << es.eigenvectors() << std::endl;
    //std::cout << es.eigenvalues() << std::endl;

    Eigen::JacobiSVD<Eigen::Matrix3f> svd;
    svd.compute(covariance_matrix);//, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //Eigen::Vector3f T = svd.singularValues().array().abs();//(svd.matrixU()*svd.singularValues()).array().abs();
    Eigen::Vector3f T = es.eigenvalues().array().abs();
    std::sort(T.data(), T.data() + T.size(), [](float f1, float f2) { return f1 > f2; });
    std::cout << T << std::endl;
    /*std::cout << svd.matrixU() << std::endl;
    std::cout << svd.singularValues() << std::endl;*/

    global_features.resize(3);
    global_features(0) = fabs(T(1) / T(0));
    global_features(1) = fabs(T(2) / T(0));
    global_features(2) = segment->size();

    //compute_sift_features(local_features, full_segment);
    compute_shot_features(local_features, full_segment);
}

// FPFH FEATURE DESCRIPTORS

// Create the FPFH estimation class, and pass the input dataset+normals to it
/*pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
fpfh.setInputCloud(segment);
fpfh.setInputNormals(segment_normals);
fpfh.setIndices(point_idx_data);
// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

// Create an empty kdtree representation, and pass it to the FPFH estimation object.
// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
fpfh.setSearchMethod (tree);
// Output datasets
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
// Use all neighbors in a sphere of radius 5cm
// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
fpfh.setRadiusSearch (0.03);
// Compute the features
fpfh.compute(*fpfhs);

local_features->resize(fpfhs->size());
size_t counter = 0;
for (pcl::FPFHSignature33 f : fpfhs->points) {
    histeig(local_features->at(counter)) = fpfheig(f);
    ++counter;
}*/

// FPFH FEATURE DESCRIPTORS

// SIFT 3D KEYPOINT EXTRACTION

/*pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
//pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>);
pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT>);
sift_detect.setSearchMethod(kdtree);
const float min_scale = 0.025; //0.0005
const int nr_octaves = 4; //4
const int nr_scales_per_octave = 5; //5
const float min_contrast = 1; //1
const int k_sift = 10;
sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
sift_detect.setMinimumContrast(min_contrast);
sift_detect.setKSearch(k_sift);
sift_detect.setInputCloud(segment);
pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
sift_detect.compute(keypoints_temp);
PointCloudT::Ptr keypoints(new PointCloudT);
pcl::copyPointCloud (keypoints_temp, *keypoints);

pcl::PointIndices::Ptr point_idx_data(new pcl::PointIndices());
point_idx_data->indices.reserve(segment->points.size());
float th = 1e-5;
std::cout << "Segment size: " << segment->size() << std::endl;
for (PointT& p : keypoints->points) {
    const Eigen::Vector3f pp = p.getVector3fMap();
    auto it = std::min_element(segment->points.begin(), segment->points.end(), [&pp] (const PointT& q1, const PointT& q2) {
        return (q1.getVector3fMap() - pp).norm() < (q2.getVector3fMap() - pp).norm();
    });
    if (it != segment->points.end()) {
        point_idx_data->indices.push_back(std::distance(segment->points.begin(), it));
        std::cout << point_idx_data->indices.back() << std::endl;
    }
}

std::cout << "Number of keypoints: " << keypoints->size() << std::endl;
std::cout << "Number of indices: " << point_idx_data->indices.size() << std::endl;*/

/*std::vector<cv::KeyPoint> keypoints_1(keypoints->size());
size_t counter = 0;
for (PointT& p : keypoints->points) {
    Eigen::Vector3f q = K*p.getVector3fMap();
    int x = int(q(0)/q(2)) - *minx;
    int y = int(q(1)/q(2)) - *miny;
    keypoints_1[counter].pt = cv::Point(x, y);
    ++counter;
}*/

// SIFT 3D KEYPOINT EXTRACTION
