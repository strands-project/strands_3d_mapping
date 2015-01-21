#include "object_3d_retrieval/segment_features.h"
#include "sift/sift.h"

#include <pcl/common/centroid.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>

segment_features::segment_features(const Eigen::Matrix3f& K, bool visualize_features) : K(K), visualize_features(visualize_features)
{
}

Eigen::Map<Eigen::Matrix<float, 33, 1> > fpfheig(pcl::FPFHSignature33& v)
{
    return Eigen::Map<Eigen::Matrix<float, 33, 1> >(v.histogram);
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

    //Eigen::Matrix3f K;
    /*K << 1.0607072507083330e3, 0.0, 9.5635447181548398e2,
            0.0, 1.0586083263054650e3, 5.1897844298824486e2,
            0.0, 0.0, 1.0;
    size_t width = 1920, height = 1080;*/
    /*K << 3.6753450559472907e2, 0.0, 2.4449142359870606e2,
            0.0, 3.6659938826108794e2, 2.0783007950245891e2,
            0.0, 0.0, 1.0;
    size_t width = 512, height = 424;*/

    /*K << 0.5*1.0607072507083330e3, 0.0, 0.5*9.5635447181548398e2,
            0.0, 0.5*1.0586083263054650e3, 0.5*5.1897844298824486e2,
            0.0, 0.0, 1.0;*/

    std::vector<int> xs, ys;
    std::vector<cv::Vec3b> colors;
    xs.reserve(segment->size());
    ys.reserve(segment->size());
    colors.reserve(segment->size());

    for (PointT& p : full_segment->points) {
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

    // SIFT 3D KEYPOINT EXTRACTION

    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
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

    std::cout << "Number of keypoints: " << keypoints->size() << std::endl;
    std::cout << "Number of indices: " << point_idx_data->indices.size() << std::endl;

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

    if (visualize_features) {
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
    }
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
