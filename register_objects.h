#ifndef REGISTER_OBJECTS_H
#define REGISTER_OBJECTS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class register_objects
{
protected:

    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    using CloudPtrT = CloudT::Ptr;

protected:

    CloudPtrT c1;
    CloudPtrT c2;
    Eigen::Matrix3f k1;
    Eigen::Matrix3f k2;
    Eigen::Matrix4f T;

public:
    void set_input_clouds(CloudPtrT& t1, const Eigen::Matrix3f& tk1, CloudPtrT& t2, const Eigen::Matrix3f& tk2);
    void do_registration();
    void get_transformation(Eigen::Matrix4f& trans);
    float get_similarity_measure();
    std::pair<int, int> calculate_image_for_cloud(cv::Mat& image, cv::Mat& depth, CloudPtrT& cloud, const Eigen::Matrix3f &K);
    void calculate_features_for_image(cv::Mat& descriptors, std::vector<cv::KeyPoint>& keypoints, CloudPtrT& cloud, cv::Mat& image,
                                      cv::Mat& depth, int minx, int miny, const Eigen::Matrix3f& K);
    double get_match_score();
    register_objects();
};

#endif // REGISTER_OBJECTS_H
