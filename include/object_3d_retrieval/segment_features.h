#ifndef SEGMENT_FEATURES_H
#define SEGMENT_FEATURES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

template<int N>
Eigen::Map<Eigen::Matrix<float, N, 1> > histeig(pcl::Histogram<N>& v)
{
    return Eigen::Map<Eigen::Matrix<float, N, 1> >(v.histogram);
}

class segment_features
{
private:

    //static const int N = 131;
    static const int N = 1344;

    using PointT = pcl::PointXYZRGB;
    using PointCloudT = pcl::PointCloud<PointT>;
    //using PointNT = pcl::PointNormal ;
    //using PointNCloudT = pcl::PointCloud<PointNT>;
    using NormalT = pcl::Normal;
    using NormalCloudT = pcl::PointCloud<NormalT>;
    using HistT = pcl::Histogram<N>;
    using HistCloudT = pcl::PointCloud<HistT>;

    static constexpr float color_weight = 0.3f;

    Eigen::Matrix3f K;
    bool visualize_features;

    void compute_sift_features(HistCloudT::Ptr& local_features, PointCloudT::Ptr& segment) const;
    void compute_shot_features(HistCloudT::Ptr& local_features, PointCloudT::Ptr& segment) const;
    void visualize_keypoints(PointCloudT::Ptr& cloud, PointCloudT::Ptr& keypoints) const;

public:
    void calculate_features(Eigen::VectorXf &global_features, HistCloudT::Ptr& local_features, PointCloudT::Ptr& segment,
                            NormalCloudT::Ptr& segment_normals, PointCloudT::Ptr& full_segment) const;
    segment_features(const Eigen::Matrix3f& K, bool visualize_features = false);
};

#endif // SEGMENT_FEATURES_H
