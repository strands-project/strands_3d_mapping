#ifndef SEGMENT_FEATURES_H
#define SEGMENT_FEATURES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

template<typename Point>
Eigen::Map<Eigen::Matrix<float, 33, 1> > eigmap(Point& v)
{
    return Eigen::Map<Eigen::Matrix<float, 33, 1> >(v.histogram);
}

class segment_features
{
private:
    using PointT = pcl::PointXYZRGB;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointNT = pcl::PointNormal ;
    using PointNCloudT = pcl::PointCloud<PointNT>;
    using NormalCloudT = pcl::PointCloud<pcl::Normal>;
    using HistT = pcl::Histogram<33>;
    using HistCloudT = pcl::PointCloud<HistT>;

    bool visualize_features;
public:
    void calculate_features(Eigen::VectorXf &global_features, HistCloudT::Ptr& local_features, PointCloudT::Ptr& segment,
                            NormalCloudT::Ptr& segment_normals, PointCloudT::Ptr& full_segment) const;
    segment_features(bool visualize_features = false);
};

#endif // SEGMENT_FEATURES_H
