#ifndef SEGMENT_FEATURES_H
#define SEGMENT_FEATURES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

class segment_features
{
private:
    using PointT = pcl::PointXYZRGB;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointNT = pcl::PointNormal ;
    using PointNCloudT = pcl::PointCloud<PointNT>;
    using NormalCloudT = pcl::PointCloud<pcl::Normal>;
public:
    void calculate_features(Eigen::VectorXf &feature, PointCloudT::Ptr& segment,
                            NormalCloudT::Ptr& segment_normals, PointCloudT::Ptr& full_segment) const;
    segment_features();
};

#endif // SEGMENT_FEATURES_H
