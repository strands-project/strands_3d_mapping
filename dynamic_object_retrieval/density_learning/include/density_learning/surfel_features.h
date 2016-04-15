#ifndef SURFEL_FEATURES_H
#define SURFEL_FEATURES_H

#include "dynamic_object_retrieval/definitions.h"
#include "dynamic_object_retrieval/surfel_type.h"
#include <opencv2/core/core.hpp>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

namespace surfel_features {

void compute_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud,
                      NormalCloudT::Ptr& normals, bool visualize_features);
void compute_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud,
                      NormalCloudT::Ptr& normals, float threshold, bool visualize_features);
NormalCloudT::Ptr compute_surfel_normals(SurfelCloudT::Ptr& surfel_cloud, CloudT::Ptr& segment);
std::pair<CloudT::Ptr, NormalCloudT::Ptr>
cloud_normals_from_surfel_mask(SurfelCloudT::Ptr& surfel_cloud, const cv::Mat& mask,
                               const Eigen::Matrix4f& T, const Eigen::Matrix3f& K);
SurfelCloudT::Ptr load_surfel_cloud_for_sweep(const std::string& xml);

} // namespace surfel_features

#endif // SURFEL_FEATURES_H
