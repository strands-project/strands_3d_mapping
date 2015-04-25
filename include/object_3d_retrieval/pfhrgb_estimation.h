#ifndef PFHRGB_ESTIMATION_H
#define PFHRGB_ESTIMATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pfhrgb_estimation {

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using PfhRgbT = pcl::Histogram<250>;
using PfhRgbCloudT = pcl::PointCloud<PfhRgbT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using ShotT = pcl::Histogram<1344>;
using ShotCloudT = pcl::PointCloud<ShotT>;

void visualize_keypoints(CloudT::Ptr& cloud, CloudT::Ptr& keypoints);
void compute_query_features(PfhRgbCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud, bool visualize_features = false);
void compute_query_features(ShotCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud, bool visualize_features = false);
void compute_features(PfhRgbCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud, bool visualize_features = false);
void compute_features(ShotCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud, bool visualize_features = false);
void split_descriptor_points(std::vector<PfhRgbCloudT::Ptr>& split_features, std::vector<CloudT::Ptr>& split_keypoints,
                             PfhRgbCloudT::Ptr& features, CloudT::Ptr& keypoints, int expected_cluster_size = 30);
void split_descriptor_points(std::vector<ShotCloudT::Ptr>& split_features, std::vector<CloudT::Ptr>& split_keypoints,
                             ShotCloudT::Ptr& features, CloudT::Ptr& keypoints, int expected_cluster_size = 30);
void visualize_split_keypoints(std::vector<CloudT::Ptr>& split_keypoints);

}

#endif // PFHRGB_ESTIMATION_H
