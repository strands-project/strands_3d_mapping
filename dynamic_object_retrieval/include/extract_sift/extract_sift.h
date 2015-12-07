#ifndef EXTRACT_SIFT_H
#define EXTRACT_SIFT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using SiftT = pcl::Histogram<128>;
using SiftCloudT = pcl::PointCloud<SiftT>;

namespace extract_sift {

void extract_sift_for_sweep(const boost::filesystem::path& xml_path, bool visualize = false);
std::pair<SiftCloudT::Ptr, CloudT::Ptr> extract_sift_for_image(cv::Mat& image, cv::Mat& depth, const Eigen::Matrix3f& K);
std::pair<SiftCloudT::Ptr, CloudT::Ptr> extract_sift_for_cloud(CloudT::Ptr& cloud, const Eigen::Matrix3f& K);
// optionally provide the cloud as well
std::pair<SiftCloudT::Ptr, CloudT::Ptr> get_sift_for_cloud_path(const boost::filesystem::path& cloud_path);
std::pair<SiftCloudT::Ptr, CloudT::Ptr> get_sift_for_cloud_path(const boost::filesystem::path& cloud_path, CloudT::Ptr& cloud);
std::pair<SiftCloudT::Ptr, CloudT::Ptr> get_sift_for_cloud_path(const std::vector<boost::filesystem::path>& cloud_path);

} // namespace extract_sift

#endif // EXTRACT_SIFT_H
