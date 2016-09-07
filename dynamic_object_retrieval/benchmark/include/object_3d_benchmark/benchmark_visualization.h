#ifndef BENCHMARK_VISUALIZATION_H
#define BENCHMARK_VISUALIZATION_H

#include <dynamic_object_retrieval/summary_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

namespace benchmark_retrieval {

void put_text(cv::Mat& image, const std::string& text);
std::pair<cv::Mat, std::vector<cv::Mat> > make_image(std::vector<CloudT::Ptr>& results,  const Eigen::Matrix4f& room_transform,
                                                     std::vector<boost::filesystem::path>& sweep_paths, const std::vector<std::string>& optional_text);
cv::Mat add_query_image(cv::Mat& results, cv::Mat& query_image, const std::string& query_label);

// this will alter the look of the point clouds, but for the better I think
//template <typename IndexT, typename PathT>
cv::Mat make_visualization_image(cv::Mat& query_image, const std::string& query_label, std::vector<CloudT::Ptr>& clouds,
                                 std::vector<boost::filesystem::path>& sweep_paths, const std::vector<std::string>& optional_text,
                                 const Eigen::Matrix4f& T);
cv::Mat make_visualization_image(cv::Mat& query_image, const std::string& query_label, std::vector<CloudT::Ptr>& clouds,
                                 std::vector<boost::filesystem::path>& sweep_paths, const std::vector<std::string>& optional_text,
                                 const Eigen::Matrix4f& T, std::vector<cv::Mat>& individual_images);
cv::Mat make_visualization_image(CloudT::Ptr& query_cloud, cv::Mat& query_mask, const boost::filesystem::path& query_path,
                                 const Eigen::Matrix3f& K, const Eigen::Matrix4f& query_transform,
                                 const std::string& query_label, std::vector<CloudT::Ptr>& clouds,
                                 std::vector<boost::filesystem::path>& sweep_paths, const std::vector<std::string>& optional_text,
                                 const Eigen::Matrix4f& T);

} // namespace benchmark_retrieval

#endif // BENCHMARK_VISUALIZATION_H
