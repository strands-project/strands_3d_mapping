#ifndef BENCHMARK_SEGMENTATION_H
#define BENCHMARK_SEGMENTATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <map>
#include <string>
#include <utility>
#include <functional>
#include <dynamic_object_retrieval/summary_types.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;

namespace benchmark_retrieval {

std::vector<CloudT::Ptr> perform_convex_segmentation(CloudT::Ptr& training_map, CloudT::Ptr& training_object,
                                                     NormalCloudT::Ptr& training_object_normals,
                                                     CloudT::Ptr& query_map, CloudT::Ptr& query_object,
                                                     const boost::filesystem::path& query_map_path);
std::map<std::string, std::pair<float, int> >
get_segmentation_scores_for_data(const std::function<std::vector<CloudT::Ptr>(CloudT::Ptr&, CloudT::Ptr&,
                                                                              NormalCloudT::Ptr&,
                                                                              CloudT::Ptr&, CloudT::Ptr&,
                                                                              const boost::filesystem::path& query_map_path)>& sfunc,
                                 const boost::filesystem::path& data_path);

} // namespace benchmark_retrieval

#endif // BENCHMARK_SEGMENTATION_H
