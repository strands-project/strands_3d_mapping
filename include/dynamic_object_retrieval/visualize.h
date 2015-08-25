#ifndef DYNAMIC_VISUALIZE_H
#define DYNAMIC_VISUALIZE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vocabulary_tree/vocabulary_tree.h>
#include <boost/filesystem.hpp>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

namespace dynamic_object_retrieval {

void visualize(CloudT::Ptr& cloud);
void save_vocabulary(vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path);
void load_vocabulary(vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path);

}

#endif // DYNAMIC_VISUALIZE_H
