#ifndef DYNAMIC_VISUALIZE_H
#define DYNAMIC_VISUALIZE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define VT_PRECOMPILE
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>
#include <boost/filesystem.hpp>
#include "dynamic_object_retrieval/definitions.h"

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;

namespace dynamic_object_retrieval {

void visualize(CloudT::Ptr& cloud);
void visualize(CloudT::Ptr& cloud, float subsample_size);
void save_vocabulary(vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path);
void load_vocabulary(vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path);
void save_vocabulary(grouped_vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path);
void load_vocabulary(grouped_vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path);

}

#endif // DYNAMIC_VISUALIZE_H
