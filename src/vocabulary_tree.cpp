#include "vocabulary_tree/vocabulary_tree.h"

#include <pcl/point_types.h>

template class vocabulary_tree<pcl::PointXYZ, 8>;
template class vocabulary_tree<pcl::PointXYZRGB, 8>;
template class vocabulary_tree<pcl::Histogram<33>, 8>;
template class vocabulary_tree<pcl::Histogram<128>, 8>;
template class vocabulary_tree<pcl::Histogram<131>, 8>;
