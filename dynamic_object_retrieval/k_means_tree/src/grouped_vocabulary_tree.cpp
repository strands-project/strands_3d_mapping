#include "grouped_vocabulary_tree/grouped_vocabulary_tree.h"

#include <pcl/point_types.h>
#include <cereal/archives/binary.hpp>

template class grouped_vocabulary_tree<pcl::PointXYZRGB, 8>;
template class grouped_vocabulary_tree<pcl::Histogram<33>, 8>;
template class grouped_vocabulary_tree<pcl::Histogram<128>, 8>;
template class grouped_vocabulary_tree<pcl::Histogram<131>, 8>;
template class grouped_vocabulary_tree<pcl::Histogram<1344>, 8>;
template class grouped_vocabulary_tree<pcl::Histogram<250>, 8>;
