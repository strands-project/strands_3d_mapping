#include "vocabulary_tree/vocabulary_tree.h"

#include <pcl/point_types.h>
#include <cereal/archives/binary.hpp>

//template class vocabulary_tree<pcl::PointXYZ, 8>;
template class vocabulary_tree<pcl::PointXYZRGB, 8>;
template class vocabulary_tree<pcl::Histogram<33>, 8>;
template class vocabulary_tree<pcl::Histogram<128>, 8>;
template class vocabulary_tree<pcl::Histogram<131>, 8>;
template class vocabulary_tree<pcl::Histogram<1344>, 8>;
//template void serialize(cereal::BinaryOutputArchive& archive, vocabulary_tree<pcl::Histogram<131>, 8>& vt);
//template void serialize(cereal::BinaryInputArchive& archive, vocabulary_tree<pcl::Histogram<131>, 8>& vt);
