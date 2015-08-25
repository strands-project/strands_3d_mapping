#ifndef DYNAMIC_RETRIEVAL_H
#define DYNAMIC_RETRIEVAL_H

#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/visualize.h"

#include <vocabulary_tree/vocabulary_tree.h>
#include <boost/filesystem.hpp>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;
using index_score = vocabulary_tree<HistT, 8>::cloud_idx_score;

namespace dynamic_object_retrieval {

// take a potentially cached vt as argument, this might allow us to choose vt type as well!
// this should just be the first round, then there should be a separate function for re-weighting
template <typename VocabularyT>
std::vector<boost::filesystem::path> query_vocabulary(HistCloudT::Ptr& features, size_t nbr_query, VocabularyT& vt,
                                                      const boost::filesystem::path& vocabulary_path)
{
    // probably not needed? or yes is might be, for annotations and paths
    vocabulary_summary summary;
    summary.load(vocabulary_path);

    // we need to cache the vt if we are to do this multiple times
    if (vt.empty()) {
        load_vocabulary(vt, vocabulary_path);
        vt.set_min_match_depth(3);
        vt.compute_normalizing_constants();
    }

    std::vector<index_score> scores;
    vt.top_combined_similarities(scores, features, nbr_query);

    // here we'll make use of the info saved in the data_summaries
    // maybe we should cache these as well as they might get big?
    data_summary noise_summary;
    noise_summary.load(boost::filesystem::path(summary.noise_data_path));
    data_summary annotated_summary;
    annotated_summary.load(boost::filesystem::path(summary.annotated_data_path));

    std::vector<boost::filesystem::path> retrieved_paths;
    size_t offset = summary.nbr_noise_segments;
    for (index_score s : scores) {
        if (s.first < offset) {
            retrieved_paths.push_back(boost::filesystem::path(noise_summary.index_convex_segment_paths[s.first]));
        }
        else {
            retrieved_paths.push_back(boost::filesystem::path(annotated_summary.index_convex_segment_paths[s.first-offset]));
        }
    }

    return retrieved_paths;
}

}

#endif // DYNAMIC_RETRIEVAL_H
