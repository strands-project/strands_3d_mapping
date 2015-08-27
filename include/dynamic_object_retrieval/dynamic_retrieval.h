#ifndef DYNAMIC_RETRIEVAL_H
#define DYNAMIC_RETRIEVAL_H

#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/visualize.h"
#include "extract_sift/extract_sift.h"

#include <vocabulary_tree/vocabulary_tree.h>
#include <object_3d_retrieval/register_objects.h>
#include <boost/filesystem.hpp>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;
using SiftT = pcl::Histogram<128>;
using SiftCloudT = pcl::PointCloud<SiftT>;
using index_score = vocabulary_tree<HistT, 8>::cloud_idx_score;
using path_index_score = std::tuple<boost::filesystem::path, int, float>;

namespace dynamic_object_retrieval {

std::vector<boost::filesystem::path> get_retrieved_paths(const std::vector<index_score>& scores, const vocabulary_summary& summary);
std::vector<path_index_score> get_retrieved_path_scores(const std::vector<index_score>& scores, const vocabulary_summary& summary);

// take a potentially cached vt as argument, this might allow us to choose vt type as well!
// this should just be the first round, then there should be a separate function for re-weighting
// maybe introduce a new struct type called path_index_score or something
template <typename VocabularyT>
std::vector<path_index_score> query_vocabulary(HistCloudT::Ptr& features, size_t nbr_query, VocabularyT& vt,
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

    return get_retrieved_path_scores(scores, summary);
}

// GAH, this method is horrible, refactor!
template <typename VocabularyT>
std::vector<path_index_score> reweight_query(HistCloudT::Ptr& features, SiftCloudT::Ptr& sift_features,
                                             CloudT::Ptr& sift_keypoints, size_t nbr_query, VocabularyT& vt,
                                             const std::vector<path_index_score>& path_scores,
                                             const boost::filesystem::path& vocabulary_path)
{
    map<int, double> weighted_indices;
    double weight_sum = 0.0;
    for (path_index_score s : path_scores) {

        CloudT::Ptr match_sift_keypoints;
        SiftCloudT::Ptr match_sift_cloud;

        // we could probably use the path for this??? would be nicer with something else
        // on the other hand the path makes it open if we cache or not
        tie(match_sift_cloud, match_sift_keypoints) = extract_sift::get_sift_for_cloud_path(get<0>(s));

        register_objects ro;
        ro.set_input_clouds(sift_keypoints, match_sift_keypoints);
        ro.do_registration(sift_features, match_sift_cloud, sift_keypoints, match_sift_keypoints);

        // color score is not used atm
        double spatial_score, color_score;
        tie(spatial_score, color_score) = ro.get_match_score();
        if (std::isinf(spatial_score)) {
            continue;
        }
        weighted_indices.insert(make_pair(get<2>(s), spatial_score));
        weight_sum += spatial_score;
    }

    for (pair<const int, double>& w : weighted_indices) {
        w.second *= double(weighted_indices.size())/weight_sum;
    }

    // TODO: improve the weighting to be done in the querying instead, makes way more sense
    map<int, double> original_norm_constants;
    map<vocabulary_tree<HistT, 8>::node*, double> original_weights;
    vt.compute_new_weights(original_norm_constants, original_weights, weighted_indices, features);

    vector<path_index_score> scores = query_vocabulary(features, nbr_query, vt, vocabulary_path);
    vt.restore_old_weights(original_norm_constants, original_weights);
    return scores;
}

}

#endif // DYNAMIC_RETRIEVAL_H
