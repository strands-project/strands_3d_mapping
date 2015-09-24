#ifndef DYNAMIC_RETRIEVAL_H
#define DYNAMIC_RETRIEVAL_H

#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/visualize.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "extract_sift/extract_sift.h"

#include <vocabulary_tree/vocabulary_tree.h>
#include <object_3d_retrieval/register_objects.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;
using SiftT = pcl::Histogram<128>;
using SiftCloudT = pcl::PointCloud<SiftT>;
using index_score = vocabulary_tree<HistT, 8>::cloud_idx_score;

namespace dynamic_object_retrieval {

template <typename IndexT>
vector<boost::filesystem::path> get_retrieved_paths(const vector<IndexT>& scores, const vocabulary_summary& summary)
{
    // here we'll make use of the info saved in the data_summaries
    // maybe we should cache these as well as they might get big?
    data_summary noise_summary;
    noise_summary.load(boost::filesystem::path(summary.noise_data_path));
    data_summary annotated_summary;
    annotated_summary.load(boost::filesystem::path(summary.annotated_data_path));

    vector<boost::filesystem::path> retrieved_paths;
    size_t offset = summary.nbr_noise_segments;
    for (IndexT s : scores) {
        // TODO: vt index is not correct for grouped_vocabulary
        if (s.index < offset) {
            retrieved_paths.push_back(boost::filesystem::path(noise_summary.index_convex_segment_paths[s.index]));
        }
        else {
            retrieved_paths.push_back(boost::filesystem::path(annotated_summary.index_convex_segment_paths[s.index-offset]));
        }
    }

    return retrieved_paths;
}

//std::vector<boost::filesystem::path> get_retrieved_paths(const std::vector<index_score>& scores, const vocabulary_summary& summary);
template <typename IndexT>
std::vector<std::pair<boost::filesystem::path, IndexT> >
get_retrieved_path_scores(const std::vector<IndexT>& scores, const vocabulary_summary& summary)
{
    vector<boost::filesystem::path> paths = get_retrieved_paths(scores, summary);
    std::vector<std::pair<boost::filesystem::path, IndexT> > path_scores;
    for (auto tup : dynamic_object_retrieval::zip(paths, scores)) {
        path_scores.push_back(make_pair(get<0>(tup), get<1>(tup)));
    }
    return path_scores;
}

// take a potentially cached vt as argument, this might allow us to choose vt type as well!
// this should just be the first round, then there should be a separate function for re-weighting
// maybe introduce a new struct type called path_index_score or something
template <typename VocabularyT>
std::vector<std::pair<boost::filesystem::path, typename VocabularyT::result_type> >
query_vocabulary(HistCloudT::Ptr& features, size_t nbr_query, VocabularyT& vt,
                 const boost::filesystem::path& vocabulary_path,
                 const vocabulary_summary& summary)
{
    // we need to cache the vt if we are to do this multiple times
    if (vt.empty()) {
        load_vocabulary(vt, vocabulary_path);
        vt.set_min_match_depth(3);
        vt.compute_normalizing_constants();
    }

    // add some common methods in vt for querying, really only one!
    std::vector<typename VocabularyT::result_type> scores;
    vt.query_vocabulary(scores, features, nbr_query);

    return get_retrieved_path_scores(scores, summary);
}

// this might have to be with a specialization for grouped_vocabulary_tree
// or rather, I could use a special return type for grouped voc tree
template <typename VocabularyT>
std::vector<std::pair<boost::filesystem::path, typename VocabularyT::result_type> >
reweight_query(HistCloudT::Ptr& features, SiftCloudT::Ptr& sift_features,
               CloudT::Ptr& sift_keypoints, size_t nbr_query, VocabularyT& vt,
               const std::vector<std::pair<boost::filesystem::path, typename VocabularyT::result_type> >& path_scores,
               const boost::filesystem::path& vocabulary_path, const vocabulary_summary& summary)
{
    using result_type = std::vector<std::pair<boost::filesystem::path, typename VocabularyT::result_type> >;

    map<int, double> weighted_indices;
    double weight_sum = 0.0;
    for (auto s : path_scores) {

        CloudT::Ptr match_sift_keypoints;
        SiftCloudT::Ptr match_sift_cloud;

        // we could probably use the path for this??? would be nicer with something else
        // on the other hand the path makes it open if we cache or not
        tie(match_sift_cloud, match_sift_keypoints) = extract_sift::get_sift_for_cloud_path(s.first);

        register_objects ro;
        ro.set_input_clouds(sift_keypoints, match_sift_keypoints);
        ro.do_registration(sift_features, match_sift_cloud, sift_keypoints, match_sift_keypoints);

        // color score is not used atm
        double spatial_score, color_score;
        tie(spatial_score, color_score) = ro.get_match_score();
        if (std::isinf(spatial_score)) {
            continue;
        }
        // TODO: vt index is not correct for grouped_vocabulary
        weighted_indices.insert(make_pair(s.second.index, spatial_score));
        weight_sum += spatial_score;
    }

    for (pair<const int, double>& w : weighted_indices) {
        w.second *= double(weighted_indices.size())/weight_sum;
    }

    // TODO: improve the weighting to be done in the querying instead, makes way more sense
    map<int, double> original_norm_constants;
    map<vocabulary_tree<HistT, 8>::node*, double> original_weights; // maybe change this to e.g. node_type
    vt.compute_new_weights(original_norm_constants, original_weights, weighted_indices, features);

    result_type scores = query_vocabulary(features, nbr_query, vt, vocabulary_path, summary);
    vt.restore_old_weights(original_norm_constants, original_weights);
    return scores;
}

// take a potentially cached vt as argument, to allow caching
// this should just be the first round, then there should be a separate function for re-weighting
// maybe introduce a new struct type called path_index_score or something
template <typename VocabularyT>
pair<std::vector<std::pair<boost::filesystem::path, typename VocabularyT::result_type> >,
std::vector<std::pair<boost::filesystem::path, typename VocabularyT::result_type> > >
query_reweight_vocabulary(const boost::filesystem::path& cloud_path, size_t nbr_query,
                          const boost::filesystem::path& vocabulary_path,
                          const vocabulary_summary& summary)
{
    using result_type = std::vector<std::pair<boost::filesystem::path, typename VocabularyT::result_type> >;

    HistCloudT::Ptr features(new HistCloudT);
    pcl::io::loadPCDFile(cloud_path.string(), *features);

    VocabularyT vt;
    result_type retrieved_paths = query_vocabulary(features, nbr_query, vt, vocabulary_path, summary);

    SiftCloudT::Ptr sift_features;
    CloudT::Ptr sift_keypoints;
    tie(sift_features, sift_keypoints) = extract_sift::get_sift_for_cloud_path(cloud_path);
    result_type reweighted_paths = reweight_query(features, sift_features, sift_keypoints, 10, vt, retrieved_paths, vocabulary_path, summary);

    return make_pair(retrieved_paths, reweighted_paths);
}

}

#endif // DYNAMIC_RETRIEVAL_H
