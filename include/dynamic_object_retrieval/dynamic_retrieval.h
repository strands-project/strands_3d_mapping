#ifndef DYNAMIC_RETRIEVAL_H
#define DYNAMIC_RETRIEVAL_H

#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/visualize.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "extract_sift/extract_sift.h"
#include "object_3d_retrieval/pfhrgb_estimation.h"

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

namespace dynamic_object_retrieval {

template <typename VocabularyT>
struct path_result {
    using type =  boost::filesystem::path;
};

template <>
struct path_result<grouped_vocabulary_tree<HistT, 8> > {
    using type = vector<boost::filesystem::path>;
};

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

template <typename IndexT, typename GroupT>
std::vector<std::pair<std::vector<boost::filesystem::path>, IndexT> >
get_retrieved_path_scores(const std::vector<IndexT>& scores, const std::vector<GroupT>& groups, const vocabulary_summary& summary)
{
    // OK, now we need to get the paths for all of the stuff within the scans instead, so basically
    // we need to get a subsegment_sweep__path_iterator and add all of the path corresponding to groups
    std::vector<std::pair<std::vector<boost::filesystem::path>, IndexT> > path_scores;

    int temp = 0;
    for (auto tup : dynamic_object_retrieval::zip(scores, groups)) {
        std::cout << "Iteration nbr: " << temp << std::endl;
        std::cout << "Scores size: " << scores.size() << std::endl;
        std::cout << "Groups size: " << groups.size() << std::endl;
        std::cout << "Groups for iteration: " << get<1>(tup).size() << std::endl;
        path_scores.push_back(make_pair(vector<boost::filesystem::path>(), get<0>(tup)));

        int sweep_id = get<0>(tup).group_index;
        std::cout << "Getting sweep xml for group: " << sweep_id << std::endl;
        std::cout << "With subsegment: " << get<0>(tup).subgroup_index << std::endl;
        boost::filesystem::path sweep_path = dynamic_object_retrieval::get_sweep_xml(sweep_id, summary);
        std::cout << "Getting a subsegment iterator for sweep: " << sweep_path.string() << std::endl;
        dynamic_object_retrieval::sweep_subsegment_keypoint_map subsegments(sweep_path);
        std::cout << "Finished getting a subsegment iterator for sweep: " << sweep_path.string() << std::endl;
        int counter = 0;
        for (const boost::filesystem::path& path : subsegments) {
            if (std::find(get<1>(tup).begin(), get<1>(tup).end(), counter) != get<1>(tup).end()) {
                path_scores.back().first.push_back(path);
            }
            ++counter;
        }
        std::cout << "Finished getting sweep xml for group: " << sweep_id << std::endl;
        std::cout << "Got " << path_scores.back().first.size() << " subsegments..." << std::endl;
        ++temp;
    }

    return path_scores;
}

template <typename VocabularyT>
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >
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

// OK, the solution here is to turn the groups into the path scores
template <>
std::vector<std::pair<path_result<grouped_vocabulary_tree<HistT, 8> >::type, typename grouped_vocabulary_tree<HistT, 8>::result_type> >
query_vocabulary(HistCloudT::Ptr& features, size_t nbr_query, grouped_vocabulary_tree<HistT, 8>& vt,
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
    std::vector<typename grouped_vocabulary_tree<HistT, 8>::result_type> scores;
    std::vector<typename grouped_vocabulary_tree<HistT, 8>::group_type> groups;
    vt.query_vocabulary(scores, groups, features, nbr_query);

    return get_retrieved_path_scores(scores, groups, summary);
}

// this should definitely be generic to both!
template <typename VocabularyT>
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >
reweight_query(HistCloudT::Ptr& features, SiftCloudT::Ptr& sift_features,
               CloudT::Ptr& sift_keypoints, size_t nbr_query, VocabularyT& vt,
               const std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >& path_scores,
               const boost::filesystem::path& vocabulary_path, const vocabulary_summary& summary)
{
    using result_type = std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

    map<int, double> weighted_indices;
    double weight_sum = 0.0;
    for (auto s : path_scores) {
        std::cout << "Loading sift for: " << s.second.index << std::endl;

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
// potentially mark this as DEPRECATED, use the funcion below instead
template <typename VocabularyT>
pair<std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >,
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> > >
query_reweight_vocabulary(const boost::filesystem::path& cloud_path, size_t nbr_query,
                          const boost::filesystem::path& vocabulary_path,
                          const vocabulary_summary& summary)
{
    using result_type = std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

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

template <typename VocabularyT>
pair<std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >,
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> > >
query_reweight_vocabulary(CloudT::Ptr& query_cloud, const Eigen::Matrix3f& K, size_t nbr_query,
                          const boost::filesystem::path& vocabulary_path,
                          const vocabulary_summary& summary)
{
    using result_type = std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >;


    std::cout << "Computing query features..." << std::endl;
    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT);
    pfhrgb_estimation::compute_features(features, keypoints, query_cloud);

    std::cout << "Querying vocabulary..." << std::endl;
    VocabularyT vt;
    result_type retrieved_paths = query_vocabulary(features, nbr_query, vt, vocabulary_path, summary);

    std::cout << "Computing sift features for query..." << std::endl;
    SiftCloudT::Ptr sift_features;
    CloudT::Ptr sift_keypoints;
    tie(sift_features, sift_keypoints) = extract_sift::extract_sift_for_cloud(query_cloud, K);

    std::cout << "Reweighting and querying..." << std::endl;
    result_type reweighted_paths = reweight_query(features, sift_features, sift_keypoints, 10, vt, retrieved_paths, vocabulary_path, summary);

    return make_pair(retrieved_paths, reweighted_paths);
}

}

#endif // DYNAMIC_RETRIEVAL_H
