#ifndef DYNAMIC_RETRIEVAL_H
#define DYNAMIC_RETRIEVAL_H

#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/visualize.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "dynamic_object_retrieval/extract_surfel_features.h"
#include "extract_sift/extract_sift.h"
#include "object_3d_retrieval/pfhrgb_estimation.h"
#include "object_3d_retrieval/shot_estimation.h"

#include <Stopwatch.h>

#define VT_PRECOMPILE
#include <vocabulary_tree/vocabulary_tree.h>
#include <object_3d_retrieval/register_objects.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <dynamic_object_retrieval/definitions.h>

#define WITH_SURFEL_NORMALS 1

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<N>;
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
    using type = std::vector<boost::filesystem::path>;
};

template <typename IndexT>
std::vector<boost::filesystem::path> get_retrieved_paths(const std::vector<IndexT>& scores, const vocabulary_summary& summary)
{
    // here we'll make use of the info saved in the data_summaries
    // maybe we should cache these as well as they might get big?
    data_summary noise_summary;
    noise_summary.load(boost::filesystem::path(summary.noise_data_path));
    data_summary annotated_summary;
    annotated_summary.load(boost::filesystem::path(summary.annotated_data_path));

    std::vector<boost::filesystem::path> retrieved_paths;
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
std::vector<std::pair<boost::filesystem::path, vocabulary_tree<HistT, 8>::result_type> >
get_retrieved_path_scores(const std::vector<vocabulary_tree<HistT, 8>::result_type>& scores, const vocabulary_summary& summary)
{
    std::vector<boost::filesystem::path> paths = get_retrieved_paths(scores, summary);
    std::vector<std::pair<boost::filesystem::path, vocabulary_tree<HistT, 8>::result_type> > path_scores;
    for (auto tup : dynamic_object_retrieval::zip(paths, scores)) {
        path_scores.push_back(std::make_pair(boost::get<0>(tup), boost::get<1>(tup)));
    }
    return path_scores;
}

std::vector<std::pair<std::vector<boost::filesystem::path>, grouped_vocabulary_tree<HistT, 8>::result_type> >
get_retrieved_path_scores(const std::vector<grouped_vocabulary_tree<HistT, 8>::result_type>& scores, const vocabulary_summary& summary)
{
    // OK, now we need to get the paths for all of the stuff within the scans instead, so basically
    // we need to get a subsegment_sweep__path_iterator and add all of the path corresponding to groups
    std::vector<std::pair<std::vector<boost::filesystem::path>, grouped_vocabulary_tree<HistT, 8>::result_type> > path_scores;

    int temp = 0;
    for (const grouped_vocabulary_tree<HistT, 8>::result_type& score : scores) {
        std::cout << "Iteration nbr: " << temp << std::endl;
        std::cout << "Scores size: " << scores.size() << std::endl;
        std::cout << "Subsegments in group: " << score.subgroup_group_indices.size() << std::endl;
        path_scores.push_back(std::make_pair(std::vector<boost::filesystem::path>(), score));

        int sweep_id = score.group_index;
        std::cout << "Getting sweep xml for group: " << sweep_id << std::endl;
        std::cout << "With subsegment: " << score.subgroup_index << std::endl;
        boost::filesystem::path sweep_path = dynamic_object_retrieval::get_sweep_xml(sweep_id, summary);
        std::cout << "Getting a subsegment iterator for sweep: " << sweep_path.string() << std::endl;
        if (summary.subsegment_type == "subsegment" || summary.subsegment_type == "supervoxel") {
            dynamic_object_retrieval::sweep_subsegment_keypoint_map subsegments(sweep_path); // this is a problem
            std::cout << "Finished getting a subsegment iterator for sweep: " << sweep_path.string() << std::endl;
            int counter = 0;
            for (const boost::filesystem::path& path : subsegments) {
                if (std::find(score.subgroup_group_indices.begin(), score.subgroup_group_indices.end(), counter) != score.subgroup_group_indices.end()) {
                    path_scores.back().first.push_back(path);
                }
                ++counter;
            }
        }
        else if (summary.subsegment_type == "convex_segment") {
            dynamic_object_retrieval::sweep_convex_segment_map subsegments(sweep_path); // this is a problem
            std::cout << "Finished getting a subsegment iterator for sweep: " << sweep_path.string() << std::endl;
            int counter = 0;
            for (const boost::filesystem::path& path : subsegments) {
                if (std::find(score.subgroup_group_indices.begin(), score.subgroup_group_indices.end(), counter) != score.subgroup_group_indices.end()) {
                    path_scores.back().first.push_back(path);
                }
                ++counter;
            }
        }
        else {
            std::cout << summary.subsegment_type << " not a valid subsegment type..." << std::endl;
            exit(-1);
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
    vt.query_vocabulary(scores, features, nbr_query);

    return get_retrieved_path_scores(scores, summary);
}

void insert_index_score(std::vector<std::pair<int, double> >& weighted_indices, const vocabulary_tree<HistT, 8>::result_type& index, float score)
{
    weighted_indices.push_back(std::make_pair(index.index, score));
}

void insert_index_score(std::vector<std::pair<std::set<int>, double> >& weighted_indices, const grouped_vocabulary_tree<HistT, 8>::result_type& index, float score)
{
    weighted_indices.push_back(std::make_pair(std::set<int>(), score));
    for (int subgroup_index : index.subgroup_global_indices) {
        weighted_indices.back().first.insert(subgroup_index);
    }
}

template <typename VocabularyT>
struct segment_index {
    using type = int;
};

template <>
struct segment_index<grouped_vocabulary_tree<HistT, 8> > {
    using type = std::set<int>;
};

float compute_cloud_volume(CloudT::Ptr& cloud)
{
    float resolution = 0.05f;
    pcl::octree::OctreePointCloud<PointT> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    std::vector<PointT, Eigen::aligned_allocator<PointT> > dummy;
    float centers = octree.getOccupiedVoxelCenters(dummy);
    return centers*resolution*resolution*resolution;
}

// this should definitely be generic to both!
template <typename VocabularyT>
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >
reweight_query(CloudT::Ptr& query_cloud, HistCloudT::Ptr& features, SiftCloudT::Ptr& sift_features,
               CloudT::Ptr& sift_keypoints, size_t nbr_query, VocabularyT& vt,
               const std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >& path_scores,
               const boost::filesystem::path& vocabulary_path, const vocabulary_summary& summary)
{
    using result_type = std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >;
    using reweight_type = std::pair<typename segment_index<VocabularyT>::type, double>;

    float query_volume = compute_cloud_volume(query_cloud);

    TICK("registration_score");
    //std::map<int, double> weighted_indices; // it would make more sense to keep a vector of sorted indices here I guess?
    std::vector<reweight_type> weighted_indices;
    double weight_sum = 0.0;
    for (auto s : path_scores) {
        std::cout << "Loading sift for: " << s.second.index << std::endl;

        CloudT::Ptr match_sift_keypoints;
        SiftCloudT::Ptr match_sift_cloud;
        CloudT::Ptr match_cloud;

        // we could probably use the path for this??? would be nicer with something else
        // on the other hand the path makes it open if we cache or not
        std::tie(match_sift_cloud, match_sift_keypoints, match_cloud) = extract_sift::get_sift_for_cloud_path(s.first);
        std::cout << "Number of sift features for match: " << match_sift_cloud->size() << std::endl;

        register_objects ro;
        ro.set_input_clouds(sift_keypoints, match_sift_keypoints);
        //ro.set_input_clouds(query_cloud, match_cloud); // here we should have the actual clouds instead
        ro.do_registration(sift_features, match_sift_cloud, sift_keypoints, match_sift_keypoints);

        // color score is not used atm
        double spatial_score, color_score;
        std::tie(spatial_score, color_score) = ro.get_match_score();
        if (std::isinf(spatial_score)) {
            continue;
        }
        // TODO: vt index is not correct for grouped_vocabulary
        //weighted_indices.insert(std::make_pair(s.second.index, spatial_score));
        if (spatial_score != 0.0f) {
            insert_index_score(weighted_indices, s.second, spatial_score);

            weight_sum += spatial_score;
        }

        /*
        float match_volume = compute_cloud_volume(match_cloud);
        float score = std::min(query_volume/match_volume, match_volume/query_volume);
        insert_index_score(weighted_indices, s.second, score);
        weight_sum += score;
        */
    }

    for (reweight_type& w : weighted_indices) {
        //std::cout << w.first << " score: " << w.second << std::endl;
        w.second *= double(weighted_indices.size())/weight_sum;
    }
    TOCK("registration_score");

    std::cout << "Starting re-weighting" << std::endl;

    TICK("reweighting");
    // TODO: improve the weighting to be done in the querying instead, makes way more sense
    std::map<int, double> original_norm_constants;
    std::map<vocabulary_tree<HistT, 8>::node*, double> original_weights; // maybe change this to e.g. node_type
    vt.compute_new_weights(original_norm_constants, original_weights, weighted_indices, features);
    TOCK("reweighting");

    std::cout << "Done re-weighting" << std::endl;

    std::cout << "Starting querying" << std::endl;

    result_type scores = query_vocabulary(features, nbr_query, vt, vocabulary_path, summary);

    std::cout << "Done querying" << std::endl;

    std::cout << "Restoring weights" << std::endl;

    vt.restore_old_weights(original_norm_constants, original_weights);

    std::cout << "Done restoring weights" << std::endl;

    return scores;
}

// take a potentially cached vt as argument, to allow caching
// potentially mark this as DEPRECATED, use the funcion below instead
template <typename VocabularyT>
std::pair<std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >,
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> > >
query_reweight_vocabulary(VocabularyT& vt, const boost::filesystem::path& query_features, size_t nbr_query,
                          const boost::filesystem::path& vocabulary_path,
                          const vocabulary_summary& summary, bool do_reweighting = true)
{
    using result_type = std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

    if (vt.empty()) {
        load_vocabulary(vt, vocabulary_path);
        vt.set_min_match_depth(3);
        vt.compute_normalizing_constants();
    }

    HistCloudT::Ptr features(new HistCloudT);
    pcl::io::loadPCDFile(query_features.string(), *features);

    result_type retrieved_paths = query_vocabulary(features, nbr_query, vt, vocabulary_path, summary);

    if (!do_reweighting) {
        return make_pair(retrieved_paths, result_type());
    }

    SiftCloudT::Ptr sift_features;
    CloudT::Ptr sift_keypoints;
    CloudT::Ptr query_cloud;
    // TODO: this is actually not correct, query_features is not the cloud
    std::tie(sift_features, sift_keypoints, query_cloud) = extract_sift::get_sift_for_cloud_path(query_features);
    result_type reweighted_paths = reweight_query(query_cloud, features, sift_features, sift_keypoints, 10, vt, retrieved_paths, vocabulary_path, summary);

    return make_pair(retrieved_paths, reweighted_paths);
}

template <typename VocabularyT>
std::pair<std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >,
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> > >
query_reweight_vocabulary(VocabularyT& vt, CloudT::Ptr& query_cloud, const Eigen::Matrix3f& K, size_t nbr_query,
                          const boost::filesystem::path& vocabulary_path,
                          const vocabulary_summary& summary, bool do_reweighting = true)
{
    using result_type = std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

    if (vt.empty()) {
        load_vocabulary(vt, vocabulary_path);
        vt.set_min_match_depth(3);
        vt.compute_normalizing_constants();
    }

    std::cout << "Computing query features..." << std::endl;
    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT);
    pfhrgb_estimation::compute_query_features(features, keypoints, query_cloud);
    //shot_estimation::compute_query_features(features, keypoints, query_cloud);

    std::cout << "Querying vocabulary..." << std::endl;
    result_type retrieved_paths = query_vocabulary(features, nbr_query, vt, vocabulary_path, summary);

    if (!do_reweighting) {
        return make_pair(retrieved_paths, result_type());
    }

    std::cout << "Computing sift features for query..." << std::endl;
    SiftCloudT::Ptr sift_features;
    CloudT::Ptr sift_keypoints;
    tie(sift_features, sift_keypoints) = extract_sift::extract_sift_for_cloud(query_cloud, K);

    std::cout << "Reweighting and querying..." << std::endl;
    result_type reweighted_paths = reweight_query(query_cloud, features, sift_features, sift_keypoints, 10, vt, retrieved_paths, vocabulary_path, summary);

    return make_pair(retrieved_paths, reweighted_paths);
}

template <typename VocabularyT>
std::pair<std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >,
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> > >
query_reweight_vocabulary(VocabularyT& vt, CloudT::Ptr& query_cloud, cv::Mat& query_image, cv::Mat& query_depth,
                          const Eigen::Matrix3f& K, size_t nbr_query, const boost::filesystem::path& vocabulary_path,
                          const vocabulary_summary& summary, bool do_reweighting = true)
{
    using result_type = std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

    if (vt.empty()) {
        load_vocabulary(vt, vocabulary_path);
        vt.set_min_match_depth(3);
        vt.compute_normalizing_constants();

        std::cout << "Mean leaves: " << vt.get_mean_leaf_points() << std::endl;
    }

    std::cout << "Computing query features..." << std::endl;
    TICK("compute_query_features");
    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT);
    pfhrgb_estimation::compute_surfel_features(features, keypoints, query_cloud, false, true);
    //shot_estimation::compute_features(features, keypoints, query_cloud);
    TOCK("compute_query_features");

    std::cout << "Querying vocabulary..." << std::endl;

    if (!do_reweighting) {
        result_type retrieved_paths = query_vocabulary(features, nbr_query, vt, vocabulary_path, summary);
        return make_pair(retrieved_paths, result_type());
    }

    TICK("query_vocabulary");

    result_type retrieved_paths = query_vocabulary(features, 2*nbr_query, vt, vocabulary_path, summary);

    TOCK("query_vocabulary");

    std::cout << "Computing sift features for query..." << std::endl;
    TICK("extract_sift_features");
    SiftCloudT::Ptr sift_features;
    CloudT::Ptr sift_keypoints;
    tie(sift_features, sift_keypoints) = extract_sift::extract_sift_for_image(query_image, query_depth, K);
    TOCK("extract_sift_features");

    std::cout << "Reweighting and querying..." << std::endl;
    std::cout << "Number of query sift features: " << sift_features->size() << std::endl;
    TICK("query_reweight_vocabulary");
    result_type reweighted_paths = reweight_query(query_cloud, features, sift_features, sift_keypoints, nbr_query, vt, retrieved_paths, vocabulary_path, summary);
    TOCK("query_reweight_vocabulary");

    return std::make_pair(retrieved_paths, reweighted_paths);
}

template <typename VocabularyT>
std::pair<std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >,
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> > >
query_reweight_vocabulary(VocabularyT& vt, CloudT::Ptr& query_cloud, cv::Mat& query_image, cv::Mat& query_depth,
                          const Eigen::Matrix3f& K, size_t nbr_query, const boost::filesystem::path& vocabulary_path,
                          const vocabulary_summary& summary, SurfelCloudT::Ptr& surfel_map, bool do_reweighting = true)
{
    using result_type = std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

    if (vt.empty()) {
        load_vocabulary(vt, vocabulary_path);
        vt.set_min_match_depth(3);
        vt.compute_normalizing_constants();

        std::cout << "Mean leaves: " << vt.get_mean_leaf_points() << std::endl;
    }

    std::cout << "Computing query features..." << std::endl;
    TICK("compute_query_features");
    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT);
    dynamic_object_retrieval::compute_query_features(features, keypoints, query_cloud, surfel_map);
    TOCK("compute_query_features");

    std::cout << "Querying vocabulary..." << std::endl;

    if (!do_reweighting) {
        result_type retrieved_paths = query_vocabulary(features, nbr_query, vt, vocabulary_path, summary);
        return make_pair(retrieved_paths, result_type());
    }

    TICK("query_vocabulary");

    result_type retrieved_paths = query_vocabulary(features, 2*nbr_query, vt, vocabulary_path, summary);

    TOCK("query_vocabulary");

    std::cout << "Computing sift features for query..." << std::endl;
    TICK("extract_sift_features");
    SiftCloudT::Ptr sift_features;
    CloudT::Ptr sift_keypoints;
    tie(sift_features, sift_keypoints) = extract_sift::extract_sift_for_image(query_image, query_depth, K);
    TOCK("extract_sift_features");

    std::cout << "Reweighting and querying..." << std::endl;
    std::cout << "Number of query sift features: " << sift_features->size() << std::endl;
    TICK("query_reweight_vocabulary");
    result_type reweighted_paths = reweight_query(query_cloud, features, sift_features, sift_keypoints, nbr_query, vt, retrieved_paths, vocabulary_path, summary);
    TOCK("query_reweight_vocabulary");

    return std::make_pair(retrieved_paths, reweighted_paths);
}

template <typename VocabularyT>
std::pair<std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >,
std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> > >
query_reweight_vocabulary(VocabularyT& vt, HistCloudT::Ptr& query_features,
                          size_t nbr_query, const boost::filesystem::path& vocabulary_path,
                          const vocabulary_summary& summary)
{
    using result_type = std::vector<std::pair<typename path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

    if (vt.empty()) {
        load_vocabulary(vt, vocabulary_path);
        vt.set_min_match_depth(3);
        vt.compute_normalizing_constants();

        std::cout << "Mean leaves: " << vt.get_mean_leaf_points() << std::endl;
    }

    std::cout << "Querying vocabulary..." << std::endl;
    TICK("query_vocabulary");

    result_type retrieved_paths = query_vocabulary(query_features, nbr_query, vt, vocabulary_path, summary);
    TOCK("query_vocabulary");

    return make_pair(retrieved_paths, result_type());
}

}

#endif // DYNAMIC_RETRIEVAL_H
