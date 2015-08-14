#include <iostream>
#include <chrono>

#include "object_3d_retrieval/object_retrieval.h"
#include "object_3d_retrieval/dataset_annotations.h"
#include "object_3d_retrieval/descriptor_config.h"
#include "object_3d_retrieval/retrieval_client.h"
#include "object_3d_retrieval/pfhrgb_estimation.h"
#include "object_3d_retrieval/register_objects.h"

#include <cereal/archives/binary.hpp>
#include <eigen_cereal/eigen_cereal.h>

using namespace std;
using namespace retrieval_client;

struct query_in_dataset_iterator {
    vector<dataset_annotations::voxel_annotation> annotations;
    object_retrieval& obr_segments_annotations;
    int i;
    bool get_next_query(string& instance, CloudT::Ptr& cloud, HistCloudT::Ptr& features, CloudT::Ptr& keypoints, int& scan_id, int& segment_id)
    {
        if (i >= annotations.size()) {
            return false;
        }
        dataset_annotations::voxel_annotation a = annotations[i];
        ++i;
        if (!a.full || !a.annotation_covered || !a.segment_covered) {
            return get_next_query(instance, cloud, features, keypoints, scan_id, segment_id);
        }
        instance = a.annotation;
        pcl::io::loadPCDFile(a.segment_file, *cloud);
        scan_id = scan_ind_for_segment(a.segment_id, obr_segments_annotations);
        segment_id = a.segment_id;
        return obr_segments_annotations.load_features_for_segment(features, keypoints, a.segment_id);
    }
    query_in_dataset_iterator(vector<dataset_annotations::voxel_annotation>& annotations, object_retrieval& obr) : annotations(annotations), obr_segments_annotations(obr), i(0)
    {

    }
};

// OK
template <typename Iterator>
void query_supervoxel_oversegments(Iterator& query_iterator, Eigen::Matrix3f& K,
                                   object_retrieval& obr_segments, object_retrieval& obr_scans,
                                   object_retrieval& obr_segments_annotations, object_retrieval& obr_scans_annotations,
                                   int noise_scans_size, const bool benchmark_reweighting)
{
    // TODO: these numbers should be adjusted depending on benchmark_reweighing
    const int nbr_query = 20;
    const int nbr_reweight_query = 31;
    const int nbr_initial_query = 200;

    map<vocabulary_tree<HistT, 8>::node*, int> mapping;

    if (obr_segments.gvt.empty()) {
        obr_segments.read_vocabulary(obr_segments.gvt);
    }
    obr_segments.gvt.set_min_match_depth(3);
    obr_segments.gvt.compute_normalizing_constants();

    read_supervoxel_groups(obr_segments); // this will not be needed eventually, should be part of class init

    obr_segments.gvt.compute_leaf_vocabulary_vectors(); // TODO: are leaf_vocabulary_vectors used anywhere?

    obr_segments.gvt.get_node_mapping(mapping);

    // this is something that I have to do in the setup I think? But we need
    //save_oversegmented_grouped_vocabulary_index_vectors(obr_scans, obr_segments);
    //save_oversegmented_grouped_vocabulary_index_vectors(obr_scans_annotations, obr_segments);
    //exit(0);

    map<string, pair<float, int> > instance_correct_ratios;
    map<string, pair<float, int> > usual_correct_ratios;
    map<string, pair<float, int> > reweight_correct_ratios;

    map<string, pair<int, int> > instance_mean_features;

    map<string, int> instance_number_queries;

    chrono::time_point<std::chrono::system_clock> start, end;
    start = chrono::system_clock::now();

    string instance;
    CloudT::Ptr cloud(new CloudT);
    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT); // add reading of keypoints as well
    int scan_id;
    int segment_id;
    while (query_iterator.get_next_query(instance, cloud, features, keypoints, scan_id, segment_id)) {

        instance_number_queries[instance] += 1;

        cout << "Features: " << features->size() << endl;

        vector<index_score> first_scores; // scores
        vector<index_score> second_scores; // updated_scores;
        vector<int> hints; // hints for where in image to start growing
        vector<vector<int> > oversegment_indices;
        find_top_oversegments_grow_and_score(first_scores, second_scores, hints, oversegment_indices, features, mapping, obr_segments,
                                             obr_scans, obr_scans_annotations, nbr_reweight_query, nbr_initial_query, noise_scans_size); // nbr_query if no reweight

        vector<index_score> reweight_scores;
        if (benchmark_reweighting) {
            reweight_query_vocabulary_sift(reweight_scores, second_scores, first_scores, hints, oversegment_indices, cloud, features, keypoints, segment_id, nbr_query,
                                           nbr_initial_query, obr_scans, obr_scans_annotations, obr_segments, obr_segments_annotations, noise_scans_size, mapping);
        }


        //second_scores.resize(nbr_query);
        //dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(instance_correct_ratios, instance, scan_id, second_scores, obr_scans_annotations, noise_scans_size);
        //first_scores.resize(nbr_query);
        //dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(usual_correct_ratios, instance, scan_id, first_scores, obr_scans_annotations, noise_scans_size);
        if (benchmark_reweighting) {
            reweight_scores.resize(nbr_query);
            dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(reweight_correct_ratios, instance, scan_id, reweight_scores, obr_scans_annotations, noise_scans_size);
        }
        cout << "Number of features: " << features->size() << endl;

        instance_mean_features[instance].first += features->size();
        instance_mean_features[instance].second += 1;
    }

    end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = end-start;

    cout << "Benchmark took " << elapsed_seconds.count() << " seconds" << endl;

    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << ":" << endl;
        cout << "Mean features: " << float(instance_mean_features[c.first].first)/float(instance_mean_features[c.first].second) << endl;
        cout << "Number of queries: " << instance_number_queries[c.first] << endl;
    }

    cout << "First round correct ratios: " << endl;
    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
    }

    for (pair<const string, pair<float, int> > c : usual_correct_ratios) {
        cout << c.first << " usual correct ratio: " << c.second.first/float(c.second.second) << endl;
    }

    for (pair<const string, pair<float, int> > c : reweight_correct_ratios) {
        cout << c.first << " reweight correct ratio: " << c.second.first/float(c.second.second) << endl;
    }
}

// OK
template <typename Iterator>
void query_supervoxels(Iterator& query_iterator, object_retrieval& obr_segments, object_retrieval& obr_segments_annotations,
                       object_retrieval& obr_scans_annotations, int noise_scans_size, int noise_segments_size, const bool benchmark_reweighting)
{
    // TODO: these numbers should be adjusted depending on benchmark_reweighing
    const int nbr_query = 20;
    const int nbr_reweight_query = 51;

    if (obr_segments.vt.empty()) {
        obr_segments.read_vocabulary(obr_segments.vt);
    }
    obr_segments.vt.set_min_match_depth(3);
    obr_segments.vt.compute_normalizing_constants();

    map<string, pair<float, int> > instance_correct_ratios;
    map<string, pair<float, int> > reweight_correct_ratios;

    vector<pair<float, int> > decay_correct_ratios;
    vector<int> intermediate_points;
    for (int i = 0; i < noise_scans_size; i += 100) {
        intermediate_points.push_back(i);
        decay_correct_ratios.push_back(make_pair(0.0f, 0));
    }

    chrono::time_point<std::chrono::system_clock> start, end;
    start = chrono::system_clock::now();

    string instance;
    CloudT::Ptr cloud(new CloudT);
    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT);
    int scan_id;
    int segment_id;
    while (query_iterator.get_next_query(instance, cloud, features, keypoints, scan_id, segment_id)) {
        vector<index_score> scores;
        obr_segments.vt.top_combined_similarities(scores, features, nbr_reweight_query);

        vector<index_score> reweight_scores;

        if (benchmark_reweighting) {
            reweight_query_vocabulary_sift(reweight_scores, scores, cloud, features, keypoints, segment_id, nbr_query,
                                           obr_segments, obr_segments_annotations, noise_segments_size);
        }

        scores.resize(nbr_query);
        for (index_score& s : scores) {
            if (s.first < noise_segments_size) {
                s.first = 0; // scan_ind_for_segment(s.first, obr_segments); // not needed since < noise_scans_size -> false
            }
            else {
                s.first = scan_ind_for_segment(s.first-noise_segments_size, obr_segments_annotations) + noise_scans_size;
            }
        }

        reweight_scores.resize(nbr_query);
        for (index_score& s : reweight_scores) {
            if (s.first < noise_segments_size) {
                s.first = 0; // scan_ind_for_segment(s.first, obr_segments); // not needed since < noise_scans_size -> false
            }
            else {
                s.first = scan_ind_for_segment(s.first-noise_segments_size, obr_segments_annotations) + noise_scans_size;
            }
        }

        dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(instance_correct_ratios, instance, scan_id, scores, obr_scans_annotations, noise_scans_size);

        if (benchmark_reweighting) {
            dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(reweight_correct_ratios, instance, scan_id, reweight_scores, obr_scans_annotations, noise_scans_size);
        }

        cout << "Number of features: " << features->size() << endl;
    }

    end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = end-start;

    cout << "Benchmark took " << elapsed_seconds.count() << " seconds" << endl;

    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
    }

    for (pair<const string, pair<float, int> > c : reweight_correct_ratios) {
        cout << c.first << " reweight correct ratio: " << c.second.first/float(c.second.second) << endl;
    }
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        cout << " Please supply the annotated data path, the noise data path and the option: \n"
             << " 1. Benchmark convex segment vocabulary tree querying\n"
             << " 2. Benchmark convex segment vocabulary tree with re-weighting"
             << " 3. Benchmark subsegment vocabulary tree querying\n"
             << " 4. Benchmark subsegment vocabulary tree with re-weighting" << endl;
        return 0;
    }

    string annotated_root_path(argv[1]); //"/home/nbore/Data/Instances/";
    string annotated_scan_path = annotated_root_path + "scan_segments";
    string annotated_segment_path = annotated_root_path + "supervoxel_segments";

    string noise_root_path(argv[2]); //"/home/nbore/Data/semantic_map/";
    string noise_scan_path = noise_root_path + "scan_segments";
    string noise_segment_path = noise_root_path + "supervoxel_segments";

    int option = atoi(argv[3]);

    object_retrieval obr_scans_annotated(annotated_scan_path);
    obr_scans_annotated.segment_name = "scan";
    object_retrieval obr_segments_annotated(annotated_segment_path);

    object_retrieval obr_scans_noise(noise_scan_path);
    obr_scans_noise.segment_name = "scan";
    object_retrieval obr_segments_noise(noise_segment_path);

    int noise_scans_size = retrieval_client::read_noise_segment_size(obr_scans_noise);
    int noise_segments_size = retrieval_client::read_noise_segment_size(obr_segments_noise);

    Eigen::Matrix3f K;
    string matrix_file = annotated_root_path + "K.cereal"; // this is a bit of a hack, we don't actually store this at any point
    ifstream in(matrix_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(K);
    }

    vector<dataset_annotations::voxel_annotation> annotations;
    string annotations_file = annotated_segment_path + "/voxel_annotations.cereal";
    dataset_annotations::list_annotated_supervoxels(annotations, annotations_file, K, obr_segments_annotated);

    query_in_dataset_iterator query_data_iter(annotations, obr_segments_annotated);

    switch (option) {
    case 1:
        query_supervoxels(query_data_iter, obr_segments_noise, obr_segments_annotated, obr_scans_annotated, noise_scans_size, noise_segments_size, false);
        break;
    case 2:
        query_supervoxels(query_data_iter, obr_segments_noise, obr_segments_annotated, obr_scans_annotated, noise_scans_size, noise_segments_size, true);
        break;
    case 3:
        query_supervoxel_oversegments(query_data_iter, K, obr_segments_noise, obr_scans_noise, obr_segments_annotated, obr_scans_annotated, noise_scans_size, false);
        break;
    case 4:
        query_supervoxel_oversegments(query_data_iter, K, obr_segments_noise, obr_scans_noise, obr_segments_annotated, obr_scans_annotated, noise_scans_size, true);
        break;
    default:
        cout << "The option provided is not valid..." << endl;
    }

    return 0;
}
