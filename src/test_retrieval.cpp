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

struct query_separate_data_iterator {
    using vec = vector<boost::filesystem::path>;
    string base_path;
    vec v;
    int i;
    bool get_next_query(string& instance, CloudT::Ptr& cloud, HistCloudT::Ptr& features, CloudT::Ptr& keypoints, int& scan_id, int& segment_id)
    {
        if (i >= v.size()) {
            return false;
        }
        boost::filesystem::path f = v[i];
        ++i;
        if (!boost::filesystem::is_regular_file(f)) {
            return get_next_query(instance, cloud, features, keypoints, scan_id, segment_id);
        }
        string stem = f.stem().string();
        if (!isdigit(stem.back())) {
            return get_next_query(instance, cloud, features, keypoints, scan_id, segment_id);
        }
        string features_path = base_path + "/" + stem + "_features.pcd";
        string keypoints_path = base_path + "/" + stem + "_keypoints.pcd";
        //string features_path = base_path + "/" + stem + "_shot_features.pcd";
        //string keypoints_path = base_path + "/" + stem + "_shot_keypoints.pcd";

        int pos = stem.find_first_of("_");
        instance = stem.substr(0, pos+2);
        cout << "Instance: " << instance << endl;
        pcl::io::loadPCDFile(f.string(), *cloud);
        pcl::io::loadPCDFile(features_path, *features);
        pcl::io::loadPCDFile(keypoints_path, *keypoints);
        scan_id = -1;
        segment_id = -1;
    }
    query_separate_data_iterator(string base_path) : base_path(base_path), v(), i(0)
    {
        boost::filesystem::path p(base_path);
        copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), back_inserter(v));
        sort(v.begin(), v.end()); // sort, since directory iteration
        cout << "Opening path " << p.string() << endl;
    }
};

// OK
template <typename Iterator>
void query_supervoxel_oversegments(Iterator& query_iterator, Eigen::Matrix3f& K,
                                   object_retrieval& obr_segments, object_retrieval& obr_scans,
                                   object_retrieval& obr_segments_annotations, object_retrieval& obr_scans_annotations,
                                   int noise_scans_size)
{
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

    obr_segments.gvt.compute_leaf_vocabulary_vectors(); // this will not be needed eventually

    obr_segments.gvt.get_node_mapping(mapping);

    //save_oversegmented_grouped_vocabulary_index_vectors(obr_scans, obr_segments);
    //save_oversegmented_grouped_vocabulary_index_vectors(obr_scans_annotations, obr_segments);
    //exit(0);

    map<string, int> nbr_full_instances;
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
        reweight_query_vocabulary_sift(reweight_scores, second_scores, first_scores, hints, oversegment_indices, cloud, features, keypoints, segment_id, nbr_query,
                                       nbr_initial_query, obr_scans, obr_scans_annotations, obr_segments, obr_segments_annotations, noise_scans_size, mapping);


        //second_scores.resize(nbr_query);
        //dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(instance_correct_ratios, instance, scan_id, second_scores, obr_scans_annotations, noise_scans_size);
        //first_scores.resize(nbr_query);
        //dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(usual_correct_ratios, instance, scan_id, first_scores, obr_scans_annotations, noise_scans_size);
        reweight_scores.resize(nbr_query);
        dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(reweight_correct_ratios, instance, scan_id, reweight_scores, obr_scans_annotations, noise_scans_size);
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
void query_cloud(CloudT::Ptr& cloud, Eigen::Matrix3f& K, object_retrieval& obr_segments, object_retrieval& obr_scans,
                 object_retrieval& obr_segments_annotations, object_retrieval& obr_scans_annotations, int noise_scans_size)
{
    const int nbr_query = 10;
    const int nbr_reweight_query = 10;
    const int nbr_initial_query = 500;

    map<vocabulary_tree<HistT, 8>::node*, int> mapping;

    if (obr_segments.gvt.empty()) {
        obr_segments.read_vocabulary(obr_segments.gvt);
    }
    obr_segments.gvt.set_min_match_depth(3);
    obr_segments.gvt.compute_normalizing_constants();

    read_supervoxel_groups(obr_segments); // this will not be needed eventually, should be part of class init

    obr_segments.gvt.compute_leaf_vocabulary_vectors(); // this will not be needed eventually

    obr_segments.gvt.get_node_mapping(mapping);

    obr_segments.visualize_cloud(cloud);

    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT);
    pfhrgb_estimation::compute_features(features, keypoints, cloud, false);

    cout << "Features: " << features->size() << endl;

    vector<index_score> first_scores; // scores
    vector<index_score> second_scores; // updated_scores;
    vector<index_score> reweight_scores; // scores after reweighting
    vector<int> hints; // hints for where in image to start growing
    vector<vector<int> > oversegment_indices;
    find_top_oversegments_grow_and_score(first_scores, reweight_scores, hints, oversegment_indices, features, mapping, obr_segments,
                                         obr_scans, obr_scans_annotations, nbr_reweight_query, nbr_initial_query, noise_scans_size); // nbr_query if no reweight

    /*SiftCloudT::Ptr sift_features(new SiftCloudT);
    CloudT::Ptr sift_keypoints(new CloudT);
    compute_sift_features_for_query(sift_features, sift_keypoints, cloud, K);

    reweight_query_vocabulary_sift(reweight_scores, second_scores, first_scores, hints, oversegment_indices, cloud, features, keypoints, -1, nbr_query,
                                   nbr_initial_query, obr_scans, obr_scans_annotations, obr_segments, obr_segments_annotations, noise_scans_size, mapping,
                                   &(*sift_features), &(*sift_keypoints));*/

    cout << "Number of features: " << features->size() << endl;

    // make this a function
    for (size_t i = 0; i < reweight_scores.size(); ++i) {
        HistCloudT::Ptr result_features(new HistCloudT);
        CloudT::Ptr result_keypoints(new CloudT);
        if (reweight_scores[i].first < noise_scans_size) {
            load_nth_keypoints_features_for_scan(result_keypoints, result_features, reweight_scores[i].first, oversegment_indices[i], obr_scans);
        }
        else {
            load_nth_keypoints_features_for_scan(result_keypoints, result_features, reweight_scores[i].first-noise_scans_size, oversegment_indices[i], obr_scans_annotations);
        }
        CloudT::Ptr result_cloud(new CloudT);
        if (reweight_scores[i].first < noise_scans_size) {
            obr_scans.read_scan(result_cloud, reweight_scores[i].first);
        }
        else {
            obr_scans_annotations.read_scan(result_cloud, reweight_scores[i].first-noise_scans_size);
        }
        register_objects ro;
        ro.visualize_feature_segmentation(result_keypoints, result_cloud);
    }

}

// OK
template <typename Iterator>
void query_supervoxels(Iterator& query_iterator, object_retrieval& obr_segments, object_retrieval& obr_segments_annotations,
                       object_retrieval& obr_scans_annotations, int noise_scans_size, int noise_segments_size)
{
    const int nbr_query = 20; // 11
    const int nbr_reweight_query = 51;

    if (obr_segments.vt.empty()) {
        obr_segments.read_vocabulary(obr_segments.vt);
    }
    obr_segments.vt.set_min_match_depth(3); // 3 in experiments
    obr_segments.vt.compute_normalizing_constants();

    //map<string, int> nbr_full_instances;
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
        /*if (instance != "kinect_1" && instance != "ajax_1" && instance != "ajax_2") {
            continue;
        }*/
        vector<index_score> scores;
        //obr_segments.vt.top_combined_similarities(scores, features, nbr_reweight_query);
        obr_segments.vt.top_similarities(scores, features, nbr_reweight_query);

        vector<index_score> reweight_scores;
        reweight_query_vocabulary_sift(reweight_scores, scores, cloud, features, keypoints, segment_id, nbr_query,
                                       obr_segments, obr_segments_annotations, noise_segments_size);

        /*scores.resize(nbr_query);
        for (index_score& s : scores) {
            if (s.first < noise_segments_size) {
                s.first = 0;//scan_ind_for_segment(s.first, obr_segments);
            }
            else {
                s.first = scan_ind_for_segment(s.first-noise_segments_size, obr_segments_annotations) + noise_scans_size;
            }
        }*/

        reweight_scores.resize(nbr_query);
        for (index_score& s : reweight_scores) {
            if (s.first < noise_segments_size) {
                s.first = 0;//scan_ind_for_segment(s.first, obr_segments);
            }
            else {
                s.first = scan_ind_for_segment(s.first-noise_segments_size, obr_segments_annotations) + noise_scans_size;
            }
        }

        //dataset_annotations::calculate_correct_ratio(instance_correct_ratios, instance, scan_id, scores, obr_scans_annotations, noise_scans_size);
        //dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(instance_correct_ratios, instance, scan_id, scores, obr_scans_annotations, noise_scans_size);
        dataset_annotations::calculate_correct_ratio_exclude_sweep_precise(reweight_correct_ratios, instance, scan_id, reweight_scores, obr_scans_annotations, noise_scans_size);

        //int scan_ind = scan_ind_for_segment(a.segment_id, obr_segments_annotations);
        //calculate_correct_ratio(instance_correct_ratios, a, scan_ind, scores, obr_scans_annotations, noise_scans_size);
        //compute_decay_correct_ratios(decay_correct_ratios, intermediate_points, a, scan_ind, scores, obr_scans_annotations, nbr_query, noise_scans_size);

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
    // TODO: make the paths configurable

    string root_path = "/home/nbore/Data/Instances/";
    string scan_path = root_path + "scan_segments";
    string segment_path = root_path + "supervoxel_segments";

    string noise_root_path = "/home/nbore/Data/semantic_map/";
    string noise_scan_path = noise_root_path + "scan_segments";
    string noise_segment_path = noise_root_path + "supervoxel_segments";

    object_retrieval obr_scans(scan_path);
    obr_scans.segment_name = "scan";
    object_retrieval obr_segments(segment_path);

    object_retrieval obr_scans_noise(noise_scan_path);
    obr_scans_noise.segment_name = "scan";
    object_retrieval obr_segments_noise(noise_segment_path);

    // TODO: Delete from here

    //compute_and_save_segments(obr_scans);
    //compute_and_save_segments(obr_scans_noise);

    //save_pfhrgb_features_for_supervoxels(obr_segments);
    //save_pfhrgb_features_for_supervoxels(obr_segments_noise);
    //save_shot_features_for_supervoxels(obr_segments);
    //save_shot_features_for_supervoxels(obr_segments_noise);

    //save_split_features(obr_segments);
    //save_split_features(obr_segments_noise);
    //exit(0);

    // train using the noise segments
    //obr_segments_noise.train_grouped_vocabulary(12000, false);

    // TODO: add something like obr_segments_noise.get_scan_count()
    int noise_scans_size = 3526;
    //obr_segments_noise.add_others_to_grouped_vocabulary(10000, obr_segments, noise_scans_size);

    //obr_segments_noise.train_vocabulary_incremental(4000, false); // 12000
    int noise_segments_size = 63136;
    //obr_segments_noise.add_others_to_vocabulary(10000, obr_segments.segment_path, noise_segments_size);
    //exit(0);

    //save_sift_features(obr_scans);
    //save_sift_features(obr_scans_noise);
    //exit(0);

    Eigen::Matrix3f K;
    string matrix_file = root_path + "K.cereal";
    ifstream in(matrix_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(K);
    }

    vector<dataset_annotations::voxel_annotation> annotations;
    string annotations_file = segment_path + "/voxel_annotations.cereal";
    dataset_annotations::list_annotated_supervoxels(annotations, annotations_file, K, obr_segments);

    query_separate_data_iterator query_sep_iter("/home/nbore/Data/query_object_recorder");
    query_in_dataset_iterator query_data_iter(annotations, obr_segments);

    //query_supervoxel_oversegments(query_data_iter, K, obr_segments, obr_scans, obr_segments, obr_scans, 0);

    //query_supervoxel_oversegments(query_data_iter, K, obr_segments_noise, obr_scans_noise, obr_segments, obr_scans, noise_scans_size);

    //query_supervoxels(query_data_iter, obr_segments_noise, obr_segments, obr_scans, noise_scans_size, noise_segments_size);

    CloudT::Ptr query_cloud_larger(new CloudT);
    // this comes from clips_1/patrol_run_3/room_0/intermediate_cloud0015.pcd, everything from that sweep are excluded in results
    pcl::io::loadPCDFile("query_chair.pcd", *query_cloud_larger);

    query_cloud(query_cloud_larger, K, obr_segments_noise, obr_scans_noise, obr_segments, obr_scans, noise_scans_size);

    cout << "Program finished..." << endl;

    return 0;
}
