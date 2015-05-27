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

int main(int argc, char** argv)
{
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

    int noise_scans_size = 3526;

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

    CloudT::Ptr query_cloud_larger(new CloudT);
    // this comes from clips_1/patrol_run_3/room_0/intermediate_cloud0015.pcd, everything from that sweep are excluded in results
    pcl::io::loadPCDFile("query_chair.pcd", *query_cloud_larger);

    query_cloud(query_cloud_larger, K, obr_segments_noise, obr_scans_noise, obr_segments, obr_scans, noise_scans_size);

    cout << "Program finished..." << endl;

    return 0;
}
