#include <iostream>
#include <chrono>

#include "object_3d_retrieval/object_retrieval.h"
#include "object_3d_retrieval/dataset_annotations.h"
#include "object_3d_retrieval/descriptor_config.h"
#include "object_3d_retrieval/retrieval_client.h"
#include "object_3d_retrieval/pfhrgb_estimation.h"
#include "object_3d_retrieval/register_objects.h"

#include "simple_xml_parser.h"
#include <tf_conversions/tf_eigen.h>

#include <cereal/archives/binary.hpp>
#include <eigen_cereal/eigen_cereal.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace retrieval_client;

void get_waypoint_position_for_scan(string& waypoint_id, Eigen::Matrix4f& T, int i, object_retrieval& obr)
{
    string scan_file = obr.get_scan_file(i);
    boost::filesystem::path scan_path = scan_file;
    string stem = scan_path.stem().string();
    size_t pos = stem.find_last_not_of("0123456789");
    int ind = stoi(stem.substr(pos+1));
    string xml_file = (scan_path.parent_path() / boost::filesystem::path("room.xml")).string();

    vector<string> xml_nodes_to_parse = {"RoomIntermediateCloud", "IntermediatePosition", "RoomStringId"};

    SimpleXMLParser<PointT> parser;
    SimpleXMLParser<PointT>::RoomData room = parser.loadRoomFromXML(xml_file, xml_nodes_to_parse);

    tf::StampedTransform st = room.vIntermediateRoomCloudTransforms[ind];
    Eigen::Affine3d e;
    tf::transformTFToEigen(st, e);

    T = e.matrix().cast<float>();
    waypoint_id = room.roomWaypointId;
}

void visualize_matches_in_map(vector<CloudT::Ptr>& matches)
{
    CloudT::Ptr map_cloud(new CloudT);
    pcl::io::loadPCDFile("/home/nbore/Data/full_cloud.pcd", *map_cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(map_cloud);
    viewer->addPointCloud<PointT>(map_cloud, rgb, "map cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    int counter = 0;
    for (CloudT::Ptr& match : matches) {
        /*for (PointT& p : match->points) {
            p.r = 255;
            p.g = 0;
            p.b = 0;
        }*/
        string cloud_name = string("match") + to_string(counter);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_match(match);
        viewer->addPointCloud<PointT>(match, rgb_match, cloud_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
        ++counter;
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
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
    vector<CloudT::Ptr> matches;
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
        string waypoint_id;
        Eigen::Matrix4f T;
        if (reweight_scores[i].first < noise_scans_size) {
            obr_scans.read_scan(result_cloud, reweight_scores[i].first);
            get_waypoint_position_for_scan(waypoint_id, T, reweight_scores[i].first, obr_scans);
        }
        else {
            obr_scans_annotations.read_scan(result_cloud, reweight_scores[i].first-noise_scans_size);
            get_waypoint_position_for_scan(waypoint_id, T, reweight_scores[i].first-noise_scans_size, obr_scans_annotations);
        }

        cout << "Found object at " << waypoint_id << endl;

        register_objects ro;
        //ro.visualize_feature_segmentation(result_keypoints, result_cloud);
        CloudT::Ptr segment(new CloudT);
        ro.get_feature_segmentation(segment, result_keypoints, result_cloud);
        for (PointT& p : segment->points) {
            p.r = 255;
            p.g = 0;
            p.b = 0;
        }
        *result_cloud += *segment;
        matches.push_back(CloudT::Ptr(new CloudT));
        pcl::transformPointCloud(*result_cloud, *matches.back(), T);
    }

    visualize_matches_in_map(matches);
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
