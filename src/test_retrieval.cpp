#include <iostream>
#include <chrono>

#include <pcl/kdtree/kdtree_flann.h>

#include "object_3d_retrieval/supervoxel_segmentation.h"
#include "object_3d_retrieval/object_retrieval.h"
#include "object_3d_retrieval/dataset_convenience.h"
#include "object_3d_retrieval/register_objects.h"
#include "object_3d_retrieval/pfhrgb_estimation.h"
#include "object_3d_retrieval/dataset_annotations.h"
#include "object_3d_retrieval/descriptor_config.h"
#include "sift/sift.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <cereal/archives/binary.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/utility.hpp>
#include <eigen_cereal/eigen_cereal.h>

using namespace std;
using namespace dataset_convenience;

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<128>,
                                   (float[128], histogram, histogram)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<250>,
                                   (float[250], histogram, histogram)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<1344>,
                                   (float[1344], histogram, histogram)
)

using SiftT = pcl::Histogram<128>;
using SiftCloudT = pcl::PointCloud<SiftT>;
using PfhRgbT = pcl::Histogram<250>;
using PfhRgbCloudT = pcl::PointCloud<PfhRgbT>;
using ShotT = pcl::Histogram<1344>;
using ShotCloudT = pcl::PointCloud<ShotT>;

// OK
void save_sift_features(object_retrieval& obr_scans)
{
    for (int i = 0; ; ++i) {
        string folder = obr_scans.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(folder)) {
            cout << folder << " is not a directory!" << endl;
            break;
        }
        CloudT::Ptr cloud(new CloudT);
        obr_scans.read_scan(cloud, i);
        cv::Mat img(480, 640, CV_8UC3);
        for (int y = 0; y < 480; ++y) {
            for (int x = 0; x < 640; ++x) {
                int ind = y*640+x;
                cv::Vec3b& c = img.at<cv::Vec3b>(y, x);
                c[2] = cloud->at(ind).r;
                c[1] = cloud->at(ind).g;
                c[0] = cloud->at(ind).b;
            }
        }
        cv::FastFeatureDetector detector;
        std::vector<cv::KeyPoint> keypoints;
        detector.detect(img, keypoints);
        cv::SIFT::DescriptorParams descriptor_params;
        descriptor_params.isNormalize = true; // always true, shouldn't matter
        descriptor_params.magnification = 3.0; // 3.0 default
        descriptor_params.recalculateAngles = true; // true default
        cv::SiftDescriptorExtractor extractor;
        cv::Mat descriptors;
        extractor.compute(img, keypoints, descriptors);

        CloudT::Ptr kp_cloud(new CloudT);
        SiftCloudT::Ptr desc_cloud(new SiftCloudT);
        int j = 0;
        for (cv::KeyPoint k : keypoints) {
            cv::Point2f p2 = k.pt;
            int ind = p2.y*640+p2.x;
            PointT p = cloud->at(ind);
            if (pcl::isFinite(p)) {
                kp_cloud->push_back(p);
                SiftT sp;
                for (int k = 0; k < 128; ++k) {
                    sp.histogram[k] = descriptors.at<float>(j, k);
                }
                desc_cloud->push_back(sp);
            }
            ++j;
        }

        string sift_file = folder + "/sift_cloud.pcd";
        string sift_points_file = folder + "/sift_points_file.pcd";

        pcl::io::savePCDFileBinary(sift_file, *desc_cloud);
        pcl::io::savePCDFileBinary(sift_points_file, *kp_cloud);
    }
}

// OK
int scan_ind_for_segment(int i, object_retrieval& obr_segments)
{
    cout << i << endl;
    string segment_folder = obr_segments.get_folder_for_segment_id(i);
    cout << segment_folder << endl;
    string metadata_file = segment_folder + "/metadata.txt";
    string metadata; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(metadata_file);
        getline(f, metadata);
        f.close();
    }
    string scan_name = boost::filesystem::path(metadata).parent_path().stem().string();
    size_t pos = scan_name.find_last_not_of("0123456789");
    cout << scan_name << endl;
    int ind = stoi(scan_name.substr(pos+1));
    return ind;
}

// OK
void save_split_features(object_retrieval& obr_segments)
{
    for (int i = 0; ; ++i) {
        string segment_folder = obr_segments.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(segment_folder)) {
            break;
        }
        // we should add a keypoints argument to object_retrieval
        string pfhrgb_file = segment_folder + "/" + descriptor_config::feature_segment_file;
        string pfhrgb_points_file = segment_folder + "/" + descriptor_config::keypoint_segment_file;
        HistCloudT::Ptr desc_cloud(new HistCloudT);
        CloudT::Ptr kp_cloud(new CloudT);
        if (pcl::io::loadPCDFile(pfhrgb_file, *desc_cloud) == -1) {
            cout << "Could not load " << pfhrgb_file << endl;
            exit(0);
        }
        if (pcl::io::loadPCDFile(pfhrgb_points_file, *kp_cloud) == -1) {
            cout << "Could not load " << pfhrgb_points_file << endl;
            exit(0);
        }
        vector<HistCloudT::Ptr> split_features;
        vector<CloudT::Ptr> split_keypoints;
        pfhrgb_estimation::split_descriptor_points(split_features, split_keypoints, desc_cloud, kp_cloud, 30);
        cout << "Split features size: " << split_features.size() << endl;
        cout << "Split keypoint size: " << split_keypoints.size() << endl;
        int counter = 0;
        int j = 0;
        for (HistCloudT::Ptr& split_cloud : split_features) {
            if (split_cloud->empty()) {
                ++j;
                continue;
            }
            if (desc_cloud->size() >= 20 && split_cloud->size() < 20) {
                cout << "Doing cloud: " << pfhrgb_file << endl;
                cout << "Split cloud had size: " << split_cloud->size() << endl;
                exit(0);
            }
            cout << "Saving features and keypoints..." << endl;
            string split_file = segment_folder + "/" + descriptor_config::split_feature_stem + to_string(counter) + ".pcd";
            string split_points_file = segment_folder + "/" + descriptor_config::split_keypoints_stem + to_string(counter) + ".pcd";
            pcl::io::savePCDFileBinary(split_file, *split_cloud);
            pcl::io::savePCDFileBinary(split_points_file, *split_keypoints[j]);
            cout << "Done saving..." << endl;
            ++j;
            ++counter;
        }
    }
}

// OK
void save_pfhrgb_features_for_supervoxels(object_retrieval& obr_segments)
{
    for (int i = 0; ; ++i) {
        string segment_folder = obr_segments.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(segment_folder)) {
            break;
        }
        string cloud_file = segment_folder + "/segment.pcd";
        CloudT::Ptr cloud(new CloudT);
        if (pcl::io::loadPCDFile(cloud_file, *cloud) == -1) {
            cout << "Could not load " << cloud_file << endl;
            exit(0);
        }
        PfhRgbCloudT::Ptr desc_cloud(new PfhRgbCloudT);
        CloudT::Ptr kp_cloud(new CloudT);
        pfhrgb_estimation::compute_features(desc_cloud, kp_cloud, cloud, false);

        string pfhrgb_file = segment_folder + "/pfhrgb_cloud.pcd";
        string pfhrgb_points_file = segment_folder + "/pfhrgb_points_file.pcd";

        if (desc_cloud->empty()) {
            // push back one inf point on descriptors and keypoints
            PfhRgbT sp;
            for (int i = 0; i < 250; ++i) {
                sp.histogram[i] = std::numeric_limits<float>::infinity();
            }
            desc_cloud->push_back(sp);
            PointT p;
            p.x = p.y = p.z = std::numeric_limits<float>::infinity();
            kp_cloud->push_back(p);
        }

        pcl::io::savePCDFileBinary(pfhrgb_file, *desc_cloud);
        pcl::io::savePCDFileBinary(pfhrgb_points_file, *kp_cloud);
    }
}

// OK
void save_shot_features_for_supervoxels(object_retrieval& obr_segments)
{
    for (int i = 0; ; ++i) {
        string segment_folder = obr_segments.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(segment_folder)) {
            break;
        }
        string cloud_file = segment_folder + "/segment.pcd";
        CloudT::Ptr cloud(new CloudT);
        if (pcl::io::loadPCDFile(cloud_file, *cloud) == -1) {
            cout << "Could not load " << cloud_file << endl;
            exit(0);
        }
        ShotCloudT::Ptr desc_cloud(new ShotCloudT);
        CloudT::Ptr kp_cloud(new CloudT);
        pfhrgb_estimation::compute_features(desc_cloud, kp_cloud, cloud, false);

        string shot_file = segment_folder + "/shot_cloud.pcd";
        string shot_points_file = segment_folder + "/shot_points_file.pcd";

        if (desc_cloud->empty()) {
            // push back one inf point on descriptors and keypoints
            ShotT sp;
            for (int i = 0; i < 1344; ++i) {
                sp.histogram[i] = std::numeric_limits<float>::infinity();
            }
            desc_cloud->push_back(sp);
            PointT p;
            p.x = p.y = p.z = std::numeric_limits<float>::infinity();
            kp_cloud->push_back(p);
        }

        pcl::io::savePCDFileBinary(shot_file, *desc_cloud);
        pcl::io::savePCDFileBinary(shot_points_file, *kp_cloud);
    }
}

// OK
void compute_and_save_segments(object_retrieval& obr_scans)
{
    using Graph = supervoxel_segmentation::Graph;

    string base_path = boost::filesystem::path(obr_scans.segment_path).parent_path().string();
    string segment_path = base_path + "/supervoxel_segments";
    boost::filesystem::create_directory(segment_path);

    int counter = 0;
    for (int i = 0; ; ++i) {
        string folder = obr_scans.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(folder)) {
            return;
        }
        CloudT::Ptr cloud(new CloudT);
        string scan_file = obr_scans.get_scan_file(i);

        cout << "Finally got scan " << scan_file << endl;
        obr_scans.read_scan(cloud, i);
        //obr.visualize_cloud(cloud);
        string graph_file = folder + "/graph.cereal";

        supervoxel_segmentation ss;
        vector<CloudT::Ptr> supervoxels;
        //ss.create_supervoxel_graph(supervoxels, cloud);
        Graph* g = ss.compute_convex_oversegmentation(supervoxels, cloud);
        ss.save_graph(*g, graph_file);
        delete g;
        vector<string> segment_folders;
        int j = 0;
        for (CloudT::Ptr& sv : supervoxels) {
            cout << "Segment size: " << sv->size() << endl;
            if (sv->empty()) {
                ++j;
                continue;
            }

            string segment_folder = segment_path + "/segment" + to_string(counter);
            boost::filesystem::create_directory(segment_folder);
            segment_folders.push_back(segment_folder);

            string cloud_file = segment_folder + "/segment.pcd";
            pcl::io::savePCDFileBinary(cloud_file, *sv);

            string metadata_file = segment_folder + "/metadata.txt";
            {
                ofstream f;
                f.open(metadata_file);
                f << folder << "\n";
                f << scan_file << "\n";
                f << "Vertex nbr: " << j << "\n";
                f.close();
            }
            ++counter;
            ++j;
        }

        string segment_folders_file = folder + "/segment_paths.txt";
        {
            ofstream f;
            f.open(segment_folders_file);
            for (string& sf : segment_folders) {
                f << sf << "\n";
            }
            f.close();
        }
    }
}

// OK
void get_voxel_vectors_for_scan(CloudT::Ptr& voxel_centers, vector<double>& vocabulary_norms, vector<map<int, double> >& vocabulary_vectors,
                                vector<map<int, int> >& vocabulary_index_vectors, int i, object_retrieval& obr_scans)
{
    string scan_path = obr_scans.get_folder_for_segment_id(i);

    string centers_file = scan_path + "/" + descriptor_config::scan_split_centers;
    if (pcl::io::loadPCDFile(centers_file, *voxel_centers) == -1) {
        cout << "Could not read file " << centers_file << endl;
        exit(0);
    }

    string norms_file = scan_path + "/" + descriptor_config::scan_vocabulary_norms;
    ifstream inn(norms_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(inn);
        archive_i(vocabulary_norms);
    }

    string vectors_file = scan_path + "/" + descriptor_config::scan_vocabulary_vectors;
    ifstream inv(vectors_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(inv);
        archive_i(vocabulary_vectors);
    }

    string index_vectors_file = scan_path + "/" + descriptor_config::scan_vocabulary_index_vectors;
    ifstream iniv(index_vectors_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(iniv);
        archive_i(vocabulary_index_vectors);
    }
}

// OK
void compute_voxel_centers(CloudT::Ptr& center_cloud, vector<CloudT::Ptr>& voxel_points, int min_features)
{
    for (CloudT::Ptr& cloud : voxel_points) {
        if (cloud->size() < min_features) {
            continue;
        }
        Eigen::Vector3f center;
        bool finite = false;
        int counter = 0;
        for (PointT& p : cloud->points) {
            if (!pcl::isFinite(p)) {
                continue;
            }
            finite = true;
            center += eig(p);
            ++counter;
        }
        if (counter > 0) {
            center *= 1.0f/float(counter);
        }

        PointT p;
        if (!finite) {
            p.x = std::numeric_limits<float>::infinity();
            p.y = std::numeric_limits<float>::infinity();
            p.z = std::numeric_limits<float>::infinity();
        }
        else {
            p.x = center(0);
            p.y = center(1);
            p.z = center(2);
        }
        center_cloud->push_back(p);
    }
}

// OK
void get_voxels_and_points_for_scan(vector<HistCloudT::Ptr>& voxels, vector<CloudT::Ptr>& voxel_points, int i, object_retrieval& obr_scans)
{
    string scan_path = obr_scans.get_folder_for_segment_id(i);
    string segment_paths_file = scan_path + "/segment_paths.txt";
    vector<string> voxel_paths;
    {
        ifstream f;
        string metadata;
        f.open(segment_paths_file);
        while (getline(f, metadata)) {
            voxel_paths.push_back(metadata);
        }
    }
    for (const string& voxel_path : voxel_paths) {
        for (int i = 0; ; ++i) {
            string oversegment_path = voxel_path + "/" + descriptor_config::split_feature_stem + to_string(i) + ".pcd";
            if (!boost::filesystem::is_regular_file(oversegment_path)) {
                break;
            }
            voxels.push_back(HistCloudT::Ptr(new HistCloudT));
            if (pcl::io::loadPCDFile(oversegment_path, *voxels.back()) == -1) {
                cout << "Could not read oversegment file " << oversegment_path << endl;
                exit(0);
            }

            string points_path = voxel_path + "/" + descriptor_config::split_keypoints_stem + to_string(i) + ".pcd";
            if (!boost::filesystem::is_regular_file(points_path)) {
                cout << "There was a features file but no points file..." << endl;
                exit(0);
            }
            voxel_points.push_back(CloudT::Ptr(new CloudT));
            if (pcl::io::loadPCDFile(points_path, *voxel_points.back()) == -1) {
                cout << "Could not read oversegment points file " << points_path << endl;
                exit(0);
            }
        }
    }
}

// OK
void save_oversegmented_grouped_vocabulary_index_vectors(object_retrieval& obr_scans, object_retrieval& obr_segments)
{
    int min_features = 20;

    map<vocabulary_tree<HistT, 8>::node*, int> mapping;
    obr_segments.gvt.get_node_mapping(mapping);

    for (int i = 0; ; ++i) {
        string path = obr_scans.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(path)) {
            break;
        }
        vector<HistCloudT::Ptr> voxels;
        vector<CloudT::Ptr> voxel_points;
        get_voxels_and_points_for_scan(voxels, voxel_points, i, obr_scans);

        vector<map<int, double> > vocabulary_vectors;
        vector<map<int, int> > vocabulary_index_vectors;
        vector<double> vocabulary_norms;

        for (HistCloudT::Ptr& voxel : voxels) {
            if (voxel->size() < min_features) {
                continue;
            }
            vocabulary_vectors.push_back(map<int, double>());
            double pnorm = obr_segments.gvt.compute_query_index_vector(vocabulary_vectors.back(), voxel, mapping);
            vocabulary_norms.push_back(pnorm);
            vocabulary_index_vectors.push_back(map<int, int>());
            obr_segments.gvt.compute_query_index_vector(vocabulary_index_vectors.back(), voxel, mapping);
        }
        string vectors_file = path + "/" + descriptor_config::scan_vocabulary_vectors;
        ofstream outv(vectors_file, std::ios::binary);
        {
            cereal::BinaryOutputArchive archive_o(outv);
            archive_o(vocabulary_vectors);
        }
        string norms_file = path + "/" + descriptor_config::scan_vocabulary_norms;
        ofstream outn(norms_file, std::ios::binary);
        {
            cereal::BinaryOutputArchive archive_o(outn);
            archive_o(vocabulary_norms);
        }
        string index_vectors_file = path + "/" + descriptor_config::scan_vocabulary_index_vectors;
        ofstream outiv(index_vectors_file, std::ios::binary);
        {
            cereal::BinaryOutputArchive archive_o(outiv);
            archive_o(vocabulary_index_vectors);
        }

        CloudT::Ptr centers_cloud(new CloudT);
        compute_voxel_centers(centers_cloud, voxel_points, min_features);
        string centers_file = path + "/" + descriptor_config::scan_split_centers;
        pcl::io::savePCDFileBinary(centers_file, *centers_cloud);
    }
}

// OK
void change_supervoxel_groups(object_retrieval& obr_voxels)
{
    map<int, int> number_features_in_group;

    int current_ind = -1;
    int current_offset = 0;
    map<int, int> temp(obr_voxels.gvt.index_group.begin(), obr_voxels.gvt.index_group.end());
    int nbr_groups = 0;
    int counter = 0;

    for (pair<const int, int>& p : temp) {
        //cout << p.second << endl;
        int scan_ind = scan_ind_for_segment(p.second, obr_voxels); // the second is the actual segment
        number_features_in_group[scan_ind] += 1;
        if (scan_ind != current_ind) {
            current_ind = scan_ind;
            current_offset = p.first; // this is the first oversegmented in the group
            ++nbr_groups;
            counter = 0;
        }
        obr_voxels.gvt.group_subgroup.insert(make_pair(p.first, make_pair(scan_ind, counter)));//p.first-current_offset)));
        ++counter;
    }

    vector<pair<int, int> > max_features_in_group;
    max_features_in_group.insert(max_features_in_group.end(), number_features_in_group.begin(), number_features_in_group.end());
    std::sort(max_features_in_group.begin(), max_features_in_group.end(), [](const pair<int, int>& p1, const pair<int, int>& p2) {
       return  p1.second < p2.second;
    });

    for (int i = 0; i < 1000; ++i) {
        cout << max_features_in_group[i].first << ": " << max_features_in_group[i].second << endl;
    }

    cout << "number of scans: " << number_features_in_group.size() << endl;
    cout << "number of supervoxels: " << obr_voxels.gvt.index_group.size() << endl;
    cout << "deep count supervoxels: " << obr_voxels.gvt.deep_count_sets() << endl;
    cout << "number of groups: " << nbr_groups << endl;

    string root_path = obr_voxels.segment_path;
    string group_file = root_path + "/" + descriptor_config::grouped_associations_file;
    ofstream out(group_file, std::ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(obr_voxels.gvt.group_subgroup);
    }

}

// OK
void read_supervoxel_groups(object_retrieval& obr_voxels)
{
    string root_path = obr_voxels.segment_path;
    string group_file = root_path + "/" + descriptor_config::grouped_associations_file;
    ifstream in(group_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(obr_voxels.gvt.group_subgroup);
    }
}

// OK
void reweight_query_vocabulary(vector<tuple<int, int, double> >& reweighted_scores, object_retrieval& obr_segments,
                               object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, const Eigen::Matrix3f& K,
                               vector<tuple<int, int, double> >& tuple_scores, int noise_scans_size, HistCloudT::Ptr& query_features,
                               CloudT::Ptr& query_cloud, int nbr_initial_query)
{
    int nbr_reweight_query = 10;

    map<int, double> original_norm_constants;
    map<vocabulary_tree<HistT, 8>::node*, double> original_weights;
    map<int, double> weighted_indices;

    int counter = 0;
    double weight_sum = 0.0;
    for (tuple<int, int, double>& t : tuple_scores) {
        if (counter > nbr_reweight_query) {
            break;
        }
        CloudT::Ptr cloud(new CloudT);
        if (get<0>(t) < noise_scans_size) {
            obr_scans.read_scan(cloud, get<0>(t));
        }
        else {
            obr_scans_annotations.read_scan(cloud, get<0>(t) - noise_scans_size);
        }
        register_objects ro;
        ro.set_input_clouds(query_cloud, K, cloud, K);
        ro.do_registration();
        pair<double, double> match_score = ro.get_match_score();
        if (std::isnan(match_score.first)) {
            continue;
        }
        int vocab_ind = obr_segments.gvt.get_id_for_group_subgroup(get<0>(t), get<1>(t));
        weighted_indices.insert(make_pair(vocab_ind, match_score.first));
        weight_sum += match_score.first;
        ++counter;
    }

    for (pair<const int, double>& w : weighted_indices) {
        w.second *= 1.0*double(weighted_indices.size())/weight_sum;
    }

    obr_segments.gvt.compute_new_weights(original_norm_constants, original_weights, weighted_indices, query_features);

    obr_segments.gvt.top_optimized_similarities(reweighted_scores, query_features, nbr_initial_query);

    obr_segments.gvt.restore_old_weights(original_norm_constants, original_weights);
}

// OK
void load_nth_keypoints_features_for_scan(CloudT::Ptr& keypoints, HistCloudT::Ptr& features,
                                          int i, vector<int>& indices, object_retrieval& obr_scans)
{
    int min_features = 20; // this should be a global parameter

    string path = obr_scans.get_folder_for_segment_id(i);
    if (!boost::filesystem::is_directory(path)) {
        cout << "Scan path was not a directory!" << endl;
        exit(0);
    }
    vector<HistCloudT::Ptr> voxels;
    vector<CloudT::Ptr> voxel_points;
    get_voxels_and_points_for_scan(voxels, voxel_points, i, obr_scans);

    int counter = 0;
    for (int i = 0; i < voxels.size(); ++i) {
        if (voxels[i]->size() < min_features) {
            continue;
        }
        if (std::find(indices.begin(), indices.end(), counter) == indices.end()) {
            ++counter;
            continue;
        }
        *keypoints += *voxel_points[i];
        *features += *voxels[i];
        ++counter;
    }
}

// OK
template <typename T, typename Compare>
vector<int> sort_permutation(vector<T> const& vec, Compare compare)
{
    vector<int> p(vec.size());
    std::iota(p.begin(), p.end(), 0);
    std::sort(p.begin(), p.end(), [&](int i, int j){ return compare(vec[i], vec[j]); });
    return p;
}

// OK
template <typename T>
vector<T> apply_permutation(vector<T> const& vec, vector<int> const& p)
{
    vector<T> sorted_vec(p.size());
    std::transform(p.begin(), p.end(), sorted_vec.begin(), [&](int i){ return vec[i]; });
    return sorted_vec;
}

// OK
void compute_grown_segment_score(vector<double>& match_scores, HistCloudT::Ptr& query_features, CloudT::Ptr& query_keypoints, CloudT::Ptr& query_cloud,
                                 vector<index_score>& updated_scores, vector<vector<int> >& oversegment_indices,
                                 object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, int noise_scans_size)
{
    for (size_t i = 0; i < updated_scores.size(); ++i) {
        HistCloudT::Ptr result_features(new HistCloudT);
        CloudT::Ptr result_keypoints(new CloudT);
        if (updated_scores[i].first < noise_scans_size) {
            load_nth_keypoints_features_for_scan(result_keypoints, result_features, updated_scores[i].first, oversegment_indices[i], obr_scans);
        }
        else {
            load_nth_keypoints_features_for_scan(result_keypoints, result_features, updated_scores[i].first-noise_scans_size, oversegment_indices[i], obr_scans_annotations);
        }
        CloudT::Ptr result_cloud(new CloudT);
        if (updated_scores[i].first < noise_scans_size) {
            obr_scans.read_scan(result_cloud, updated_scores[i].first);
        }
        else {
            obr_scans_annotations.read_scan(result_cloud, updated_scores[i].first-noise_scans_size);
        }
        register_objects ro;
        ro.set_input_clouds(query_cloud, result_cloud);
        //ro.register_using_features(query_features, query_keypoints, result_features, result_keypoints);
        pair<double, double> match_score = ro.get_match_score();
        match_scores.push_back(match_score.first);
    }
}

// OK
void compute_grow_subsegment_scores(vector<index_score>& updated_scores, vector<vector<int> >& oversegment_indices, vector<index_score>& scores,
                                    vector<int>& hints, HistCloudT::Ptr& features, map<vocabulary_tree<HistT, 8>::node*, int>& mapping, object_retrieval& obr_segments,
                                    object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, int nbr_query, int noise_scans_size)
{
    map<int, vocabulary_tree<HistT, 8>::node*> inverse_mapping;
    for (const pair<vocabulary_tree<HistT, 8>::node*, int>& u : mapping) {
        inverse_mapping.insert(make_pair(u.second, u.first));
    }

    //vector<index_score> total_scores;
    for (size_t i = 0; i < scores.size(); ++i) {
        CloudT::Ptr voxel_centers(new CloudT);
        vector<double> vocabulary_norms;
        vector<map<int, double> > vocabulary_vectors;
        vector<map<int, int> > vocabulary_index_vectors;

        if (scores[i].first < noise_scans_size) {
            get_voxel_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, vocabulary_index_vectors, scores[i].first, obr_scans);
        }
        else {
            get_voxel_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, vocabulary_index_vectors, scores[i].first-noise_scans_size, obr_scans_annotations);
        }

        vector<int> selected_indices;
        //double score = obr_segments.gvt.compute_min_combined_dist(selected_indices, features, vocabulary_vectors, vocabulary_norms, voxel_centers, mapping, hints[i]);
        double score = obr_segments.gvt.compute_min_combined_dist(selected_indices, features, vocabulary_index_vectors, voxel_centers, mapping, inverse_mapping, hints[i]);
        updated_scores.push_back(index_score(scores[i].first, score));
        oversegment_indices.push_back(selected_indices);
    }

    auto p = sort_permutation(updated_scores, [](const index_score& s1, const index_score& s2) {
        return s1.second < s2.second; // find min elements!
    });
    updated_scores = apply_permutation(updated_scores, p);
    oversegment_indices = apply_permutation(oversegment_indices, p);
    updated_scores.resize(nbr_query);
    oversegment_indices.resize(nbr_query);
}

// OK
void find_top_oversegments_grow_and_score(vector<index_score>& first_scores, vector<index_score>& second_scores, vector<int>& hints, vector<vector<int> >& oversegment_indices,
                                          HistCloudT::Ptr& features, map<vocabulary_tree<HistT, 8>::node*, int>& mapping, object_retrieval& obr_segments,
                                          object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, int nbr_query, int nbr_initial_query, int noise_scans_size)
{
    vector<tuple<int, int, double> > tuple_scores;
    obr_segments.gvt.top_optimized_similarities(tuple_scores, features, nbr_initial_query);

    // also, grow and reweight
    for (const tuple<int, int, double>& t : tuple_scores) {
        first_scores.push_back(index_score(get<0>(t), get<2>(t)));
        hints.push_back(get<1>(t));
    }

    // this function call indicates that we need some better abstractions
    compute_grow_subsegment_scores(second_scores, oversegment_indices, first_scores, hints, features, mapping,
                                   obr_segments, obr_scans, obr_scans_annotations, nbr_query, noise_scans_size);
}

// OK
void get_sift_features_for_segment(SiftCloudT::Ptr& sift_cloud, CloudT::Ptr& sift_keypoints, CloudT::Ptr& keypoints, const string& scan_folder)
{
    string sift_features_file = scan_folder + "/sift_cloud.pcd";
    string sift_keypoints_file = scan_folder + "/sift_points_file.pcd";

    SiftCloudT::Ptr sift_scan_cloud(new SiftCloudT);
    CloudT::Ptr keypoints_scan_cloud(new CloudT);
    if (pcl::io::loadPCDFile(sift_features_file, *sift_scan_cloud) == -1) {
        cout << "Could not read file " << sift_features_file << endl;
        exit(-1);
    }
    if (pcl::io::loadPCDFile(sift_keypoints_file, *keypoints_scan_cloud) == -1) {
        cout << "Could not read file " << sift_keypoints_file << endl;
        exit(-1);
    }

    // pick all the sift keypoints close enough to a point in keypoints
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(keypoints);

    vector<int> indices;
    vector<float> distances;
    int counter = 0;
    for (const PointT& p : keypoints_scan_cloud->points) {
        if (!pcl::isFinite(p)) {
            ++counter;
            continue;
        }
        indices.clear();
        distances.clear();
        kdtree.nearestKSearchT(p, 1, indices, distances);
        if (distances.empty()) {
            cout << "Distances empty, wtf??" << endl;
            exit(0);
        }
        float dist = sqrt(distances[0]);
        if (dist < 0.05) {
            sift_keypoints->push_back(p);
            sift_cloud->push_back(sift_scan_cloud->at(counter));
        }
        ++counter;
    }
}

// OK
void reweight_query_vocabulary_sift(vector<index_score>& second_scores, vector<index_score>& first_scores, CloudT::Ptr& query_cloud,
                                    HistCloudT::Ptr& query_features, CloudT::Ptr& query_keypoints, int query_id, int nbr_query,
                                    object_retrieval& obr_segments, object_retrieval& obr_segments_annotations, int noise_segments_size)
{
    // get sift cloud and keypoints for query cloud
    SiftCloudT::Ptr sift_cloud(new SiftCloudT);
    CloudT::Ptr sift_keypoints(new CloudT);
    get_sift_features_for_segment(sift_cloud, sift_keypoints, query_keypoints, obr_segments_annotations.get_scan_folder_for_segment(query_id));

    map<int, double> weighted_indices;

    double weight_sum = 0.0;
    for (index_score& s : first_scores) {
        //CloudT::Ptr match_scan(new CloudT);
        HistCloudT::Ptr match_features(new HistCloudT);
        CloudT::Ptr match_keypoints(new CloudT);
        CloudT::Ptr match_sift_keypoints(new CloudT);
        SiftCloudT::Ptr match_sift_cloud(new SiftCloudT);
        if (s.first < noise_segments_size) {
            obr_segments.load_features_for_segment(match_features, match_keypoints, s.first);
            get_sift_features_for_segment(match_sift_cloud, match_sift_keypoints, match_keypoints, obr_segments.get_scan_folder_for_segment(s.first));
            //obr_segments.read_scan_for_segment(match_scan, s.first);
        }
        else {
            obr_segments_annotations.load_features_for_segment(match_features, match_keypoints, s.first-noise_segments_size);
            get_sift_features_for_segment(match_sift_cloud, match_sift_keypoints, match_keypoints,
                                          obr_segments_annotations.get_scan_folder_for_segment(s.first-noise_segments_size));
            //obr_segments_annotations.read_scan_for_segment(match_scan, s.first-noise_segments_size);
        }
        register_objects ro;
        ro.set_input_clouds(sift_keypoints, match_sift_keypoints);
        ro.do_registration(sift_cloud, match_sift_cloud, sift_keypoints, match_sift_keypoints);
        double spatial_score, color_score;
        tie(spatial_score, color_score) = ro.get_match_score();
        if (std::isinf(spatial_score)) {
            continue;
        }
        weighted_indices.insert(make_pair(s.first, spatial_score));
        weight_sum += spatial_score;
    }

    for (pair<const int, double>& w : weighted_indices) {
        w.second *= 1.0*double(weighted_indices.size())/weight_sum;
    }

    map<int, double> original_norm_constants;
    map<vocabulary_tree<HistT, 8>::node*, double> original_weights;

    obr_segments.vt.compute_new_weights(original_norm_constants, original_weights, weighted_indices, query_features);
    obr_segments.vt.top_combined_similarities(second_scores, query_features, nbr_query);
    obr_segments.vt.restore_old_weights(original_norm_constants, original_weights);
}

// OK
void reweight_query_vocabulary_sift(vector<index_score>& reweight_grown_scores, vector<index_score>& first_grown_scores,
                                    vector<index_score>& first_scores, vector<int>& hints, vector<vector<int> >& oversegment_indices, CloudT::Ptr& query_cloud,
                                    HistCloudT::Ptr& query_features, CloudT::Ptr& query_keypoints, int query_id, int nbr_query, int nbr_initial_query,
                                    object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, object_retrieval& obr_segments,
                                    object_retrieval& obr_segments_annotations, int noise_scans_size, map<vocabulary_tree<HistT, 8>::node*, int>& mapping)
{
    // get sift cloud and keypoints for query cloud
    SiftCloudT::Ptr sift_cloud(new SiftCloudT);
    CloudT::Ptr sift_keypoints(new CloudT);
    get_sift_features_for_segment(sift_cloud, sift_keypoints, query_keypoints, obr_segments_annotations.get_scan_folder_for_segment(query_id));
    int scan_id = obr_segments_annotations.scan_ind_for_segment(query_id);

    map<int, double> weighted_indices;

    double weight_sum = 0.0;
    int counter = 0;
    for (index_score& s : first_grown_scores) {
        if (s.first > noise_scans_size && s.first - noise_scans_size == scan_id) {
            ++counter;
            continue;
        }
        //CloudT::Ptr match_scan(new CloudT);
        HistCloudT::Ptr match_features(new HistCloudT);
        CloudT::Ptr match_keypoints(new CloudT);
        CloudT::Ptr match_sift_keypoints(new CloudT);
        SiftCloudT::Ptr match_sift_cloud(new SiftCloudT);
        if (s.first < noise_scans_size) {
            load_nth_keypoints_features_for_scan(match_keypoints, match_features, s.first, oversegment_indices[counter], obr_scans);
            get_sift_features_for_segment(match_sift_cloud, match_sift_keypoints, match_keypoints, obr_scans.get_folder_for_segment_id(s.first));
            //obr_scans.read_scan(match_scan, s.first);
        }
        else {
            load_nth_keypoints_features_for_scan(match_keypoints, match_features, s.first-noise_scans_size, oversegment_indices[counter], obr_scans_annotations);
            get_sift_features_for_segment(match_sift_cloud, match_sift_keypoints, match_keypoints,
                                          obr_scans_annotations.get_folder_for_segment_id(s.first-noise_scans_size));
            //obr_scans_annotations.read_scan(match_scan, s.first-noise_scans_size);
        }
        register_objects ro;
        ro.set_input_clouds(sift_keypoints, match_sift_keypoints);
        //ro.set_input_clouds(query_cloud, match_scan);
        ro.do_registration(sift_cloud, match_sift_cloud, sift_keypoints, match_sift_keypoints);
        double spatial_score, color_score;
        tie(spatial_score, color_score) = ro.get_match_score();
        if (std::isinf(spatial_score)) {
            ++counter;
            continue;
        }
        for (int i : oversegment_indices[counter]) {
            int voc_index = obr_segments.gvt.get_id_for_group_subgroup(s.first, i);
            weighted_indices.insert(make_pair(voc_index, spatial_score));
            weight_sum += spatial_score;
        }
        ++counter;
    }

    for (pair<const int, double>& w : weighted_indices) {
        w.second *= 1.0*double(weighted_indices.size())/weight_sum;
    }

    map<int, double> original_norm_constants;
    map<vocabulary_tree<HistT, 8>::node*, double> original_weights;

    //vector<vector<int> > reweighted_oversegment_indices;
    //vector<index_score> reweighted_oversegment_scores;

    obr_segments.gvt.compute_new_weights(original_norm_constants, original_weights, weighted_indices, query_features);
    //find_top_oversegments_grow_and_score(reweighted_oversegment_scores, second_scores, reweighted_oversegment_indices, query_features, mapping, obr_segments,
    //                                     obr_scans, obr_scans_annotations, nbr_query, nbr_initial_query, noise_scans_size);

    vector<tuple<int, int, double> > tuple_scores;
    obr_segments.gvt.top_optimized_similarities(tuple_scores, query_features, nbr_initial_query);
    vector<index_score> reweight_scores;
    vector<int> reweight_hints;
    for (const tuple<int, int, double>& t : tuple_scores) {
        reweight_scores.push_back(index_score(get<0>(t), get<2>(t)));
        reweight_hints.push_back(get<1>(t));
    }

    vector<vector<int> > reweight_oversegment_indices;
    //compute_grow_subsegment_scores(reweight_grown_scores, reweight_oversegment_indices, first_scores, hints, query_features, mapping,
    //                               obr_segments, obr_scans, obr_scans_annotations, nbr_query, noise_scans_size);
    compute_grow_subsegment_scores(reweight_grown_scores, reweight_oversegment_indices, reweight_scores, reweight_hints, query_features, mapping,
                                   obr_segments, obr_scans, obr_scans_annotations, nbr_query, noise_scans_size);
    obr_segments.gvt.restore_old_weights(original_norm_constants, original_weights);
}

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
    const int nbr_query = 15;
    const int nbr_reweight_query = 30;
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


        second_scores.resize(nbr_query);
        dataset_annotations::calculate_correct_ratio_exclude_sweep(instance_correct_ratios, instance, scan_id, second_scores, obr_scans_annotations, noise_scans_size);
        first_scores.resize(nbr_query);
        dataset_annotations::calculate_correct_ratio_exclude_sweep(usual_correct_ratios, instance, scan_id, first_scores, obr_scans_annotations, noise_scans_size);
        reweight_scores.resize(nbr_query);
        dataset_annotations::calculate_correct_ratio_exclude_sweep(reweight_correct_ratios, instance, scan_id, reweight_scores, obr_scans_annotations, noise_scans_size);
        cout << "Number of features: " << features->size() << endl;

        instance_mean_features[instance].first += features->size();
        instance_mean_features[instance].second += 1;
    }

    end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = end-start;

    cout << "Benchmark took " << elapsed_seconds.count() << " seconds" << endl;

    cout << "First round correct ratios: " << endl;
    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
        cout << "Mean features: " << float(instance_mean_features[c.first].first)/float(instance_mean_features[c.first].second) << endl;
        cout << "Number of queries: " << instance_number_queries[c.first] << endl;
    }

    for (pair<const string, pair<float, int> > c : usual_correct_ratios) {
        cout << c.first << " usual correct ratio: " << c.second.first/float(c.second.second) << endl;
    }

    for (pair<const string, pair<float, int> > c : reweight_correct_ratios) {
        cout << c.first << " reweight correct ratio: " << c.second.first/float(c.second.second) << endl;
    }
}

// OK
void query_cloud(CloudT::Ptr& cloud, object_retrieval& obr_segments, object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, int noise_scans_size)
{
    const int nbr_query = 11;
    const int nbr_initial_query = 500;

    map<vocabulary_tree<HistT, 8>::node*, int> mapping;

    if (obr_segments.gvt.empty()) {
        obr_segments.read_vocabulary(obr_segments.gvt);
    }
    obr_segments.gvt.set_min_match_depth(2);
    obr_segments.gvt.compute_normalizing_constants();

    read_supervoxel_groups(obr_segments); // this will not be needed eventually, should be part of class init

    obr_segments.gvt.compute_leaf_vocabulary_vectors(); // this will not be needed eventually

    obr_segments.gvt.get_node_mapping(mapping);

    obr_segments.visualize_cloud(cloud);

    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr kp_cloud(new CloudT);
    pfhrgb_estimation::compute_features(features, kp_cloud, cloud, false);

    vector<tuple<int, int, double> > tuple_scores;
    // this also takes care of the scan association
    obr_segments.gvt.top_optimized_similarities(tuple_scores, features, nbr_initial_query);

    vector<index_score> scores;
    vector<int> hints;
    for (const tuple<int, int, double>& t : tuple_scores) {
        scores.push_back(index_score(get<0>(t), get<2>(t)));
        hints.push_back(get<1>(t));
    }

    vector<index_score> updated_scores;
    vector<vector<int> > oversegment_indices;
    for (size_t i = 0; i < scores.size(); ++i) {
        CloudT::Ptr voxel_centers(new CloudT);
        vector<double> vocabulary_norms;
        vector<map<int, double> > vocabulary_vectors;
        vector<map<int, int> > vocabulary_index_vectors;

        if (scores[i].first < noise_scans_size) {
            get_voxel_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, vocabulary_index_vectors, scores[i].first, obr_scans);
        }
        else {
            get_voxel_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, vocabulary_index_vectors, scores[i].first-noise_scans_size, obr_scans_annotations);
        }

        vector<int> selected_indices;
        double score = obr_segments.gvt.compute_min_combined_dist(selected_indices, features, vocabulary_vectors, vocabulary_norms, voxel_centers, mapping, hints[i]);
        updated_scores.push_back(index_score(scores[i].first, score));
        oversegment_indices.push_back(selected_indices);
    }

    auto p = sort_permutation(updated_scores, [](const index_score& s1, const index_score& s2) {
        return s1.second < s2.second; // find min elements!
    });
    updated_scores = apply_permutation(updated_scores, p);
    oversegment_indices = apply_permutation(oversegment_indices, p);
    updated_scores.resize(nbr_query);
    oversegment_indices.resize(nbr_query);

    // make this a function
    for (size_t i = 0; i < updated_scores.size(); ++i) {
        HistCloudT::Ptr result_features(new HistCloudT);
        CloudT::Ptr result_keypoints(new CloudT);
        if (updated_scores[i].first < noise_scans_size) {
            load_nth_keypoints_features_for_scan(result_keypoints, result_features, updated_scores[i].first, oversegment_indices[i], obr_scans);
        }
        else {
            load_nth_keypoints_features_for_scan(result_keypoints, result_features, updated_scores[i].first-noise_scans_size, oversegment_indices[i], obr_scans_annotations);
        }
        CloudT::Ptr result_cloud(new CloudT);
        if (updated_scores[i].first < noise_scans_size) {
            obr_scans.read_scan(result_cloud, updated_scores[i].first);
        }
        else {
            obr_scans_annotations.read_scan(result_cloud, updated_scores[i].first-noise_scans_size);
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
    const int nbr_query = 15; // 11
    const int nbr_reweight_query = 11;

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
        obr_segments.vt.top_combined_similarities(scores, features, nbr_reweight_query);
        //obr_segments.vt.top_similarities(scores, features, nbr_reweight_query);

        vector<index_score> reweight_scores;
        reweight_query_vocabulary_sift(reweight_scores, scores, cloud, features, keypoints, segment_id, nbr_query,
                                       obr_segments, obr_segments_annotations, noise_segments_size);

        scores.resize(nbr_query);
        for (index_score& s : scores) {
            if (s.first < noise_segments_size) {
                s.first = 0;//scan_ind_for_segment(s.first, obr_segments);
            }
            else {
                s.first = scan_ind_for_segment(s.first-noise_segments_size, obr_segments_annotations) + noise_scans_size;
            }
        }

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
        dataset_annotations::calculate_correct_ratio_exclude_sweep(instance_correct_ratios, instance, scan_id, scores, obr_scans_annotations, noise_scans_size);
        dataset_annotations::calculate_correct_ratio_exclude_sweep(reweight_correct_ratios, instance, scan_id, reweight_scores, obr_scans_annotations, noise_scans_size);

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

    //compute_and_save_segments(obr_scans);
    //compute_and_save_segments(obr_scans_noise);

    //save_pfhrgb_features_for_supervoxels(obr_segments);
    //save_pfhrgb_features_for_supervoxels(obr_segments_noise);
    //save_shot_features_for_supervoxels(obr_segments);
    //save_shot_features_for_supervoxels(obr_segments_noise);

    //save_split_features(obr_segments);
    //save_split_features(obr_segments_noise);
    //exit(0);

    // probably train using the noise segments
    //obr_segments_noise.train_grouped_vocabulary(12000, false);

    // TODO: add something like obr_segments_noise.get_scan_count()
    int noise_scans_size = 3526;
    //obr_segments_noise.add_others_to_grouped_vocabulary(10000, obr_segments, noise_scans_size);

    //obr_segments_noise.train_vocabulary_incremental(4000, false); // 12000
    int noise_segments_size = 63136;
    //int noise_segments_size = 20000;
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

    query_supervoxel_oversegments(query_data_iter, K, obr_segments_noise, obr_scans_noise, obr_segments, obr_scans, noise_scans_size);

    //query_supervoxels(query_data_iter, obr_segments_noise, obr_segments, obr_scans, noise_scans_size, noise_segments_size);

    /*CloudT::Ptr query_cloud_larger(new CloudT);
    pcl::io::loadPCDFile("/home/nbore/Data/rgb_0015_label_0.pcd", *query_cloud_larger);

    query_cloud(query_cloud_larger, obr_segments_noise, obr_scans_noise, obr_scans, noise_scans_size);*/

    cout << "Program finished..." << endl;

    return 0;
}
