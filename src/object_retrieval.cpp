#include "object_3d_retrieval/object_retrieval.h"

#include <iostream>
#include <boost/filesystem.hpp>

#include "object_3d_retrieval/convex_voxel_segmentation.h"
#include "object_3d_retrieval/segment_features.h"
#include "object_3d_retrieval/register_objects.h"

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "eigen_cereal/eigen_cereal.h"
#include <cereal/types/vector.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/utility.hpp>

#define VISUALIZE false

using namespace std;

// using SIFT
//string object_retrieval::feature_vocabulary_file = "vocabulary_sift.cereal";
//string object_retrieval::feature_vocabulary_file = "vocabulary_duplet.cereal";
//string object_retrieval::feature_segment_file = "features_sift.pcd";
//string object_retrieval::feature_segment_file = "duplet_points_file.pcd";
//string object_retrieval::feature_segment_file = "sift_cloud.pcd";
//string object_retrieval::indices_segment_file = "indices_sift.cereal";

// using PFH
string object_retrieval::feature_vocabulary_file = "vocabulary_pfhrgb_3.cereal";
string object_retrieval::grouped_vocabulary_file = "vocabulary_grouped_3.cereal";
string object_retrieval::feature_segment_file = "pfhrgb_cloud.pcd";
string object_retrieval::indices_segment_file = "indices_pfhrgb.cereal";

// using SHOT
//string object_retrieval::feature_vocabulary_file = "vocabulary.cereal";
//string object_retrieval::feature_segment_file = "features.pcd";
//string object_retrieval::indices_segment_file = "indices.cereal";

object_retrieval::object_retrieval(const std::string& segment_path) : segment_path(segment_path)
{
    segment_name = "segment";
}

object_retrieval::~object_retrieval()
{
    ofstream out(segment_path + "/saved_match_scores.cereal", std::ios::binary);
    cereal::BinaryOutputArchive archive_o(out);
    archive_o(saved_match_scores);
    cout << "Saved match scores!" << endl;
}

void object_retrieval::visualize_cloud(CloudT::Ptr& cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void object_retrieval::extract_features(vector<int>& inds, HistCloudT::Ptr& features, vector<CloudT::Ptr>& segments,
                      vector<NormalCloudT::Ptr>& normals, vector<CloudT::Ptr>& hd_segments, const Eigen::Matrix3f& K)
{
    int counter = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        segment_features sf(K, false);
        HistCloudT::Ptr featuresi(new HistCloudT);
        Eigen::VectorXf globalf;
        sf.calculate_features(globalf, featuresi, segments[i], normals[i], hd_segments[i]);
        for (size_t j = 0; j < featuresi->size(); ++j) {
            inds.push_back(counter);
        }
        features->insert(features->end(), featuresi->begin(), featuresi->end());
        ++counter;
    }
}

void object_retrieval::extract_feature(vector<int>& inds, HistCloudT::Ptr& feature, CloudT::Ptr& segment,
                                       NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment, const Eigen::Matrix3f& K, int ind)
{
    segment_features sf(K, false);
    Eigen::VectorXf globalf;
    sf.calculate_features(globalf, feature, segment, normal, hd_segment);
    for (size_t j = 0; j < feature->size(); ++j) {
        inds.push_back(ind);
    }
}

void object_retrieval::get_query_cloud(HistCloudT::Ptr& query_cloud, CloudT::Ptr& segment, NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment, Eigen::Matrix3f& K)
{
    segment_features sf(K, false);
    Eigen::VectorXf globalf;
    sf.calculate_features(globalf, query_cloud, segment, normal, hd_segment);
    cout << "Number of features: " << query_cloud->size() << endl;
    //visualize_cloud(hd_segment);
}

size_t object_retrieval::write_segments(vector<CloudT::Ptr>& segments, vector<NormalCloudT::Ptr>& normals, vector<CloudT::Ptr>& hd_segments,  const Eigen::Matrix3f& K, vector<string>& files, size_t istart)
{
    boost::filesystem::path base_dir = segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    boost::filesystem::create_directory(base_dir); // may fail if already existing
    // for each segment, create a folder indicating segment name
    size_t i;
    for (i = istart; i < istart+segments.size(); ++i) {
        // save the point clouds for this segment
        size_t j = i - istart;
        boost::filesystem::path sub_dir = base_dir / (segment_name + to_string(i));
        cout << "Writing directory " << sub_dir.string() << endl;
        boost::filesystem::create_directory(sub_dir);
        pcl::io::savePCDFileBinary(sub_dir.string() + "/segment.pcd", *segments[j]);
        pcl::io::savePCDFileBinary(sub_dir.string() + "/normals.pcd", *normals[j]);
        pcl::io::savePCDFileBinary(sub_dir.string() + "/hd_segment.pcd", *hd_segments[j]);
        {
            ofstream out(sub_dir.string() + "/K.cereal", std::ios::binary);
            cereal::BinaryOutputArchive archive_o(out);
            archive_o(K);
        }
        ofstream f;
        f.open(sub_dir.string() + "/metadata.txt");
        f << files[j] << '\n';
        f.close();
    }
    return i;
}

void object_retrieval::read_segments(vector<CloudT::Ptr>& segments, vector<NormalCloudT::Ptr>& normals,
                                     vector<CloudT::Ptr>& hd_segments, Eigen::Matrix3f& K, size_t max_segments)
{
    boost::filesystem::path base_dir = segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    for (size_t i = 0; ; ++i) { // i < max_segments
        boost::filesystem::path sub_dir = base_dir / (segment_name + to_string(i));
        cout << "Reading directory " << sub_dir.string() << endl;
        if (!boost::filesystem::is_directory(sub_dir)) {
            break;
        }
        segments.push_back(CloudT::Ptr(new CloudT));
        normals.push_back(NormalCloudT::Ptr(new NormalCloudT));
        hd_segments.push_back(CloudT::Ptr(new CloudT));
        if (pcl::io::loadPCDFile<PointT>(sub_dir.string() + "/segment.pcd", *segments[i]) == -1) exit(0);
        if (pcl::io::loadPCDFile<NormalT>(sub_dir.string() + "/normals.pcd", *normals[i]) == -1) exit(0);
        if (pcl::io::loadPCDFile<PointT>(sub_dir.string() + "/hd_segment.pcd", *hd_segments[i]) == -1) exit(0);
        {
            ifstream in(sub_dir.string() + "/K.cereal", std::ios::binary);
            cereal::BinaryInputArchive archive_i(in);
            archive_i(K);
        }
    }
}

bool object_retrieval::read_segment(CloudT::Ptr& segment, NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment,
                                    Eigen::Matrix3f& K, string& metadata, size_t segment_id)
{
    cout << "Entering" << endl;
    boost::filesystem::path base_dir = segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    boost::filesystem::path sub_dir = base_dir / (segment_name + to_string(segment_id));
    cout << "Reading directory " << sub_dir.string() << endl;
    if (!boost::filesystem::is_directory(sub_dir)) {
        return false;
    }
    if (pcl::io::loadPCDFile<PointT>(sub_dir.string() + "/segment.pcd", *segment) == -1) exit(0);
    if (pcl::io::loadPCDFile<NormalT>(sub_dir.string() + "/normals.pcd", *normal) == -1) exit(0);
    if (pcl::io::loadPCDFile<PointT>(sub_dir.string() + "/hd_segment.pcd", *hd_segment) == -1) exit(0);
    {
        cout << "Reading matrix: " << sub_dir.string() + "/K.cereal" << endl;
        ifstream in(sub_dir.string() + "/K.cereal", std::ios::binary);
        cereal::BinaryInputArchive archive_i(in);
        archive_i(K);
    }
    ifstream f;
    f.open(sub_dir.string() + "/metadata.txt");
    f >> metadata;
    f.close();
    cout << "Returning" << endl;
    return true;
}

bool object_retrieval::read_segment(CloudT::Ptr& segment, size_t segment_id)
{
    boost::filesystem::path base_dir = segment_path;
    boost::filesystem::path sub_dir = base_dir / (segment_name + to_string(segment_id));
    cout << "Reading directory " << sub_dir.string() << endl;
    if (!boost::filesystem::is_directory(sub_dir)) {
        return false;
    }
    if (pcl::io::loadPCDFile<PointT>(sub_dir.string() + "/segment.pcd", *segment) == -1) exit(0);
    cout << "Returning" << endl;
    return true;
}

bool object_retrieval::read_other_segment(CloudT::Ptr& segment, NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment,
                                    Eigen::Matrix3f& K, string& metadata, size_t segment_id, const string& other_segment_path)
{
    boost::filesystem::path base_dir = other_segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    boost::filesystem::path sub_dir = base_dir / (segment_name + to_string(segment_id));
    cout << "Reading directory " << sub_dir.string() << endl;
    if (!boost::filesystem::is_directory(sub_dir)) {
        return false;
    }
    if (pcl::io::loadPCDFile<PointT>(sub_dir.string() + "/segment.pcd", *segment) == -1) exit(0);
    if (pcl::io::loadPCDFile<NormalT>(sub_dir.string() + "/normals.pcd", *normal) == -1) exit(0);
    if (pcl::io::loadPCDFile<PointT>(sub_dir.string() + "/hd_segment.pcd", *hd_segment) == -1) exit(0);
    {
        ifstream in(sub_dir.string() + "/K.cereal", std::ios::binary);
        cereal::BinaryInputArchive archive_i(in);
        archive_i(K);
    }
    ifstream f;
    f.open(sub_dir.string() + "/metadata.txt");
    f >> metadata;
    f.close();
    return true;
}

void object_retrieval::write_vocabulary(vocabulary_tree<HistT, 8>& vt)
{
    boost::filesystem::path base_dir = segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    boost::filesystem::create_directory(base_dir);
    std::string vocabulary_file = base_dir.string() + "/" + feature_vocabulary_file;
    ofstream out(vocabulary_file, std::ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(vt);
    }
}

void object_retrieval::write_vocabulary(grouped_vocabulary_tree<HistT, 8>& vt)
{
    boost::filesystem::path base_dir = segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    boost::filesystem::create_directory(base_dir);
    std::string vocabulary_file = base_dir.string() + "/" + grouped_vocabulary_file;
    ofstream out(vocabulary_file, std::ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(vt);
    }
}

void object_retrieval::read_vocabulary(vocabulary_tree<HistT, 8>& rvt)
{
    boost::filesystem::path base_dir = segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    std::string vocabulary_file = base_dir.string() + "/" + feature_vocabulary_file;
    {
        ifstream in(vocabulary_file, std::ios::binary);
        {
            cereal::BinaryInputArchive archive_i(in);
            archive_i(rvt);
        }
        in.close();
    }
    cout << "Vocabulary size: " << rvt.size() << endl;
}

void object_retrieval::read_vocabulary(vocabulary_tree<HistT, 8>& vt, const string& vocabulary_path)
{
    ifstream in(vocabulary_path, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(vt);
    }
    in.close();
    cout << "Vocabulary size: " << vt.size() << endl;
}

void object_retrieval::read_vocabulary(grouped_vocabulary_tree<HistT, 8>& rvt)
{
    boost::filesystem::path base_dir = segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    std::string vocabulary_file = base_dir.string() + "/" + grouped_vocabulary_file;
    {
        ifstream in(vocabulary_file, std::ios::binary);
        {
            cereal::BinaryInputArchive archive_i(in);
            archive_i(rvt);
        }
        in.close();
    }
    cout << "Vocabulary size: " << rvt.size() << endl;
}

pair<double, double> object_retrieval::calculate_similarity(CloudT::Ptr& cloud1, const Eigen::Matrix3f& K1,
                           CloudT::Ptr& cloud2, const Eigen::Matrix3f& K2)
{
    register_objects ro;
    ro.set_input_clouds(cloud1, K1, cloud2, K2);
    ro.do_registration();
    return ro.get_match_score();
}

size_t object_retrieval::compute_segments(vector<CloudT::Ptr>& sweeps, vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& intrinsics, vector<string>& files, size_t i)
{
    size_t counter = 0;
    for (CloudT::Ptr cloud : sweeps) {
        if (VISUALIZE) {
            visualize_cloud(cloud);
        }
        convex_voxel_segmentation cvs(VISUALIZE, 0.012f, 0.02f);
        vector<CloudT::Ptr> segmentsi;
        vector<NormalCloudT::Ptr> normalsi;
        vector<CloudT::Ptr> hd_segmentsi;
        vector<string> filesi;
        cvs.segment_objects(segmentsi, normalsi, hd_segmentsi, cloud);

        for (size_t j = 0; j < segmentsi.size(); ++j) {
            filesi.push_back(files[counter]);
        }
        ++counter;

        i = write_segments(segmentsi, normalsi, hd_segmentsi,  intrinsics[0], filesi, i);
    }

    return i;
}

void object_retrieval::process_segments()
{
    HistCloudT::Ptr features(new HistCloudT);
    vector<int> indices;
    if (!load_features(features, indices)) {
        vector<CloudT::Ptr> segments;
        vector<NormalCloudT::Ptr> normals;
        vector<CloudT::Ptr> hd_segments;
        Eigen::Matrix3f K;

        read_segments(segments, normals, hd_segments, K, 200);

        cout << K << endl;
        cout << segments.size() << endl;
        cout << hd_segments.size() << endl;
        cout << normals.size() << endl;
        cout << segments[0]->size() << endl;
        cout << hd_segments[0]->size() << endl;
        cout << normals[0]->size() << endl;

        extract_features(indices, features, segments, normals, hd_segments, K);
        save_features(features, indices);
    }

    for (int i : indices) cout << i << " "; cout << endl;

    for (HistT& h : features->points) {
        eig(h).normalize();
    }

    vocabulary_tree<HistT, 8> vt1;
    vt1.set_input_cloud(features, indices);
    vt1.add_points_from_input_cloud();

    write_vocabulary(vt1);
}

void object_retrieval::process_segments_incremental()
{
    for (size_t i = 0; ; ++i) { // remember to put this back to zero
        CloudT::Ptr segment(new CloudT);
        NormalCloudT::Ptr normal(new NormalCloudT);
        CloudT::Ptr hd_segment(new CloudT);
        Eigen::Matrix3f K;
        string metadata;
        if (!read_segment(segment, normal, hd_segment, K, metadata, i)) {
            break;
        }

        HistCloudT::Ptr features_i(new HistCloudT);
        vector<int> indices_i;
        extract_feature(indices_i, features_i, segment, normal, hd_segment, K, i);

        save_features_for_segment(features_i, i);
    }
}

int object_retrieval::scan_ind_for_segment(int i)
{
    string segment_folder = get_folder_for_segment_id(i);

    if (!boost::filesystem::is_directory(segment_folder)) {
        return -1;
    }

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
    int ind = stoi(scan_name.substr(pos+1));

    return ind;
}

void object_retrieval::train_grouped_vocabulary(int max_segments, bool simply_train)
{
    int min_features = 20;

    grouped_vocabulary_tree<HistT, 8> vt1;
    size_t counter = 0;
    bool are_done = false;

    int current_scan_ind = -1;
    int current_scan_group = 0;
    {
        HistCloudT::Ptr features(new HistCloudT);
        vector<pair<int, int> > indices;

        // atm the training has problems dealing with more than ~700000 vectors
        for (size_t i = 0; i < max_segments && features->size() < 1200000; ++i) {
            if (!exclude_set.empty()) {
                if (exclude_set.count(counter) != 0) {
                    ++counter;
                    continue;
                }
            }

            int scan_ind = scan_ind_for_segment(counter);
            if (scan_ind == -1) {
                are_done = true;
                break;
            }
            if (current_scan_ind != scan_ind) {
                current_scan_ind = scan_ind;
                current_scan_group = 0;
            }

            HistCloudT::Ptr features_i(new HistCloudT);
            vector<pair<int, int> > indices_i;
            int groups_loaded = load_grouped_features_for_segment(features_i, indices_i, counter, scan_ind, current_scan_group);
            if (groups_loaded == -1) {
                are_done = true;
                break;
            }
            if (features_i->size() < min_features) {
                ++counter;
                continue;
            }
            current_scan_group += groups_loaded;
            features->insert(features->end(), features_i->begin(), features_i->end());
            indices.insert(indices.end(), indices_i.begin(), indices_i.end());
            ++counter;
        }

        cout << "Adding " << features->size() << " features to vocabulary." << endl;
        vt1.set_input_cloud(features, indices);
        vt1.add_points_from_input_cloud(false);
    }

    if (simply_train) {
        write_vocabulary(vt1);
        return;
    }

    while (!are_done) {
        HistCloudT::Ptr features(new HistCloudT);
        vector<pair<int, int> > indices;

        // as long as we can keep them all in memory, no upper bound on features here
        for (size_t i = 0; i < max_segments && features->size() < 2000000; ++i) {
            if (!exclude_set.empty()) {
                if (exclude_set.count(counter) != 0) {
                    ++counter;
                    continue;
                }
            }

            int scan_ind = scan_ind_for_segment(counter);
            if (scan_ind == -1) {
                are_done = true;
                break;
            }
            if (current_scan_ind != scan_ind) {
                current_scan_ind = scan_ind;
                current_scan_group = 0;
            }

            HistCloudT::Ptr features_i(new HistCloudT);
            vector<pair<int, int> > indices_i;
            int groups_loaded = load_grouped_features_for_segment(features_i, indices_i, counter, scan_ind, current_scan_group);
            if (groups_loaded == -1) {
                are_done = true;
                break;
            }
            if (features_i->size() < min_features) {
                ++counter;
                continue;
            }
            current_scan_group += groups_loaded;
            features->insert(features->end(), features_i->begin(), features_i->end());
            indices.insert(indices.end(), indices_i.begin(), indices_i.end());
            ++counter;
        }

        if (features->size() > 0) {
            vt1.append_cloud(features, indices, false);
        }
    }

    write_vocabulary(vt1);
    string group_file = segment_path + "/group_subgroup_1.cereal";
    vt1.save_group_associations(group_file);
}

int object_retrieval::add_others_to_grouped_vocabulary(int max_segments, object_retrieval& obr_segments, int previous_scan_size)
{
    int min_features = 20;
    string group_file = segment_path + "/group_subgroup_1.cereal";

    if (gvt.empty()) {
        read_vocabulary(gvt);
    }
    gvt.load_group_associations(group_file);

    size_t counter = 0;
    bool are_done = false;

    int current_scan_ind = -1;
    int current_scan_group = 0;
    while (!are_done) {
        HistCloudT::Ptr features(new HistCloudT);
        vector<pair<int, int> > indices;

        // as long as we can keep them all in memory, no upper bound on features here
        for (size_t i = 0; i < max_segments && features->size() < 2000000; ++i) {
            if (!exclude_set.empty()) {
                if (exclude_set.count(counter) != 0) {
                    ++counter;
                    continue;
                }
            }

            // TODO: add a method to check how many scans we have in this folder
            // we could also add a method to grouped vocabulary tree to do the same
            int scan_ind = obr_segments.scan_ind_for_segment(counter) + previous_scan_size; // this is to add above the ones already in there
            if (scan_ind == -1) {
                are_done = true;
                break;
            }
            if (current_scan_ind != scan_ind) {
                current_scan_ind = scan_ind;
                current_scan_group = 0;
            }

            HistCloudT::Ptr features_i(new HistCloudT);
            vector<pair<int, int> > indices_i;
            int groups_loaded = obr_segments.load_grouped_features_for_segment(features_i, indices_i, counter, scan_ind, current_scan_group);
            if (groups_loaded == -1) {
                are_done = true;
                break;
            }
            if (features_i->size() < min_features) {
                ++counter;
                continue;
            }
            current_scan_group += groups_loaded;
            features->insert(features->end(), features_i->begin(), features_i->end());
            indices.insert(indices.end(), indices_i.begin(), indices_i.end());
            ++counter;
        }

        cout << "Features size: " << features->size() << endl;
        cout << "Current scan ind: " << current_scan_ind << endl;
        if (features->size() > 0) {
            gvt.append_cloud(features, indices, false);
        }
    }

    write_vocabulary(gvt);
    gvt.save_group_associations(group_file);
    //gvt.save_group_associations(segment_path + "/group_subgroup_1.cereal");

    return previous_scan_size;
}

// this should probably be max_features instead
void object_retrieval::train_vocabulary_incremental(int max_segments, bool simply_train)
{
    int min_features = 20;

    vocabulary_tree<HistT, 8> vt1;
    size_t counter = 0;
    bool are_done = false;

    {
        HistCloudT::Ptr features(new HistCloudT);
        vector<int> indices;

        // atm the training has problems dealing with more than ~700000 vectors
        for (size_t i = 0; i < max_segments && features->size() < 2000000; ++i) {
            if (!exclude_set.empty()) {
                if (exclude_set.count(counter) != 0) {
                    ++counter;
                    continue;
                }
            }
            HistCloudT::Ptr features_i(new HistCloudT);
            if (!load_features_for_segment(features_i, counter)) {
                are_done = true;
                break;
            }
            if (features_i->size() < min_features) {
                ++counter;
                continue;
            }
            features->insert(features->end(), features_i->begin(), features_i->end());
            for (size_t j = 0; j < features_i->size(); ++j) {
                indices.push_back(counter);
            }
            ++counter;
        }

        vt1.set_input_cloud(features, indices);
        vt1.add_points_from_input_cloud(false);
    }

    if (simply_train) {
        write_vocabulary(vt1);
        return;
    }

    while (!are_done) {
        HistCloudT::Ptr features(new HistCloudT);
        vector<int> indices;

        // as long as we can keep them all in memory, no upper bound on features here
        for (size_t i = 0; i < max_segments && features->size() < 2000000; ++i) {
            if (!exclude_set.empty()) {
                if (exclude_set.count(counter) != 0) {
                    ++counter;
                    continue;
                }
            }
            HistCloudT::Ptr features_i(new HistCloudT);
            if (!load_features_for_segment(features_i, counter)) {
                are_done = true;
                break;
            }
            if (features_i->size() < min_features) {
                ++counter;
                continue;
            }
            features->insert(features->end(), features_i->begin(), features_i->end());
            for (size_t j = 0; j < features_i->size(); ++j) {
                indices.push_back(counter);
            }
            ++counter;
        }

        if (features->size() > 0) {
            vt1.append_cloud(features, indices, false);
        }
    }

    write_vocabulary(vt1);
}

// this should just be 2 separate functions
int object_retrieval::add_others_to_vocabulary(int max_segments, const std::string& other_segment_path, int nbr_original_segments, bool add_some)
{
    if (vt.empty()) {
        read_vocabulary(vt);
    }
    int initial_size;
    size_t counter;
    if (add_some) {
        initial_size = 0;
        counter = nbr_original_segments;
    }
    else {
        if (nbr_original_segments != -1) {
            initial_size = nbr_original_segments;
        }
        else {
            initial_size = vt.max_ind();
        }
        counter = 0;
    }
    bool are_done = false;

    while (!are_done) {
        HistCloudT::Ptr features(new HistCloudT);
        vector<int> indices;

        for (size_t i = 0; i < max_segments; ++i) {
            HistCloudT::Ptr features_i(new HistCloudT);
            if (!load_features_for_other_segment(features_i, other_segment_path, counter)) {
                are_done = true;
                break;
            }
            features->insert(features->end(), features_i->begin(), features_i->end());
            for (size_t j = 0; j < features_i->size(); ++j) {
                indices.push_back(initial_size + counter);
            }
            ++counter;
        }

        if (!features->empty()) {
            vt.append_cloud(features, indices, false);
        }
        if (add_some) {
            break;
        }
    }

    write_vocabulary(vt);

    if (add_some) {
        return static_cast<int>(are_done); // we need to communicate if we've reached the end
    }
    else {
        return initial_size;
    }
}

void object_retrieval::query_vocabulary(vector<index_score>& scores, size_t query_ind, size_t nbr_query, bool visualize_query,
                                        int number_original_features, const string& other_segments_path)
{
    HistCloudT::Ptr query_cloud(new HistCloudT);
    CloudT::Ptr segment(new CloudT);
    CloudT::Ptr query_segment(new CloudT);
    NormalCloudT::Ptr normal(new NormalCloudT);
    CloudT::Ptr hd_segment(new CloudT);
    Eigen::Matrix3f K, query_K;
    string metadata;

    if (vt.empty()) {
        read_vocabulary(vt);
    }

    cout << "Querying segment nbr: " << query_ind << endl;
    if (number_original_features != 0 && query_ind >= number_original_features) {
        if (!read_other_segment(segment, normal, query_segment, query_K, metadata,
                                query_ind-number_original_features, other_segments_path)) { // 20 Drawer // 34 Blue Cup // 50 Monitor
            cout << "Error reading segment!" << endl;
            exit(0);
        }
    }
    else {
        if (!read_segment(segment, normal, query_segment, query_K, metadata, query_ind)) { // 20 Drawer // 34 Blue Cup // 50 Monitor
            cout << "Error reading segment!" << endl;
            exit(0);
        }
    }
    if (true) { // DEBUG
        if (number_original_features != 0 && query_ind >= number_original_features) { // annotations in different data set
            load_features_for_other_segment(query_cloud, other_segments_path, query_ind-number_original_features); // test_reweight_results
        }
        else {
            load_features_for_segment(query_cloud, query_ind);
        }
    }
    else {
        get_query_cloud(query_cloud, segment, normal, query_segment, query_K);
        if (visualize_query) {
            visualize_cloud(query_segment);
        }
    }

    /*for (HistT& h : query_cloud->points) {
        eig(h).normalize();
    }*/

    vt.top_similarities(scores, query_cloud, nbr_query);

    if (visualize_query) {
        for (index_score s : scores) {
            cout << "Index: " << s.first << " with score: " << s.second << endl;
            if (number_original_features != 0 && s.first >= number_original_features) {
                if (!read_other_segment(segment, normal, hd_segment, K, metadata,
                                        s.first-number_original_features, other_segments_path)) { // 20 Drawer // 34 Blue Cup // 50 Monitor
                    cout << "Error reading segment!" << endl;
                    exit(0);
                }
            }
            else {
                if (!read_segment(segment, normal, hd_segment, K, metadata, s.first)) {
                    cout << "Error reading segment!" << endl;
                    exit(0);
                }
            }
            visualize_cloud(hd_segment);
        }
    }
}

void object_retrieval::query_reweight_vocabulary(vector<index_score>& first_scores, vector<index_score>& reweight_scores,
                                                 size_t query_ind, size_t nbr_query, bool visualize_query,
                                                 int number_original_features, const string& other_segments_path)
{
    HistCloudT::Ptr query_cloud(new HistCloudT);
    CloudT::Ptr segment(new CloudT);
    CloudT::Ptr query_segment(new CloudT);
    NormalCloudT::Ptr normal(new NormalCloudT);
    CloudT::Ptr hd_segment(new CloudT);
    Eigen::Matrix3f K, query_K;
    string metadata;

    cout << "Querying segment nbr: " << query_ind << endl;
    if (number_original_features != 0 && query_ind >= number_original_features) { // annotations in different data set
        if (!read_other_segment(segment, normal, query_segment, query_K, metadata,
                                query_ind-number_original_features, other_segments_path)) { // 20 Drawer // 34 Blue Cup // 50 Monitor
            cout << "Error reading segment!" << endl;
            exit(0);
        }
    }
    else {
        if (!read_segment(segment, normal, query_segment, query_K, metadata, query_ind)) { // 20 Drawer // 34 Blue Cup // 50 Monitor
            cout << "Error reading segment!" << endl;
            exit(0);
        }
    }
    if (true) { // DEBUG
        if (number_original_features != 0 && query_ind >= number_original_features) { // annotations in different data set
            load_features_for_other_segment(query_cloud, other_segments_path, query_ind-number_original_features); // test_reweight_results
        }
        else {
            load_features_for_segment(query_cloud, query_ind);
        }
    }
    else {
        get_query_cloud(query_cloud, segment, normal, query_segment, query_K);
        if (visualize_query) {
            visualize_cloud(query_segment);
        }
    }

    /*for (HistT& h : query_cloud->points) {
        eig(h).normalize();
    }*/

    cout << "Histogram cloud size: " << query_cloud->size() << endl;

    if (rvt.empty()) {
        read_vocabulary(rvt);
    }
    rvt.top_similarities(first_scores, query_cloud, 2*(nbr_query-1)+1); // do 2 times matches to get better reweight results
    //rvt.top_pyramid_match_similarities(first_scores, query_cloud, nbr_query); // do 2 times matches to get better reweight results

    const int reweight_rounds = 1;
    saved_match_scores.insert(make_pair(query_ind, vector<pair<int, double> >()));
    for (int i = 0; i < reweight_rounds; ++i) { // do the reweighting and querying 1 time to begin with, maybe add caching of matches later
        map<int, double> weights;
        map<int, double> color_weights;
        double sum = 0.0;
        double color_sum = 0.0;
        for (index_score s : first_scores) {
            if (s.first == query_ind) { // we do not want to reweight with a perfect match (itself)
                continue;
            }
            cout << "Index: " << s.first << " with score: " << s.second << endl;
            if (number_original_features != 0 && s.first >= number_original_features) {
                if (!read_other_segment(segment, normal, hd_segment, K, metadata,
                                        s.first-number_original_features, other_segments_path)) { // 20 Drawer // 34 Blue Cup // 50 Monitor
                    cout << "Error reading segment!" << endl;
                    exit(0);
                }
            }
            else {
                if (!read_segment(segment, normal, hd_segment, K, metadata, s.first)) { // 20 Drawer // 34 Blue Cup // 50 Monitor
                    cout << "Error reading segment!" << endl;
                    exit(0);
                }
            }
            double similarity;
            double color_similarity;
            tie(similarity, color_similarity) = calculate_similarity(query_segment, query_K, hd_segment, K);
            if (std::isnan(similarity) || (use_color_weights && std::isnan(color_similarity))) {
                continue;
            }
            saved_match_scores.at(query_ind).push_back(make_pair(s.first, similarity));
            weights.insert(make_pair(s.first, similarity));
            color_weights.insert(make_pair(s.first, color_similarity));
            sum += similarity;
            color_sum += color_similarity;
            cout << "Shape similarity: " << similarity << endl;
            //visualize_cloud(hd_segment);
        }

        for (pair<const int, double>& w : weights) {
            /*if (use_color_weights) {
                w.second = (w.second/sum + color_weights.at(w.first)/color_sum)/double(weights.size());
            }
            else {
                w.second *= 1.0*double(weights.size())/sum; // 1.0 seems best, needs no explanation
            }*/
            //w.second = 1.5; // just to check how much "bootstrapping" contributes
            if (w.second < 80) {
                w.second = -1;
            }
            else {
                w.second = 1;
            }
        }

        reweight_scores.clear();
        rvt.top_similarities_reweighted(reweight_scores, weights, query_cloud, nbr_query);
        if (i < reweight_rounds-1) {
            first_scores.swap(reweight_scores);
        }
    }

    cout << "Reweighted distances: " << endl;
    for (index_score s : reweight_scores) {
        cout << s.second << ", ";
    }
    cout << endl;
}

void object_retrieval::read_scan(CloudT::Ptr& cloud, int i)
{
    string folder = get_folder_for_segment_id(i);
    string metadata_file = folder + "/metadata.txt";
    //cout << metadata_file << endl;
    string metadata; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(metadata_file);
        getline(f, metadata);
        f.close();
    }
    cout << metadata << endl;
    if (pcl::io::loadPCDFile(metadata, *cloud) == -1) {
        cout << "Could not load scan point cloud..." << endl;
        exit(-1);
    }
}

string object_retrieval::get_scan_file(int i)
{
    string folder = get_folder_for_segment_id(i);
    string metadata_file = folder + "/metadata.txt";
    string metadata; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(metadata_file);
        getline(f, metadata);
        f.close();
    }
    return metadata;
}

void object_retrieval::query_reweight_vocabulary(vector<index_score>& first_scores, vector<index_score>& reweight_scores,
                                                 HistCloudT::Ptr& query_cloud, CloudT::Ptr& query_segment, Eigen::Matrix3f& K, size_t nbr_query)
{
    static int m = 0;
    cout << "Histogram cloud size: " << query_cloud->size() << endl;

    if (rvt.empty()) {
        read_vocabulary(rvt);
    }
    rvt.top_larger_similarities(first_scores, query_cloud, 1*(nbr_query-1)+1);

    const int reweight_rounds = 1;
    for (int i = 0; i < reweight_rounds; ++i) { // do the reweighting and querying 1 time to begin with, maybe add caching of matches later
        map<int, double> weights;
        map<int, double> color_weights;
        double sum = 0.0;
        double color_sum = 0.0;
        bool first = true;
        for (index_score s : first_scores) {
            if (first) { // first is itself, the perfect match
                first = false;
                continue;
            }
            cout << "Index: " << s.first << " with score: " << s.second << endl;
            CloudT::Ptr hd_segment(new CloudT);
            read_scan(hd_segment, s.first);
            double similarity;
            double color_similarity;
            tie(similarity, color_similarity) = calculate_similarity(query_segment, K, hd_segment, K);
            if (std::isnan(similarity) || (use_color_weights && std::isnan(color_similarity))) {
                continue;
            }
            weights.insert(make_pair(s.first, similarity));
            color_weights.insert(make_pair(s.first, color_similarity));
            sum += similarity;
            color_sum += color_similarity;
            cout << "Shape similarity: " << similarity << endl;
        }

        for (pair<const int, double>& w : weights) {
            if (use_color_weights) {
                w.second = (w.second/sum + color_weights.at(w.first)/color_sum)/double(weights.size());
            }
            else {
                w.second *= 1.0*double(weights.size())/sum;
            }
        }

        reweight_scores.clear();
        rvt.top_similarities_reweighted(reweight_scores, weights, query_cloud, nbr_query);
        if (i < reweight_rounds-1) {
            first_scores.swap(reweight_scores);
        }
    }

    cout << "Reweighted distances: " << endl;
    for (index_score s : reweight_scores) {
        cout << s.second << ", ";
    }
    cout << endl;
    ++m;
}

POINT_CLOUD_REGISTER_POINT_STRUCT (object_retrieval::HistT,
                                   (float[object_retrieval::N], histogram, histogram)
)

void object_retrieval::save_features(HistCloudT::Ptr& features, std::vector<int>& indices)
{
    boost::filesystem::path base_dir = segment_path;
    std::string features_file = base_dir.string() + "/" + feature_segment_file;
    std::string indices_file = base_dir.string() + "/" + indices_segment_file;
    {
        ofstream out(indices_file, std::ios::binary);
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(indices);
    }
    pcl::io::savePCDFileBinary(features_file, *features);
}

bool object_retrieval::load_features(HistCloudT::Ptr& features, std::vector<int>& indices)
{
    boost::filesystem::path base_dir = segment_path;
    boost::filesystem::path features_file = base_dir / feature_segment_file;
    if (!boost::filesystem::is_regular_file(features_file)) {
        return false;
    }
    std::string indices_file = base_dir.string() + "/" + indices_segment_file;
    {
        ifstream in(indices_file, std::ios::binary);
        cereal::BinaryInputArchive archive_i(in);
        archive_i(indices);
    }
    return (pcl::io::loadPCDFile<HistT>(features_file.string(), *features) != -1);
}

void object_retrieval::save_features_for_segment(HistCloudT::Ptr& features, int i)
{
    if (features->empty()) {
        HistT p;
        for (size_t j = 0; j < N; ++j) {
            p.histogram[j] = std::numeric_limits<float>::quiet_NaN();
        }
        features->push_back(p);
    }
    boost::filesystem::path base_dir = segment_path;
    std::string features_file = base_dir.string() + "/" + segment_name + to_string(i) + "/" + feature_segment_file;
    pcl::io::savePCDFileBinary(features_file, *features);
}

bool object_retrieval::load_features_for_segment(HistCloudT::Ptr& features, int i)
{
    boost::filesystem::path base_dir = segment_path;
    std::string features_file = base_dir.string() + "/" + segment_name + to_string(i) + "/" + feature_segment_file;
    return (pcl::io::loadPCDFile<HistT>(features_file, *features) != -1);
}

int object_retrieval::load_grouped_features_for_segment(HistCloudT::Ptr& features, vector<pair<int, int> >& indices, int ind, int opt_ind, int current_group)
{
   string segment_folder = segment_path + "/" + segment_name + to_string(ind) + "/";
   if (!boost::filesystem::is_directory(segment_folder)) {
       return -1;
   }
   int i;
   for (i = 0; ;  ++i) {
       std::string features_file = segment_folder + "split_features" + to_string(i) + ".pcd";
       HistCloudT::Ptr featuresi(new HistCloudT);
       //cout << features_file << endl;
       if (pcl::io::loadPCDFile<HistT>(features_file, *featuresi) == -1) {
           return i;
       }
       features->insert(features->end(), featuresi->begin(), featuresi->end());
       for (int j = 0; j < featuresi->size(); ++j) {
           if (opt_ind != -1) {
               indices.push_back(make_pair(opt_ind, current_group + i));
           }
           else {
               indices.push_back(make_pair(ind, i));
           }
       }
   }

   return i; // won't reach
}

bool object_retrieval::load_features_for_other_segment(HistCloudT::Ptr& features, const std::string& other_segment_path, int i)
{
    boost::filesystem::path base_dir = other_segment_path;
    std::string features_file = base_dir.string() + "/" + segment_name + to_string(i) + "/" + feature_segment_file;
    std::cout << "features_file: " << features_file << std::endl;
    return (pcl::io::loadPCDFile<HistT>(features_file, *features) != -1);
}

string object_retrieval::get_folder_for_segment_id(int i) const
{
    return segment_path + "/" + segment_name + to_string(i) + "/";
}
