#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include "eigen_cereal/eigen_cereal.h"
#include "cereal/types/utility.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/string.hpp"

#include "object_3d_retrieval/dataset_convenience.h"

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

pair<float, bool> segment_is_correct(CloudT::Ptr& cloud, const Eigen::Matrix3f& K, const string& annotation)
{
    cout << "annotation: " << annotation << endl;
    // annotation example: cereal_2 full 16 102 78 221 OR null
    if (annotation == "null") {
        return make_pair(0.0f, false);
    }

    vector<string> strs;
    boost::split(strs, annotation, boost::is_any_of(" \t\n"));

    int minx = stoi(strs[2]);
    int maxx = stoi(strs[3]);
    int miny = stoi(strs[4]);
    int maxy = stoi(strs[5]);

    size_t counter = 0;
    for (PointT& p : cloud->points) {
        Eigen::Vector3f q = K*p.getVector3fMap();
        int x = int(q(0)/q(2) + 0.5f);
        int y = int(q(1)/q(2) + 0.5f);
        if (x >= minx && x <= maxx && y >= miny && y <= maxy) {
            ++counter;
        }
    }

    return make_pair(float(counter)/float(cloud->size()), strs[1] == "full");
}

pair<bool, bool> is_correctly_classified(const string& instance, CloudT::Ptr& segment, const Eigen::Matrix3f& K, const string& segments_path, int i)
{
    string metadata_file = segments_path + "/segment" + to_string(i) + "/metadata.txt";
    string metadata;
    {
        ifstream f;
        f.open(metadata_file);
        getline(f, metadata);
        f.close();
    }
    // metadata example: cereal_1/patrol_run_1/room_0/room.xml 2

    vector<string> strs;
    boost::split(strs, metadata, boost::is_any_of(" \t\n"));

    string annotated_instance = boost::filesystem::path(strs[0]).parent_path().parent_path().parent_path().stem().string();
    if (annotated_instance != instance) {
        cout << instance << " is not " << annotated_instance << endl;
        return make_pair(false, false);
    }
    string annotation_file = boost::filesystem::path(strs[0]).parent_path().string() + "/annotation" + strs[1] + ".txt";

    string annotation;
    {
        ifstream f;
        f.open(annotation_file);
        getline(f, annotation);
        f.close();
    }

    float ratio;
    bool is_full;
    tie(ratio, is_full) = segment_is_correct(segment, K, annotation);
    cout << "ratio: " << ratio << endl;
    return make_pair(ratio > 0.85f, is_full);
}

string get_annotation(const string& sweep_folder, int i)
{

    string annotation_file = sweep_folder + "/annotation" + to_string(i) + ".txt";
    string annotation;
    {
        ifstream f;
        f.open(annotation_file);
        getline(f, annotation);
        f.close();
    }
    if (annotation == "null") {
        return annotation;
    }
    vector<string> strs2;
    boost::split(strs2, annotation, boost::is_any_of(" \t\n"));

    return strs2[0];
}

void list_all_annotated_segments(map<int, string>& annotated, map<int, string>& full_annotated, const string& segments_path)
{
    // example: "/home/nbore/Data/Instances/object_segments"
    boost::filesystem::path path(segments_path);
    boost::filesystem::path annotations_file = path.parent_path() / "annotations.cereal";
    if (boost::filesystem::is_regular_file(annotations_file)) {
        ifstream in(annotations_file.string(), std::ios::binary);
        cereal::BinaryInputArchive archive_i(in);
        archive_i(annotated);
        archive_i(full_annotated);
        return;
    }

    // list all of the folders, check that they start with "segment"
    typedef vector<boost::filesystem::path> vec;
    vec v; // so we can sort them later
    copy(boost::filesystem::directory_iterator(path), boost::filesystem::directory_iterator(), back_inserter(v));
    for (const boost::filesystem::path& folder : v) {
        if (!boost::filesystem::is_directory(folder)) {
            continue;
        }
        string name = folder.stem().string();
        if (name.substr(0, 7) != "segment") {
            continue;
        }
        string last_part(name.begin()+7, name.end());
        // add a try catch here
        int segment_id = stoi(last_part);
        CloudT::Ptr segment(new CloudT);
        if (pcl::io::loadPCDFile<PointT>(folder.string() + "/hd_segment.pcd", *segment) == -1) {
            cout << "Could not find potentially annotated segment" << endl;
            exit(0);
        }
        string metadata_file = folder.string() + "/metadata.txt";
        string metadata;
        {
            ifstream f;
            f.open(metadata_file);
            f >> metadata;
            f.close();
        }
        Eigen::Matrix3f K;
        string intrinsics_file = folder.string() + "/K.cereal";
        {
            ifstream in(intrinsics_file, std::ios::binary);
            cereal::BinaryInputArchive archive_i(in);
            archive_i(K);
        }
        string instance = boost::filesystem::path(metadata).parent_path().parent_path().parent_path().stem().string();
        bool is_instance;
        bool is_full;
        tie(is_instance, is_full) = is_correctly_classified(instance, segment, K, segments_path, segment_id);
        if (is_instance) {
            annotated.insert(make_pair(segment_id, instance));
            if (is_full) {
                full_annotated.insert(make_pair(segment_id, instance));
            }
        }
    }

    {
        ofstream out(annotations_file.string(), std::ios::binary);
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(annotated);
        archive_o(full_annotated);
    }
}

void list_all_annotations(map<int, string>& annotated, map<int, string>& full_annotated, const string& segments_path)
{
    // list all of the folders, check that they start with "segment"
    boost::filesystem::path path(segments_path);
    typedef vector<boost::filesystem::path> vec;
    vec v; // so we can sort them later
    copy(boost::filesystem::directory_iterator(path), boost::filesystem::directory_iterator(), back_inserter(v));
    for (const boost::filesystem::path& folder : v) {
        if (!boost::filesystem::is_directory(folder)) {
            continue;
        }
        string name = folder.stem().string();
        if (name.substr(0, 7) != "segment") {
            continue;
        }
        string last_part(name.begin()+7, name.end());
        // add a try catch here
        int segment_id = stoi(last_part);
        CloudT::Ptr segment(new CloudT);
        if (pcl::io::loadPCDFile<PointT>(folder.string() + "/hd_segment.pcd", *segment) == -1) {
            cout << "Could not find potentially annotated segment" << endl;
            exit(0);
        }
        string metadata_file = folder.string() + "/metadata.txt";
        string metadata;
        {
            ifstream f;
            f.open(metadata_file);
            f >> metadata;
            f.close();
        }
        Eigen::Matrix3f K;
        string intrinsics_file = folder.string() + "/K.cereal";
        {
            ifstream in(intrinsics_file, std::ios::binary);
            cereal::BinaryInputArchive archive_i(in);
            archive_i(K);
        }
        string instance = boost::filesystem::path(metadata).parent_path().parent_path().parent_path().stem().string();
        bool is_instance;
        bool is_full;
        tie(is_instance, is_full) = is_correctly_classified(instance, segment, K, segments_path, segment_id);
        if (is_instance) {
            annotated.insert(make_pair(segment_id, instance));
            if (is_full) {
                full_annotated.insert(make_pair(segment_id, instance));
            }
        }
    }
}

void compute_instance_counts(map<string, int>& instance_counts, map<int, string>& annotated)
{
    for (pair<const int, string>& a : annotated) {
        string key = a.second;
        if (instance_counts.count(key) == 0) {
            instance_counts.insert(make_pair(key, 1));
        }
        else {
            instance_counts.at(key) += 1;
        }
    }
}

void list_all_annotated_scans(map<string, int>& annotations, const string& scans_path)
{
    object_retrieval obr(scans_path);
    obr.segment_name = "scan";
    for (int i = 0; ; ++i) {
        string folder = obr.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(folder)) {
            break;
        }
        string annotation = dataset_convenience::annotation_for_scan(i, obr);
        annotations[annotation] += 1;
    }
}

int main(int argc, char** argv)
{
    string annotations_path = "/home/nbore/Data/Instances/object_segments";
    map<int, string> annotated;
    map<int, string> full_annotated;
    list_all_annotated_segments(annotated, full_annotated, annotations_path);
    map<string, int> full_instance_counts; // the number of fully observed instances of every type
    map<string, int> all_instance_counts;
    compute_instance_counts(full_instance_counts, full_annotated);
    compute_instance_counts(all_instance_counts, annotated);
    map<string, int> all_annotations;
    list_all_annotated_scans(all_annotations, "/home/nbore/Data/Instances/scan_segments");

    stringstream fully_string;
    fully_string << "Fully observed";
    stringstream partially_string;
    partially_string << "Partially observed";
    stringstream total_string;
    total_string << "All annotated";
    for (pair<const string, int>& a : all_instance_counts) {
        int fully_observed = full_instance_counts[a.first];
        int total_annotation = all_annotations[a.first];
        cout << "Partially observed " << a.first << ": " << a.second - fully_observed << endl;
        cout << "Fully observed " << a.first << ": " << fully_observed << endl;
        cout << "Total annotations " << a.first << ": " << total_annotation << endl;
        fully_string << " & " << fully_observed;
        partially_string << " & " << a.second - fully_observed;
        total_string << " & " << total_annotation;
    }

    cout << fully_string.str() << endl;
    cout << partially_string.str() << endl;
    cout << total_string.str() << endl;

    return 0;
}
