#include <iostream>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>

#include "object_3d_retrieval/object_retrieval.h"
#include "eigen_cereal/eigen_cereal.h"
#include "cereal/types/utility.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/string.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "simple_xml_parser.h"
#include "simple_summary_parser.h"
#include "simple_xml_parser.h"

#include <tf_conversions/tf_eigen.h>

using namespace std;

typedef typename SimpleSummaryParser::EntityStruct Entities;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<131>;
using HistCloudT = pcl::PointCloud<HistT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;

using index_score = object_retrieval::index_score;

void subsample_cloud(CloudT::Ptr& cloud_in, CloudT::Ptr& cloud_out)
{
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_out);
}

void translate_cloud(CloudT::Ptr& cloud, const Eigen::Vector3f& offset)
{
    for (PointT& p : cloud->points) {
        p.getVector3fMap() += offset;
    }
}

void get_rooms(vector<SimpleSummaryParser::EntityStruct>& entities, string summary_xml_path)
{
    SimpleSummaryParser summary_parser(summary_xml_path + "index.xml");
    summary_parser.createSummaryXML(summary_xml_path);
    entities = summary_parser.getRooms();
    cout << "Entities: " << entities.size() << endl;
}

void get_room_from_xml(vector<CloudT::Ptr>& clouds,
                       vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& transforms,
                       string& room_id, vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& intrinsics,
                       SimpleSummaryParser::EntityStruct& entity)
{
    SimpleXMLParser<PointT> parser;
    vector<string> xml_nodes_to_parser = {"RoomIntermediateCloud", "IntermediatePosition", "RoomStringId"};  // "RoomCompleteCloud", "RoomDynamicClusters"
    SimpleXMLParser<PointT>::RoomData room = parser.loadRoomFromXML(entity.roomXmlFile, xml_nodes_to_parser);

    size_t counter;
    for (const tf::StampedTransform& Tj : room.vIntermediateRoomCloudTransforms) {
        Eigen::Affine3d e;
        tf::transformTFToEigen(Tj, e);
        Eigen::Matrix3d R = e.rotation();
        // only take the scans looking down far enough
        if (acos(Eigen::Vector3d::UnitZ().dot(R.col(2))) < 0.6*M_PI) {
            ++counter;
            continue;
        }

        transforms.push_back(e);

        clouds.push_back(room.vIntermediateRoomClouds[counter]);

        image_geometry::PinholeCameraModel model = room.vIntermediateRoomCloudCamParams[counter];
        cv::Matx33d cvK = model.intrinsicMatrix();
        Eigen::Matrix3d dK = Eigen::Map<Eigen::Matrix3d>(cvK.val);
        intrinsics.push_back(dK.cast<float>().transpose());

        ++counter;
    }

    room_id = room.roomWaypointId;

    cout << "Found " << clouds.size() << " scans looking down far enough" << endl;
}

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

    cout << minx << " " << maxx << " " << miny << " " << maxy << endl;
    //cout << K << endl;

    size_t counter = 0;
    for (PointT& p : cloud->points) {
        Eigen::Vector3f q = K*p.getVector3fMap();
        int x = int(q(0)/q(2) + 0.5f);
        int y = int(q(1)/q(2) + 0.5f);
        //cout << x << " " << y << endl;
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
        //f >> metadata;
        f.close();
    }
    // metadata example: cereal_1/patrol_run_1/room_0/room.xml 2

    vector<string> strs;
    boost::split(strs, metadata, boost::is_any_of(" \t\n"));

    //string room_xml_file = annotations_path + "/" + strs[0];
    string annotated_instance = boost::filesystem::path(strs[0]).parent_path().parent_path().parent_path().stem().string();
    if (annotated_instance != instance) {
        cout << instance << " is not " << annotated_instance << endl;
        return make_pair(false, false);
    }
    string annotation_file = boost::filesystem::path(strs[0]).parent_path().string() + "/annotation" + strs[1] + ".txt";
    //int scan_id = stoi(strs[1]);

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
    return make_pair(ratio > 0.75f, is_full);

    /*SimpleXMLParser<PointT> parser;
    vector<string> xml_nodes_to_parser = {"RoomIntermediateCloud", "IntermediatePosition", "RoomStringId"};  // "RoomCompleteCloud", "RoomDynamicClusters"
    SimpleXMLParser<PointT>::RoomData room = parser.loadRoomFromXML(room_xml_file, xml_nodes_to_parser);

    CloudT::Ptr cloud = room.vIntermediateRoomClouds[scan_id];*/
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
        /*if (instance != "cereal_2") { // DEBUG: remove this when done!
            continue;
        }*/
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

void visualize_annotations(map<int, string>& annotated, const string& annotations_path)
{
    object_retrieval obr(annotations_path);
    for (pair<const int, string>& a : annotated) {
        cout << "Segment " << a.first << " is annotated with " << a.second << endl;
        CloudT::Ptr segment(new CloudT);
        string segment_file = annotations_path + "/segment" + to_string(a.first) + "/hd_segment.pcd";
        if (pcl::io::loadPCDFile<PointT>(segment_file, *segment) == -1) {
            cout << "Could not find annotated segment" << endl;
            exit(0);
        }
        obr.visualize_cloud(segment);
    }
}

void compute_correct_ratios(object_retrieval& obr, map<string, vector<float> >& first_correct_ratios,
                            map<string, vector<float> >& reweight_correct_ratios,
                            map<string, int>& instance_counts, map<int, string>& annotated,
                            map<int, string>& full_annotated, int number_not_annotated,
                            const string& annotations_path)
{
    //object_retrieval obr(segments_path);
    const int query_number = 11;

    map<string, int> first_matched_instances;
    map<string, int> reweight_matched_instances;
    for (pair<const string, int>& m : instance_counts) {
        first_matched_instances.insert(make_pair(m.first, 0));
        reweight_matched_instances.insert(make_pair(m.first, 0));
    }

    for (pair<const int, string>& a : full_annotated) {
        vector<index_score> first_scores;
        vector<index_score> reweight_scores;
        obr.query_reweight_vocabulary(first_scores, reweight_scores, number_not_annotated+a.first, query_number, false, number_not_annotated, annotations_path);
        string queried_instance = a.second;
        for (index_score& score : first_scores) {
            if (score.first >= number_not_annotated && annotated.count(score.first-number_not_annotated) != 0) {
                string matched_instance = annotated.at(score.first-number_not_annotated);
                if (matched_instance == queried_instance) {
                    first_matched_instances.at(queried_instance) += 1;
                }
            }
        }
        for (index_score& score : reweight_scores) {
            if (score.first >= number_not_annotated && annotated.count(score.first-number_not_annotated) != 0) {
                string matched_instance = annotated.at(score.first-number_not_annotated);
                if (matched_instance == queried_instance) {
                    reweight_matched_instances.at(queried_instance) += 1;
                }
            }
        }
    }

    for (pair<const string, int>& m : instance_counts) {
        // instance counts is also counting the instances in the partially observed =)
        // we need another, full instance counts
        float first_correct_ratio = float(first_matched_instances.at(m.first)-1) / float((query_number-1)*m.second);
        if (first_correct_ratios.count(m.first) == 0) {
            first_correct_ratios.insert(make_pair(m.first, vector<float>({first_correct_ratio})));
        }
        else {
            first_correct_ratios.at(m.first).push_back(first_correct_ratio);
        }
        float reweight_correct_ratio = float(reweight_matched_instances.at(m.first)-1) / float((query_number-1)*m.second);
        if (reweight_correct_ratios.count(m.first) == 0) {
            reweight_correct_ratios.insert(make_pair(m.first, vector<float>({reweight_correct_ratio})));
        }
        else {
            reweight_correct_ratios.at(m.first).push_back(reweight_correct_ratio);
        }
        //cout << m.first << " had ratio " << correct_ratio << endl;
    }
}

int main(int argc, char** argv)
{
    //int i = 12022; // cereal box
    //int i = 2966; // graphics box
    //int i = 4855; // kinect box
    int i = 1498; // paper clips box
    if (argc >= 2) {
        i = atoi(argv[1]);
    }

    using entity = SimpleSummaryParser::EntityStruct;
    string root_path = "/home/nbore/Data/semantic_map/";
    string segments_path = root_path + "object_segments";

    object_retrieval obr(segments_path);

    cout << "Extracted all of the features" << endl;

    //exit(0);

    // compute or load all annotations
    string annotations_path = "/home/nbore/Data/Instances/object_segments";
    map<int, string> annotated;
    map<int, string> full_annotated;
    list_all_annotated_segments(annotated, full_annotated, annotations_path);
    //visualize_annotations(annotated, annotations_path);
    map<string, int> instance_counts; // the number of fully observed instances of every type
    compute_instance_counts(instance_counts, full_annotated);
    cout << "Number of annotated segments: " << annotated.size() << endl;

    bool compute_decay = false; // this is true if we want to add more observations to see how that affects retrieval rates
    // do initial training of vocabulary, optionally only add 5000 first
    //obr.train_vocabulary_incremental(5000, compute_decay);

    int number_not_annotated;
    if (compute_decay) {
        //number_not_annotated = obr.add_others_to_vocabulary(5000, annotations_path, 22557); // 22557
    }
    else {
        //number_not_annotated = obr.add_others_to_vocabulary(5000, annotations_path);
    }
    number_not_annotated = 22557;
    cout << "number not annotated: " << number_not_annotated << endl;
    //exit(0);

    map<string, vector<float> > first_correct_ratios;
    map<string, vector<float> > reweight_correct_ratios;
    if (compute_decay) {
        bool are_done = false;
        for (int add_extra = 5000; !are_done; add_extra += 2000) {
            //i += number_not_annotated;
            cout << "segments: " << add_extra << " - " << add_extra + 2000 << endl;
            are_done = obr.add_others_to_vocabulary(2000, segments_path, add_extra, true) == 1;
            compute_correct_ratios(obr, first_correct_ratios, reweight_correct_ratios, instance_counts, annotated, full_annotated, number_not_annotated, annotations_path);
        }
    }
    else {
        compute_correct_ratios(obr, first_correct_ratios, reweight_correct_ratios, instance_counts, annotated, full_annotated, number_not_annotated, annotations_path);
    }

    for (pair<const string, vector<float> >& c : first_correct_ratios) {
        cout << c.first << " = [ ";
        for (float f : c.second) {
            cout << f << " ";
        }
        cout << ", ";
        for (float f : reweight_correct_ratios.at(c.first)) {
            cout << f << " ";
        }
        cout << " ]; " << endl;
    }

    //vector<index_score> scores;
    //obr.query_vocabulary(scores, i, 50, true, number_not_annotated, annotations_path);
}
