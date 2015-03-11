#include "object_3d_retrieval/dataset_convenience.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf_conversions/tf_eigen.h>

#include "eigen_cereal/eigen_cereal.h"
#include "cereal/types/utility.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/string.hpp"
#include "cereal/archives/binary.hpp"

POINT_CLOUD_REGISTER_POINT_STRUCT (object_retrieval::HistT,
                                   (float[object_retrieval::N], histogram, histogram)
)

namespace dataset_convenience {

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

void compute_correct_ratios(object_retrieval& obr, map<string, vector<float> >& correct_ratios,
                            map<string, int>& instance_counts, map<int, string>& annotated,
                            map<int, string>& full_annotated, int number_not_annotated,
                            const string& annotations_path)
{
    //object_retrieval obr(segments_path);
    const int query_number = 11;

    map<string, int> matched_instances;
    for (pair<const string, int>& m : instance_counts) {
        matched_instances.insert(make_pair(m.first, 0));
    }

    for (pair<const int, string>& a : full_annotated) {
        vector<index_score> scores;
        obr.query_vocabulary(scores, number_not_annotated+a.first, query_number, false, number_not_annotated, annotations_path);
        string queried_instance = a.second;
        for (index_score& score : scores) {
            if (score.first >= number_not_annotated && annotated.count(score.first-number_not_annotated) != 0) {
                string matched_instance = annotated.at(score.first-number_not_annotated);
                if (matched_instance == queried_instance) {
                    matched_instances.at(queried_instance) += 1;
                }
            }
        }
    }

    for (pair<const string, int>& m : instance_counts) {
        // instance counts is also counting the instances in the partially observed =)
        // we need another, full instance counts
        float correct_ratio = float(matched_instances.at(m.first)-1) / float((query_number-1)*m.second);
        if (correct_ratios.count(m.first) == 0) {
            correct_ratios.insert(make_pair(m.first, vector<float>({correct_ratio})));
        }
        else {
            correct_ratios.at(m.first).push_back(correct_ratio);
        }
    }
}

void aggregate_features(object_retrieval& obr, const string& aggregate_dir)
{
    boost::filesystem::path aggregate_path(aggregate_dir);
    boost::filesystem::create_directory(aggregate_path);
    cout << "Creating directory " << aggregate_path.string() << endl;

    string previous_cloud = "";
    vector<int> segment_ids;
    HistCloudT::Ptr scan_features(new HistCloudT);
    int scan_id = 0;

    // the segments of a scan are stored consecutively
    // iterate through all our segments, aggregate them in one folder for each scan
    for (int i = 0; ; ++i) {
        string folder = obr.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(folder)) {
            boost::filesystem::path scan_path(aggregate_path / ("scan" + to_string(scan_id)));
            boost::filesystem::create_directory(scan_path);
            cout << "Creating directory " << scan_path.string() << endl;
            string scan_file = scan_path.string() + "/features.pcd"; // object_retrieval::feature_file
            string new_metadata_file = scan_path.string() + "/metadata.txt";
            pcl::io::savePCDFileBinary(scan_file, *scan_features);
            {
                ofstream f;
                f.open(new_metadata_file);
                f << previous_cloud << '\n';
                for (int j : segment_ids) {
                    f << j << " ";
                }
                f << '\n';
                f.close();
            }
            return;
        }
        cout << "Reading segment folder " << folder << endl;
        string metadata_file = folder + "/metadata.txt";
        string feature_file = folder + "/features.pcd"; // object_retrieval::feature_file
        string metadata; // in this dataset, this is the path to the scan
        {
            ifstream f;
            f.open(metadata_file);
            getline(f, metadata);
            f.close();
        }
        vector<string> strs;
        boost::split(strs, metadata, boost::is_any_of(" \t\n"));
        stringstream ss;
        ss << boost::filesystem::path(strs[0]).parent_path().string() << "/intermediate_cloud" << setw(4) << setfill('0') << stoi(strs[1]) << ".pcd";
        string cloud_file = ss.str();
        cout << "Got metadata " << ss.str() << endl;
        if (cloud_file == previous_cloud) {
            // load segment features and append to scan features
            HistCloudT::Ptr segment_features(new HistCloudT);
            if (pcl::io::loadPCDFile<HistT>(feature_file, *segment_features) == -1) {
                cout << "Could not load segment features" << endl;
                exit(0);
            }
            *scan_features += *segment_features;
            segment_ids.push_back(i);
        }
        else {
            cout << "Got new scan id " << cloud_file << endl;
            // first, save features to the path of the previous file
            if (previous_cloud != "") {
                boost::filesystem::path scan_path(aggregate_path / ("scan" + to_string(scan_id)));
                boost::filesystem::create_directory(scan_path);
                cout << "Creating directory " << scan_path.string() << endl;
                string scan_file = scan_path.string() + "/features.pcd"; // object_retrieval::feature_file
                pcl::io::savePCDFileBinary(scan_file, *scan_features);

                // also save the metadata so we know which orignal scan the features belong to
                string new_metadata_file = scan_path.string() + "/metadata.txt";
                {
                    ofstream f;
                    f.open(new_metadata_file);
                    f << previous_cloud << '\n';
                    for (int j : segment_ids) {
                        f << j << " ";
                    }
                    f << '\n';
                    f.close();
                }
                ++scan_id;
            }
            scan_features = HistCloudT::Ptr(new HistCloudT);
            if (pcl::io::loadPCDFile<HistT>(feature_file, *scan_features) == -1) {
                cout << "Could not load segment features" << endl;
                exit(0);
            }
            segment_ids.clear();
            segment_ids.push_back(i);
            previous_cloud = cloud_file;
        }
    }
}

void aggregate_pfhrgb_features(object_retrieval& obr)
{
    for (int i = 0; ; ++i) {
        string scan_path = obr.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(scan_path)) {
            break;
        }
        string supervoxel_paths_file = scan_path + "/segment_paths.txt";
        HistCloudT::Ptr scan_cloud(new HistCloudT);
        vector<string> supervoxel_paths; // in this dataset, this is the path to the scan
        {
            ifstream f;
            string metadata;
            f.open(supervoxel_paths_file);
            while (getline(f, metadata)) {
                if (metadata.length() > 1) {
                    supervoxel_paths.push_back(metadata);
                }
            }
            f.close();
        }
        for (string supervoxel_path : supervoxel_paths) {
            HistCloudT::Ptr supervoxel_cloud(new HistCloudT);
            string supervoxel_cloud_file = supervoxel_path + "/pfhrgb_cloud_1.pcd";
            if (pcl::io::loadPCDFile<HistT>(supervoxel_cloud_file, *supervoxel_cloud) == -1) {
                cout << "Could not load segment features" << endl;
                continue;
            }
            scan_cloud->insert(scan_cloud->end(), supervoxel_cloud->begin(), supervoxel_cloud->end());
        }
        string scan_cloud_file = scan_path + "/pfhrgb_cloud_1.pcd";
        pcl::io::savePCDFileBinary(scan_cloud_file, *scan_cloud);
    }
}

string annotation_for_scan(int i, object_retrieval& obr)
{
    string folder = obr.get_folder_for_segment_id(i);
    string metadata_file = folder + "/metadata.txt";
    string metadata; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(metadata_file);
        getline(f, metadata);
        f.close();
    }
    cout << metadata << endl;
    boost::filesystem::path metadata_path(metadata);
    string name = metadata_path.stem().string();
    size_t pos = name.find_last_not_of("0123456789");
    int ind = stoi(name.substr(pos+1));
    string annotations_file = metadata_path.parent_path().string() + "/annotation" + to_string(ind) + ".txt";
    cout << annotations_file << endl;
    string annotation;
    {
        ifstream f;
        f.open(annotations_file);
        f >> annotation;
        f.close();
    }
    return annotation;
}

} // namespace dataset_convenience
