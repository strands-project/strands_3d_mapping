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
    //cout << metadata << endl;
    boost::filesystem::path metadata_path(metadata);
    string name = metadata_path.stem().string();
    size_t pos = name.find_last_not_of("0123456789");
    int ind = stoi(name.substr(pos+1));
    string annotations_file = metadata_path.parent_path().string() + "/annotation" + to_string(ind) + ".txt";
    //cout << annotations_file << endl;
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
