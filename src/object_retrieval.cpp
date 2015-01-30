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

#define VISUALIZE false

using namespace std;

object_retrieval::object_retrieval(const std::string& segment_path) : segment_path(segment_path)
{
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
        boost::filesystem::path sub_dir = base_dir / (string("segment") + to_string(i));
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
        boost::filesystem::path sub_dir = base_dir / (string("segment") + to_string(i));
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
            archive_i(K); // Is reading of the K matrix slowing this down?
        }
    }
}

bool object_retrieval::read_segment(CloudT::Ptr& segment, NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment,
                                    Eigen::Matrix3f& K, string& metadata, size_t segment_id)
{
    boost::filesystem::path base_dir = segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    boost::filesystem::path sub_dir = base_dir / (string("segment") + to_string(segment_id));
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
    std::string vocabulary_file = base_dir.string() + "/vocabulary.cereal";
    ofstream out(vocabulary_file, std::ios::binary);
    cereal::BinaryOutputArchive archive_o(out);
    archive_o(vt);
}

void object_retrieval::read_vocabulary(vocabulary_tree<HistT, 8>& vt)
{
    boost::filesystem::path base_dir = segment_path;//"/home/nbore/Workspace/objectness_score/object_segments";
    std::string vocabulary_file = base_dir.string() + "/vocabulary.cereal";
    {
        cout << __FILE__ << ", " << __LINE__ << endl;
        ifstream in(vocabulary_file, std::ios::binary);
        cout << __FILE__ << ", " << __LINE__ << endl;
        cereal::BinaryInputArchive archive_i(in);
        cout << __FILE__ << ", " << __LINE__ << endl;
        archive_i(vt);
        cout << __FILE__ << ", " << __LINE__ << endl;
        in.close();
        cout << __FILE__ << ", " << __LINE__ << endl;
    }
}

float object_retrieval::calculate_similarity(CloudT::Ptr& cloud1, const Eigen::Matrix3f& K1,
                           CloudT::Ptr& cloud2, const Eigen::Matrix3f& K2)
{
    register_objects ro;
    ro.set_input_clouds(cloud1, K1, cloud2, K2);
    ro.do_registration();
    return ro.get_match_score();
}

//void object_retrieval::compute_segments()
size_t object_retrieval::compute_segments(vector<CloudT::Ptr>& sweeps, vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& intrinsics, vector<string>& files, size_t i)
{
    //vector<CloudT::Ptr> sweeps;
    //vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > intrinsics;
    //read_clouds(sweeps, intrinsics, 3);

    //vector<CloudT::Ptr> segments;
    //vector<NormalCloudT::Ptr> normals;
    //vector<CloudT::Ptr> hd_segments;
    //vector<string> segment_files;

    //size_t max_segments = 5;
    size_t counter = 0;
    //size_t i = 0;
    for (CloudT::Ptr cloud : sweeps) {
        /*if (counter >= max_segments) {
            break;
        }*/

        if (VISUALIZE) {
            visualize_cloud(cloud);
        }
        convex_voxel_segmentation cvs(VISUALIZE, 0.012f, 0.02f);
        vector<CloudT::Ptr> segmentsi;
        vector<NormalCloudT::Ptr> normalsi;
        vector<CloudT::Ptr> hd_segmentsi;
        vector<string> filesi;
        cvs.segment_objects(segmentsi, normalsi, hd_segmentsi, cloud);

        //segments.insert(segments.end(), segmentsi.begin(), segmentsi.end());
        //normals.insert(normals.end(), normalsi.begin(), normalsi.end());
        //hd_segments.insert(hd_segments.end(), hd_segmentsi.begin(), hd_segmentsi.end());

        for (size_t j = 0; j < segmentsi.size(); ++j) {
            //segment_files.push_back(files[counter]);
            filesi.push_back(files[counter]);
        }
        ++counter;

        i = write_segments(segmentsi, normalsi, hd_segmentsi,  intrinsics[0], filesi, i);
    }

    //write_segments(segments, normals, hd_segments,  intrinsics[0], segment_files);
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

void object_retrieval::query_vocabulary(vector<index_score>& scores, size_t query_ind, size_t nbr_query, bool visualize_query)
{
    static HistCloudT::Ptr features(new HistCloudT);
    static vector<int> indices;

    //vocabulary_tree<HistT, 8> vt;
    HistCloudT::Ptr query_cloud(new HistCloudT);
    CloudT::Ptr segment(new CloudT);
    CloudT::Ptr query_segment(new CloudT);
    NormalCloudT::Ptr normal(new NormalCloudT);
    CloudT::Ptr hd_segment(new CloudT);
    Eigen::Matrix3f K, query_K;
    string metadata;

#if 0
    {
        vector<CloudT::Ptr> segments;
        vector<NormalCloudT::Ptr> normals;
        vector<CloudT::Ptr> hd_segments;

        read_segments(segments, normals, hd_segments, K, 200);
        HistCloudT::Ptr features(new HistCloudT);
        vector<int> indices;
        extract_features(indices, features, segments, normals, hd_segments, K);
        for (HistT& h : features->points) {
            eig(h).normalize();
        }
        vt.set_input_cloud(features, indices);
        vt.add_points_from_input_cloud();
    }
#endif

    cout << __FILE__ << ", " << __LINE__ << endl;
    if (vt.empty()) {
        read_vocabulary(vt);
    }
    cout << __FILE__ << ", " << __LINE__ << endl;

    cout << "Querying segment nbr: " << query_ind << endl;
    if(!read_segment(segment, normal, query_segment, query_K, metadata, query_ind)) { // 20 Drawer // 34 Blue Cup // 50 Monitor
        cout << "Error reading segment!" << endl;
        exit(0);
    }
    get_query_cloud(query_cloud, segment, normal, query_segment, query_K);
    for (HistT& h : query_cloud->points) {
        eig(h).normalize();
    }

    if (true) {
        if (indices.empty() || features->empty()) {
            if (!load_features(features, indices)) {
                exit(0);
            }
        }
        vt.set_input_cloud(features, indices);
        vt.compute_pyramid_match_weights();
        vt.top_pyramid_match_similarities(scores, query_cloud, nbr_query);
        return;
    }

    //vector<index_score> scores;
    vt.top_similarities(scores, query_cloud, nbr_query);

    if (visualize_query) {
        for (index_score s : scores) {
            cout << "Index: " << s.first << " with score: " << s.second << endl;
            if (!read_segment(segment, normal, hd_segment, K, metadata, s.first)) {
                cout << "Error reading segment!" << endl;
                exit(0);
            }
            visualize_cloud(hd_segment);
        }
    }

    // don't register right now
    /*for (index_score s : scores) {
        cout << "Index: " << s.first << " with score: " << s.second << endl;
        if (!read_segment(segment, normal, hd_segment, K, metadata, s.first)) {
            cout << "Error reading segment!" << endl;
            exit(0);
        }
        //float similarity = calculate_similarity(query_segment, query_K, hd_segment, K);
        //cout << "Shape similarity: " << similarity << endl;
        visualize_cloud(hd_segment);
    }*/
}

/*template <class Archive>
void serialize(Archive& archive, object_retrieval::HistT& m)
{
    archive(m.histogram);
}*/

POINT_CLOUD_REGISTER_POINT_STRUCT (object_retrieval::HistT,
                                   (float[object_retrieval::N], histogram, histogram)
)

void object_retrieval::save_features(HistCloudT::Ptr& features, std::vector<int>& indices)
{
    boost::filesystem::path base_dir = segment_path;
    std::string features_file = base_dir.string() + "/features.pcd";
    std::string indices_file = base_dir.string() + "/indices.cereal";
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
    boost::filesystem::path features_file = base_dir / "features.pcd";
    if (!boost::filesystem::is_regular_file(features_file)) {
        return false;
    }
    std::string indices_file = base_dir.string() + "/indices.cereal";
    {
        ifstream in(indices_file, std::ios::binary);
        cereal::BinaryInputArchive archive_i(in);
        archive_i(indices);
    }
    return (pcl::io::loadPCDFile<HistT>(features_file.string(), *features) != -1);
}
