#include <iostream>
#include <chrono>

#include <pcl/kdtree/kdtree_flann.h>

#include "object_3d_retrieval/supervoxel_segmentation.h"
#include "object_3d_retrieval/object_retrieval.h"
#include "object_3d_retrieval/dataset_convenience.h"
#include "object_3d_retrieval/register_objects.h"
#include "object_3d_retrieval/segment_features.h"
#include "object_3d_retrieval/pfhrgb_estimation.h"
#include "sift/sift.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <cereal/archives/binary.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>

using namespace std;
using namespace dataset_convenience;

//using sift_point = pcl::Histogram<128>;
//using sift_cloud = pcl::PointCloud<sift_point>;

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<128>,
                                   (float[128], histogram, histogram)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<256>,
                                   (float[256], histogram, histogram)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<250>,
                                   (float[250], histogram, histogram)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<1344>,
                                   (float[1344], histogram, histogram)
)

using SiftT = pcl::Histogram<128>;
using SiftCloudT = pcl::PointCloud<SiftT>;
using DupletT = pcl::Histogram<256>;
using DupletCloudT = pcl::PointCloud<DupletT>;
using PfhRgbT = pcl::Histogram<250>;
using PfhRgbCloudT = pcl::PointCloud<PfhRgbT>;

//POINT_CLOUD_REGISTER_POINT_STRUCT (object_retrieval::HistT,
//                                   (float[object_retrieval::N], histogram, histogram)
//)

template<typename Key, typename Value>
void inverse_map(map<Value, vector<Key> >& imap, const map<Key, Value>& omap)
{
    for (const pair<const Key, Value>& v : omap) {
        imap[v.second].push_back(v.first);
    }
}

void calculate_sift_features(cv::Mat& descriptors, vector<cv::KeyPoint>& keypoints, CloudT::Ptr& cloud, cv::Mat& image,
                             cv::Mat& depth, int minx, int miny, const Eigen::Matrix3f& K)
{
    cv::SIFT::DetectorParams detector_params;
    detector_params.edgeThreshold = 15.0; // 10.0 default
    detector_params.threshold = 0.04; // 0.04 default
    cv::SiftFeatureDetector detector(detector_params);
    detector.detect(image, keypoints);

    //-- Step 2: Calculate descriptors (feature vectors)
    //cv::Mat descriptors;
    // the length of the descriptors is 128
    // the 3D sift keypoint detectors are probably very unreliable
    cv::SIFT::DescriptorParams descriptor_params;
    descriptor_params.isNormalize = true; // always true, shouldn't matter
    descriptor_params.magnification = 3.0; // 3.0 default
    descriptor_params.recalculateAngles = true; // true default

    cv::SiftDescriptorExtractor extractor;
    extractor.compute(image, keypoints, descriptors);

    // get back to 3d coordinates
    for (cv::KeyPoint k : keypoints) {
        cv::Point2f p2 = k.pt;
        Eigen::Vector3f p3;
        p3(0) = p2.x + float(minx);
        p3(1) = p2.y + float(miny);
        p3(2) = 1.0f;
        p3 = K.colPivHouseholderQr().solve(p3);
        p3 *= depth.at<float>(int(p2.y), int(p2.x))/p3(2);
        PointT p;
        p.getVector3fMap() = p3;
        cloud->push_back(p);
    }

    cv::Mat img_keypoints;
    cv::drawKeypoints(image, keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("my keypoints", img_keypoints);
    cv::waitKey(0);
}

void shot_features_for_segment(HistCloudT::Ptr& desc_cloud, CloudT::Ptr& cloud, Eigen::Matrix3f& K, int points=0)
{
    segment_features sf(K, false);
    sf.compute_shot_features(desc_cloud, cloud, points != 0, points);
}

void save_supervoxel_features(object_retrieval& obr)
{
    using Graph = supervoxel_segmentation::Graph;

    string base_path = boost::filesystem::path(obr.segment_path).parent_path().string();
    string segment_path = base_path + "/supervoxel_segments";
    boost::filesystem::create_directory(segment_path);

    Eigen::Matrix3f K; // dummy

    bool found = false;
    int counter = 0;
    for (int i = 0; ; ++i) {
        string folder = obr.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(folder)) {
            return;
        }
        CloudT::Ptr cloud(new CloudT);
        string scan_file = obr.get_scan_file(i);
        if (!found) {
            if (scan_file != "/home/nbore/Data/Instances//muesli_1/patrol_run_5/room_1/intermediate_cloud0010.pcd") {
                ++counter;
                continue;
            }
            else {
                found = true;
                counter = 28114;
            }
        }

        cout << "Finally got scan " << scan_file << endl;
        obr.read_scan(cloud, i);
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

            HistCloudT::Ptr desc_cloud(new HistCloudT);
            shot_features_for_segment(desc_cloud, sv, K);

            if (desc_cloud->empty()) {
                ++j;
                continue;
            }

            string segment_folder = segment_path + "/segment" + to_string(counter);
            boost::filesystem::create_directory(segment_folder);
            segment_folders.push_back(segment_folder);

            // save features, maybe also save the segment itself? yes
            string features_file = segment_folder + "/features.pcd";
            pcl::io::savePCDFileBinary(features_file, *desc_cloud);

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

void cloud_for_scan(CloudT::Ptr& cloud, int i, object_retrieval& obr)
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
    if (pcl::io::loadPCDFile(metadata, *cloud) == -1) {
        cout << "Could not load scan point cloud..." << endl;
        exit(-1);
    }
}

void compute_feature_duplets(DupletCloudT::Ptr& duplets, SiftCloudT::Ptr& desc_cloud, CloudT::Ptr& kp_cloud)
{
    float radius = 0.1;
    float plus_minus = 0.01;

    set<pair<int, int> > used;

    auto flip_pair = [](int i, int j) -> pair<int, int> {
        if (i > j) {
            return make_pair(i, j);
        }
        else {
            return make_pair(j, i);
        }
    };

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(kp_cloud);
    size_t counter = 0;
    size_t nbr_not_finite = 0;
    for (PointT& p : kp_cloud->points) {
        // find all points close to a certain radius
        if (!pcl::isFinite(p)) {
            ++counter;
            ++nbr_not_finite;
            continue;
        }

        vector<int> inds;
        vector<float> dists;
        //int max_nbr = 100;
        kdtree.radiusSearch(p, radius+plus_minus, inds, dists);

        size_t _inds_size = inds.size();
        for (size_t i = 0; i < _inds_size; ++i) {
            if (sqrt(dists[i]) < radius-plus_minus) {
                continue;
            }
            if (used.count(flip_pair(counter, inds[i])) == 1) {
                continue;
            }
            DupletT duplet;
            for (size_t j = 0; j < 128; ++j) {
                duplet.histogram[j] = desc_cloud->at(counter).histogram[j];
                duplet.histogram[128+j] = desc_cloud->at(inds[i]).histogram[j];
            }
            duplets->push_back(duplet);
            used.insert(flip_pair(counter, inds[i]));
        }

        ++counter;
    }

    cout << "Number not finite: " << nbr_not_finite << endl;
}

void save_duplet_features(object_retrieval& obr)
{
    for (int i = 0; ; ++i) {
        string folder = obr.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(folder)) {
            cout << folder << " is not a directory!" << endl;
            break;
        }
        SiftCloudT::Ptr features(new SiftCloudT);
        //obr.load_features_for_segment(features, i);
        DupletCloudT::Ptr duplet_cloud(new DupletCloudT);
        CloudT::Ptr kp_cloud(new CloudT);
        string sift_points_file = folder + "/sift_points_file.pcd";
        if (pcl::io::loadPCDFile(sift_points_file, *kp_cloud) == -1) {
            cout << "Could not load keypoint cloud" << endl;
            exit(0);
        }
        compute_feature_duplets(duplet_cloud, features, kp_cloud);
        cout << "Features size: " << features->size() << endl;
        cout << "Keypoints size: " << kp_cloud->size() << endl;
        cout << "Duplets size: " << duplet_cloud->size() << endl;
        string duplet_points_file = folder + "/duplet_points_file.pcd";
        pcl::io::savePCDFileBinary(duplet_points_file, *duplet_cloud);
    }
}

void save_sift_features(object_retrieval& obr)
{
    for (int i = 0; ; ++i) {
        string folder = obr.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(folder)) {
            cout << folder << " is not a directory!" << endl;
            break;
        }
        CloudT::Ptr cloud(new CloudT);
        obr.read_scan(cloud, i);
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
        HistCloudT::Ptr desc_cloud(new HistCloudT);
        int j = 0;
        for (cv::KeyPoint k : keypoints) {
            cv::Point2f p2 = k.pt;
            int ind = p2.y*640+p2.x;
            PointT p = cloud->at(ind);
            if (pcl::isFinite(p)) {
                kp_cloud->push_back(p);
                HistT sp;
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

void sift_features_for_segment(SiftCloudT::Ptr& desc_cloud, CloudT::Ptr& kp_cloud, CloudT::Ptr& cloud, Eigen::Matrix3f& K)
{
    int minx, miny;
    cv::Mat img, depth;
    register_objects ro;
    tie(minx, miny) = ro.calculate_image_for_cloud(img, depth, cloud, K);
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    calculate_sift_features(descriptors, keypoints, kp_cloud, img, depth, minx, miny, K);//ro.calculate_features_for_image(descriptors, keypoints, kp_cloud, img, depth, minx, miny, K);
    int j = 0;
    for (cv::KeyPoint& k  : keypoints) {
        // we need to check if the points are finite
        SiftT sp;
        for (int k = 0; k < 128; ++k) {
            sp.histogram[k] = descriptors.at<float>(j, k);
        }
        desc_cloud->push_back(sp);
        ++j;
    }
}

bool scan_contains_segment(int i, int segment, object_retrieval& obr)
{
    string folder = obr.get_folder_for_segment_id(i);
    string metadata_file = folder + "/metadata.txt";
    string metadata; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(metadata_file);
        getline(f, metadata);
        getline(f, metadata);
        f.close();
    }
    stringstream ss(metadata);
    vector<int> numbers((istream_iterator<int>(ss)), istream_iterator<int>());
    return (find(numbers.begin(), numbers.end(), segment) != numbers.end());
}

pair<bool, bool> supervoxel_is_correct(CloudT::Ptr& cloud, const Eigen::Matrix3f& K, int minx, int maxx, int miny, int maxy)
{
    cv::Mat cover = cv::Mat::zeros(maxy - miny + 1, maxx - minx + 1, CV_32SC1);
    size_t counter = 0;
    for (PointT& p : cloud->points) {
        Eigen::Vector3f q = K*p.getVector3fMap();
        int x = int(q(0)/q(2) + 0.5f);
        int y = int(q(1)/q(2) + 0.5f);
        if (x >= minx && x <= maxx && y >= miny && y <= maxy) {
            cover.at<int>(y - miny, x - minx) = 1;
            ++counter;
        }
    }
    int allpixels = cv::sum(cover)[0];
    float segment_cover = float(allpixels)/float((maxx-minx)*(maxy-miny));
    float annotation_cover = float(counter)/float(cloud->size());
    return make_pair(annotation_cover > 0.75, segment_cover > 0.5);
}

struct voxel_annotation {
    int segment_id;
    string segment_file;
    string scan_folder;
    int scan_id;
    string annotation;
    bool full;
    bool segment_covered;
    bool annotation_covered;
    template <typename Archive>
    void serialize(Archive& archive)
    {
        archive(segment_id, segment_file, scan_folder, scan_id, annotation, full, segment_covered, annotation_covered);
    }
};

int scan_ind_for_supervoxel(int i, object_retrieval& obr)
{
    string segment_folder = obr.get_folder_for_segment_id(i);
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

// scan folder, scan number, annotation, covered
voxel_annotation scan_for_supervoxel(int i, const Eigen::Matrix3f& K, object_retrieval& obr)
{
    string segment_folder = obr.get_folder_for_segment_id(i);
    string metadata_file = segment_folder + "/metadata.txt";
    string metadata; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(metadata_file);
        getline(f, metadata);
        getline(f, metadata);
        f.close();
    }
    string scan_folder = boost::filesystem::path(metadata).parent_path().string();
    string scan_name = boost::filesystem::path(metadata).stem().string();
    size_t pos = scan_name.find_last_not_of("0123456789");
    int ind = stoi(scan_name.substr(pos+1));
    string annotation_file = scan_folder + "/annotation" + to_string(ind) + ".txt";
    string annotation; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(annotation_file);
        getline(f, annotation);
        f.close();
    }
    bool annotation_covered = false;
    bool segment_covered = false;
    bool full = false;
    string cloud_file = segment_folder + "/segment.pcd";
    if (annotation != "null") {
        vector<string> strs;
        boost::split(strs, annotation, boost::is_any_of(" \t\n"));
        annotation = strs[0];
        full = (strs[1] == "full");
        int minx = stoi(strs[2]);
        int maxx = stoi(strs[3]);
        int miny = stoi(strs[4]);
        int maxy = stoi(strs[5]);
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(cloud_file, *cloud);
        tie(segment_covered, annotation_covered) = supervoxel_is_correct(cloud, K, minx, maxx, miny, maxy);
    }

    return voxel_annotation { i, cloud_file, scan_folder, ind, annotation, full, segment_covered, annotation_covered };
}

string annotation_for_supervoxel(int i, object_retrieval& obr)
{
    string segment_folder = obr.get_folder_for_segment_id(i);
    string metadata_file = segment_folder + "/metadata.txt";
    string metadata; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(metadata_file);
        getline(f, metadata);
        getline(f, metadata);
        f.close();
    }
    string scan_folder = boost::filesystem::path(metadata).parent_path().string();
    string scan_name = boost::filesystem::path(metadata).stem().string();
    size_t pos = scan_name.find_last_not_of("0123456789");
    int ind = stoi(scan_name.substr(pos+1));
    string annotation_file = scan_folder + "/annotation" + to_string(ind) + ".txt";
    string annotation; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(annotation_file);
        getline(f, annotation);
        f.close();
    }
    if (annotation != "null") {
        vector<string> strs;
        boost::split(strs, annotation, boost::is_any_of(" \t\n"));
        annotation = strs[0];
    }
    return annotation;
}

void list_annotated_supervoxels(vector<voxel_annotation>& annotations, const string& annotations_file,
                                const Eigen::Matrix3f& K, object_retrieval& obr)
{
    if (boost::filesystem::is_regular_file(annotations_file)) {
        ifstream in(annotations_file, std::ios::binary);
        cereal::BinaryInputArchive archive_i(in);
        archive_i(annotations);
        return;
    }
    for (int i = 0; ; ++i) {
        string segment_folder = obr.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(segment_folder)) {
            break;
        }
        voxel_annotation annotation = scan_for_supervoxel(i, K, obr);
        annotations.push_back(annotation);
    }
    {
        ofstream out(annotations_file, std::ios::binary);
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(annotations);
    }
}

void visualize_supervoxel_annotation(vector<voxel_annotation>& annotations, object_retrieval& obr)
{
    for (voxel_annotation& a : annotations) {
        if (!a.annotation_covered || !a.segment_covered) {
            continue;
        }
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(a.segment_file, *cloud);
        cout << "Showing " << a.segment_file << " with annotation " << a.annotation << endl;
        obr.visualize_cloud(cloud);
    }
}

void save_sift_features_for_supervoxels(Eigen::Matrix3f& K, object_retrieval& obr)
{
    for (int i = 0; ; ++i) {
        string segment_folder = obr.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(segment_folder)) {
            break;
        }
        string cloud_file = segment_folder + "/segment.pcd";
        CloudT::Ptr cloud(new CloudT);
        if (pcl::io::loadPCDFile(cloud_file, *cloud) == -1) {
            cout << "Could not load " << cloud_file << endl;
            exit(0);
        }
        SiftCloudT::Ptr desc_cloud(new SiftCloudT);
        CloudT::Ptr kp_cloud(new CloudT);
        sift_features_for_segment(desc_cloud, kp_cloud, cloud, K);

        string sift_file = segment_folder + "/sift_cloud.pcd";
        string sift_points_file = segment_folder + "/sift_points_file.pcd";

        if (desc_cloud->empty()) {
            // push back one inf point on descriptors and keypoints
            SiftT sp;
            for (int i = 0; i < 128; ++i) {
                sp.histogram[i] = std::numeric_limits<float>::infinity();
            }
            desc_cloud->push_back(sp);
            PointT p;
            p.x = p.y = p.z = std::numeric_limits<float>::infinity();
            kp_cloud->push_back(p);
        }

        pcl::io::savePCDFileBinary(sift_file, *desc_cloud);
        pcl::io::savePCDFileBinary(sift_points_file, *kp_cloud);
    }
}

void save_pfhrgb_features_for_supervoxels(object_retrieval& obr)
{
    for (int i = 0; ; ++i) {
        string segment_folder = obr.get_folder_for_segment_id(i);
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
        pfhrgb_estimation::compute_pfhrgb_features(desc_cloud, kp_cloud, cloud, false);

        string pfhrgb_file = segment_folder + "/pfhrgb_cloud_2.pcd";
        string pfhrgb_points_file = segment_folder + "/pfhrgb_points_file_2.pcd";

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

void save_split_features(object_retrieval& obr)
{
    for (int i = 0; ; ++i) {
        string segment_folder = obr.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(segment_folder)) {
            break;
        }
        // we should add a keypoints argument to object_retrieval
        string pfhrgb_file = segment_folder + "/pfhrgb_cloud.pcd";
        string pfhrgb_points_file = segment_folder + "/pfhrgb_points_file.pcd";
        PfhRgbCloudT::Ptr desc_cloud(new PfhRgbCloudT);
        CloudT::Ptr kp_cloud(new CloudT);
        if (pcl::io::loadPCDFile(pfhrgb_file, *desc_cloud) == -1) {
            cout << "Could not load " << pfhrgb_file << endl;
            exit(0);
        }
        if (pcl::io::loadPCDFile(pfhrgb_points_file, *kp_cloud) == -1) {
            cout << "Could not load " << pfhrgb_points_file << endl;
            exit(0);
        }
        vector<PfhRgbCloudT::Ptr> split_features;
        pfhrgb_estimation::split_descriptor_points(split_features, desc_cloud, kp_cloud, 30);
        int counter = 0;
        for (PfhRgbCloudT::Ptr& split_cloud : split_features) {
            if (split_cloud->empty()) {
                continue;
            }
            if (desc_cloud->size() >= 20 && split_cloud->size() < 20) {
                cout << "Doing cloud: " << pfhrgb_file << endl;
                cout << "Split cloud had size: " << split_cloud->size() << endl;
                exit(0);
            }
            string split_file = segment_folder + "/split_features" + to_string(counter) + ".pcd";
            pcl::io::savePCDFile(split_file, *split_cloud);
            ++counter;
        }
    }
}

void get_voxels_for_scan(vector<int>& voxels, int i, object_retrieval& obr_scans)
{
    string scan_path = obr_scans.get_folder_for_segment_id(i);
    string segment_paths_file = scan_path + "/segment_paths.txt";
    string metadata; // in this dataset, this is the path to the scan
    {
        ifstream f;
        f.open(segment_paths_file);
        while (getline(f, metadata)) {
            size_t pos = metadata.find_last_not_of("0123456789");
            int ind = stoi(metadata.substr(pos+1));
            voxels.push_back(ind);
        }
        f.close();
    }
}

void get_oversegmented_voxels_for_scan(vector<HistCloudT::Ptr>& voxels, CloudT::Ptr& voxel_centers,
                                       vector<double>& vocabulary_norms, int i, object_retrieval& obr_scans)
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
            string oversegment_path = voxel_path + "/split_features" + to_string(i) + ".pcd";
            if (!boost::filesystem::is_regular_file(oversegment_path)) {
                break;
            }
            voxels.push_back(HistCloudT::Ptr(new HistCloudT));
            if (pcl::io::loadPCDFile(oversegment_path, *voxels.back()) == -1) {
                cout << "Could not read oversegment file " << oversegment_path << endl;
                exit(0);
            }

            string center_path = voxel_path + "/split_center" + to_string(i) + ".pcd";
            CloudT split_cloud;
            if (pcl::io::loadPCDFile(center_path, split_cloud) == -1) {
                cout << "Could not read center file " << center_path << endl;
                exit(0);
            }
            if (split_cloud.size() != 1) {
                cout << "Cloud did not have size 1: " << center_path << endl;
                exit(0);
            }
            voxel_centers->push_back(split_cloud.at(0));
        }
    }

    string norms_file = scan_path + "/vocabulary_norms.cereal";
    ifstream in(norms_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(vocabulary_norms);
    }
}

void get_oversegmented_vectors_for_scan(CloudT::Ptr& voxel_centers, vector<double>& vocabulary_norms,
                                        vector<map<int, double> >& vocabulary_vectors, int i, object_retrieval& obr_scans)
{
    string scan_path = obr_scans.get_folder_for_segment_id(i);

    string centers_file = scan_path + "/split_centers.pcd";
    if (pcl::io::loadPCDFile(centers_file, *voxel_centers) == -1) {
        cout << "Could not read file " << centers_file << endl;
        exit(0);
    }

    string norms_file = scan_path + "/vocabulary_norms.cereal";
    ifstream inn(norms_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(inn);
        archive_i(vocabulary_norms);
    }

    string vectors_file = scan_path + "/vocabulary_vectors.cereal";
    ifstream inv(vectors_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(inv);
        archive_i(vocabulary_vectors);
    }
}

void save_oversegmented_vocabulary_vectors(object_retrieval& obr_scans)
{
    for (int i = 0; ; ++i) {
        string path = obr_scans.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(path)) {
            break;
        }
        vector<HistCloudT::Ptr> voxels;
        CloudT::Ptr voxel_centers(new CloudT);
        //get_oversegmented_voxels_for_scan(voxels, voxel_centers, i, obr_scans);
        vector<double> vocabulary_norms;
        for (HistCloudT::Ptr& voxel : voxels) {
            double pnorm = obr_scans.rvt.compute_vocabulary_norm(voxel);
            vocabulary_norms.push_back(pnorm);
        }
        string norms_file = path + "/vocabulary_norms.cereal";
        ofstream out(norms_file, std::ios::binary);
        {
            cereal::BinaryOutputArchive archive_o(out);
            archive_o(vocabulary_norms);
        }
    }
}

void save_supervoxel_centers_for_scan(int i, object_retrieval& obr_scans)
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
            string oversegment_path = voxel_path + "/split_points" + to_string(i) + ".pcd";
            if (!boost::filesystem::is_regular_file(oversegment_path)) {
                break;
            }
            CloudT::Ptr cloud(new CloudT);
            if (pcl::io::loadPCDFile(oversegment_path, *cloud) == -1) {
                cout << "Could not read oversegment file " << oversegment_path << endl;
                exit(0);
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
            CloudT::Ptr center_cloud(new CloudT);
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
            string center_path = voxel_path + "/split_center" + to_string(i) + ".pcd";
            pcl::io::savePCDFileBinary(center_path, *center_cloud);
        }
    }
}

void save_supervoxel_centers(object_retrieval& obr_scans)
{

    for (int i = 0; ; ++i) {
        string path = obr_scans.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(path)) {
            break;
        }
        save_supervoxel_centers_for_scan(i, obr_scans);
    }
}

void convert_voxels_to_binary(object_retrieval& obr_scans)
{

    for (int i = 0; ; ++i) {
        string path = obr_scans.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(path)) {
            break;
        }
        vector<HistCloudT::Ptr> dummy;
        CloudT::Ptr dummy_centers(new CloudT);
        vector<double> dummy_norms;
        get_oversegmented_voxels_for_scan(dummy, dummy_centers, dummy_norms, i, obr_scans);
    }
}

void save_oversegmented_voxels_for_scan(int i, object_retrieval& obr_scans)
{
    vector<HistCloudT::Ptr> voxels;

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
        string features_path = voxel_path + "/pfhrgb_cloud.pcd";
        HistCloudT::Ptr features(new HistCloudT);
        if (pcl::io::loadPCDFile(features_path, *features) == -1) {
            cout << "Could not read features file " << features_path << endl;
            exit(0);
        }

        string point_path = voxel_path + "/pfhrgb_points_file.pcd";
        CloudT::Ptr points(new CloudT);
        if (pcl::io::loadPCDFile(point_path, *points) == -1) {
            cout << "Could not read points file " << point_path << endl;
            exit(0);
        }

        for (int i = 0; ; ++i) {
            string oversegment_path = voxel_path + "/split_features" + to_string(i) + ".pcd";
            if (!boost::filesystem::is_regular_file(oversegment_path)) {
                break;
            }
            voxels.push_back(HistCloudT::Ptr(new HistCloudT));
            if (pcl::io::loadPCDFile(oversegment_path, *voxels.back()) == -1) {
                cout << "Could not read oversegment file " << oversegment_path << endl;
                exit(0);
            }

            string oversegments_points_path = voxel_path + "/split_points" + to_string(i) + ".pcd";
            CloudT::Ptr split_points(new CloudT);
            for (HistT& h : voxels.back()->points) {
                // find h in features
                bool found = false;
                for (int j = 0; j < features->size(); ++j) {
                    if ((eig(h).isApprox(eig(features->at(j)), 1e-30))) {
                        split_points->push_back(points->at(j));
                        /*cout << "Found matching points:" << endl;
                        cout << eig(h).transpose() << endl;
                        cout << eig(features->at(j)).transpose() << endl;*/
                        found = true;
                        break;
                    }
                }
                if (features->size() > 0 && !found) {
                    cout << "Could not find point in global cloud " << features_path << endl;
                    cout << "Global cloud size: " << features->size() << endl;
                    cout << eig(h).transpose() << endl;
                    //exit(0);
                    PointT p;
                    p.x = std::numeric_limits<float>::infinity();
                    p.y = std::numeric_limits<float>::infinity();
                    p.z = std::numeric_limits<float>::infinity();
                    split_points->push_back(p);
                }
            }

            pcl::io::savePCDFileBinary(oversegments_points_path, *split_points);
        }
    }
}

void save_oversegmented_points(object_retrieval& obr_scans)
{
    for (int i = 0; ; ++i) {
        string path = obr_scans.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(path)) {
            break;
        }
        save_oversegmented_voxels_for_scan(i, obr_scans);
    }
}

vector<index_score> top_combined_similarities(map<int, double>& larger_map, map<int, double>& smaller_map, int nbr_query, float& ratio, int nbr_features);

void query_supervoxel_annotations(vector<voxel_annotation>& annotations, object_retrieval& obr)
{
    const int nbr_query = 11;

    /*if (obr.gvt.empty()) {
        obr.read_vocabulary(obr.gvt);
    }
    obr.gvt.set_min_match_depth(3);
    obr.gvt.compute_normalizing_constants();*/

    if (obr.vt.empty()) {
        obr.read_vocabulary(obr.vt);
    }
    obr.vt.set_min_match_depth(3);
    obr.vt.compute_normalizing_constants();

    map<string, int> nbr_full_instances;
    map<string, pair<float, int> > instance_correct_ratios;

    for (voxel_annotation& a : annotations) {
        if (a.full) {
            nbr_full_instances[a.annotation] += 1;
        }
        if (!a.full || !a.annotation_covered || !a.segment_covered) {
            continue;
        }
        if (a.annotation != "ajax_1" && a.annotation != "ajax_2" && a.annotation != "ball_1") {
            //continue;
        }
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(a.segment_file, *cloud);

        HistCloudT::Ptr features(new HistCloudT);
        obr.load_features_for_segment(features, a.segment_id);
        vector<index_score> larger_scores;
        //obr.gvt.top_grouped_similarities(scores, features, nbr_query);
        obr.vt.top_larger_similarities(larger_scores, features, 0);
        vector<index_score> smaller_scores;
        obr.vt.top_smaller_similarities(smaller_scores, features, 0);

        map<int, double> larger_map((larger_scores.begin()), larger_scores.end());
        map<int, double> smaller_map((smaller_scores.begin()), smaller_scores.end());

        float max_ratio;
        vector<index_score> scores = top_combined_similarities(larger_map, smaller_map, nbr_query, max_ratio, features->size());

        bool found = false;
        int counter = 0;
        int partial_counter = 0;
        bool last_match;
        for (index_score s : scores) {
            //CloudT::Ptr segment(new CloudT);
            //obr.read_scan(segment, s.first);
            //obr.visualize_cloud(segment);
            string instance = annotation_for_supervoxel(s.first, obr);
            if (s.first == a.segment_id) {
                found = true;
                continue;
            }
            last_match = false;
            if (instance == a.annotation) {
                ++counter;
                last_match = true;
                if (!a.full) {
                    ++partial_counter;
                }
                cout << "This was true." << endl;
            }
            else {
                cout << "This was false." << endl;
            }
            HistCloudT::Ptr features_match(new HistCloudT);
            obr.load_features_for_segment(features_match, s.first);
            cout << "Score: " << s.second << endl;
            cout << "Number of features: " << features_match->size() << endl;
        }

        if (!found) {
            if (last_match) {
                --counter;
            }
        }

        float correct_ratio = float(counter)/float(scores.size()-1);
        instance_correct_ratios[a.annotation].first += correct_ratio;
        instance_correct_ratios[a.annotation].second += 1;

        cout << "Showing " << a.segment_file << " with annotation " << a.annotation << endl;
        cout << "Number of features: " << features->size() << endl;
        cout << "Correct ratio: " << correct_ratio << endl;
        cout << "Partial ratio: " << float(partial_counter)/float(counter) << endl;

        //obr.visualize_cloud(cloud);
    }

    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
    }
}

void calculate_correct_ratio(map<string, pair<float, int> >& instance_correct_ratios, voxel_annotation& a,
                             int scan_ind, vector<index_score>& scores, object_retrieval& obr_scans, const bool verbose = true)
{
    bool found = false;
    int counter = 0;
    int partial_counter = 0;
    bool last_match;
    for (index_score s : scores) {
        string instance = annotation_for_scan(s.first, obr_scans);
        if (s.first == scan_ind) {
            found = true;
            continue;
        }
        last_match = false;
        if (instance == a.annotation) {
            ++counter;
            last_match = true;
            if (!a.full) {
                ++partial_counter;
            }
            if (verbose) cout << "This was true." << endl;
        }
        else {
            if (verbose) cout << "This was false." << endl;
        }
        HistCloudT::Ptr features_match(new HistCloudT);
        obr_scans.load_features_for_segment(features_match, s.first);
        if (verbose) {
            cout << "Score: " << s.second << endl;
            cout << "Number of features: " << features_match->size() << endl;
        }
    }

    if (!found) {
        if (last_match) {
            --counter;
        }
    }

    float correct_ratio = float(counter)/float(scores.size()-1);
    instance_correct_ratios[a.annotation].first += correct_ratio;
    instance_correct_ratios[a.annotation].second += 1;

    if (verbose) {
        cout << "Showing " << a.segment_file << " with annotation " << a.annotation << endl;
        cout << "Correct ratio: " << correct_ratio << endl;
        cout << "Partial ratio: " << float(partial_counter)/float(counter) << endl;
    }
}

void query_supervoxel_annotations_scans(vector<voxel_annotation>& annotations, Eigen::Matrix3f& K,
                                        object_retrieval& obr, object_retrieval& obr_scans)
{
    const int nbr_query = 11;

    if (obr_scans.rvt.empty()) {
        obr_scans.read_vocabulary(obr_scans.rvt);
    }

    obr_scans.rvt.set_min_match_depth(3);
    obr_scans.rvt.compute_normalizing_constants();

    map<string, int> nbr_full_instances;
    map<string, pair<float, int> > instance_correct_ratios;
    map<string, pair<float, int> > reweight_correct_ratios;

    map<string, pair<int, int> > instance_mean_features;

    map<string, int> instance_number_queries;

    for (voxel_annotation& a : annotations) {
        if (a.full) {
            nbr_full_instances[a.annotation] += 1;
        }
        if (!a.full || !a.annotation_covered || !a.segment_covered) {
            continue;
        }
        if (a.annotation != "ajax_1" && a.annotation != "ajax_2" && a.annotation != "ball_1") {
            //continue;
        }

        instance_number_queries[a.annotation] += 1;

        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(a.segment_file, *cloud);

        HistCloudT::Ptr features(new HistCloudT);
        obr.load_features_for_segment(features, a.segment_id);
        vector<index_score> scores;
        vector<index_score> reweight_scores;
        obr_scans.rvt.top_larger_similarities(scores, features, nbr_query);
        obr_scans.rvt.top_smaller_similarities(reweight_scores, features, nbr_query);
        //obr_scans.query_reweight_vocabulary(scores, reweight_scores, features, cloud, K, nbr_query);

        int scan_ind = scan_ind_for_supervoxel(a.segment_id, obr);
        calculate_correct_ratio(instance_correct_ratios, a, scan_ind, scores, obr_scans);
        calculate_correct_ratio(reweight_correct_ratios, a, scan_ind, reweight_scores, obr_scans);
        cout << "Number of features: " << features->size() << endl;

        instance_mean_features[a.annotation].first += features->size();
        instance_mean_features[a.annotation].second += 1;

        //obr.visualize_cloud(cloud);
    }

    cout << "First round correct ratios: " << endl;
    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
        cout << "Mean features: " << float(instance_mean_features[c.first].first)/float(instance_mean_features[c.first].second) << endl;
        cout << "Number of queries: " << instance_number_queries[c.first] << endl;
    }

    cout << "Reweight round correct ratios: " << endl;
    for (pair<const string, pair<float, int> > c : reweight_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
    }
}

void save_oversegmented_vocabulary_index_vectors(object_retrieval& obr_scans)
{
    map<vocabulary_tree<HistT, 8>::node*, int> mapping;
    obr_scans.rvt.get_node_mapping(mapping);

    for (int i = 0; ; ++i) {
        string path = obr_scans.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(path)) {
            break;
        }
        vector<HistCloudT::Ptr> voxels;
        CloudT::Ptr voxel_centers(new CloudT);
        vector<double> voxel_norms;
        get_oversegmented_voxels_for_scan(voxels, voxel_centers, voxel_norms, i, obr_scans);
        vector<map<int, double> > vocabulary_vectors;
        for (HistCloudT::Ptr& voxel : voxels) {
            vocabulary_vectors.push_back(map<int, double>());
            double pnorm = obr_scans.rvt.compute_query_index_vector(vocabulary_vectors.back(), voxel, mapping);
            //vocabulary_norms.push_back(pnorm);
        }
        string vectors_file = path + "/vocabulary_vectors.cereal";
        ofstream out(vectors_file, std::ios::binary);
        {
            cereal::BinaryOutputArchive archive_o(out);
            archive_o(vocabulary_vectors);
        }
    }
}

void query_supervoxel_annotations_oversegments(vector<voxel_annotation>& annotations, Eigen::Matrix3f& K,
                                               object_retrieval& obr, object_retrieval& obr_scans)
{
    const int nbr_query = 11;
    const int nbr_intial_query = 1000;

    if (obr_scans.rvt.empty()) {
        obr_scans.read_vocabulary(obr_scans.rvt);
    }
    obr_scans.rvt.set_min_match_depth(1);
    obr_scans.rvt.compute_normalizing_constants();

    map<vocabulary_tree<HistT, 8>::node*, int> mapping;
    obr_scans.rvt.get_node_mapping(mapping);

    //save_oversegmented_vocabulary_vectors(obr_scans);
    //save_oversegmented_vocabulary_index_vectors(obr_scans);
    //exit(0);

    map<string, int> nbr_full_instances;
    map<string, pair<float, int> > instance_correct_ratios;
    map<string, pair<float, int> > first_correct_ratios;
    map<string, pair<float, int> > usual_correct_ratios;

    map<string, pair<int, int> > instance_mean_features;

    map<string, int> instance_number_queries;

    for (voxel_annotation& a : annotations) {
        if (a.full) {
            nbr_full_instances[a.annotation] += 1;
        }
        if (!a.full || !a.annotation_covered || !a.segment_covered) {
            continue;
        }
        if (a.annotation != "ajax_1" && a.annotation != "ajax_2" && a.annotation != "ball_1") {
            continue;
        }

        instance_number_queries[a.annotation] += 1;

        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(a.segment_file, *cloud);

        HistCloudT::Ptr features(new HistCloudT);
        obr.load_features_for_segment(features, a.segment_id);
        vector<index_score> scores;

        obr_scans.rvt.top_larger_similarities(scores, features, nbr_intial_query);

        vector<index_score> updated_scores;
        for (size_t i = 0; i < scores.size(); ++i) {
            /*chrono::time_point<std::chrono::system_clock> start, end;
            start = chrono::system_clock::now();*/
            CloudT::Ptr voxel_centers(new CloudT);
            vector<double> vocabulary_norms;
            vector<map<int, double> > vocabulary_vectors;

            //vector<HistCloudT::Ptr> voxels;
            //CloudT::Ptr dummy_centers(new CloudT);
            //vector<double> dummy_norms;
            //get_oversegmented_voxels_for_scan(voxels, dummy_centers, dummy_norms, scores[i].first, obr_scans);
            get_oversegmented_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, scores[i].first, obr_scans);

            /*for (size_t j = 0; j < vocabulary_norms.size(); ++j) {
                double norm1 = vocabulary_norms[j];
                double norm2 = 0.0;
                for (pair<const int, double>& e : vocabulary_vectors[j]) {
                    //cout << "(" << e.first << ", " << e.second << ") ";
                    norm2 += e.second;
                }
                //cout << endl;
                if (fabs(norm1 - norm2) > 1e-7f) {
                    cout << "Norms not equal: " << norm1 << " != " << norm2 << endl;
                    exit(0);
                }

                map<int, double> vocabulary_vector;
                obr_scans.rvt.compute_query_index_vector(vocabulary_vector, voxels[j], mapping);

                if (vocabulary_vector.size() != vocabulary_vectors[j].size()) {
                    cout << "Vectors don't have same size: " << vocabulary_vector.size() << " != " << vocabulary_vectors[j].size() << endl;
                }

                for (pair<const int, double>& u : vocabulary_vectors[j]) {
                    if (vocabulary_vector.count(u.first) == 0) {
                        cout << "Missing an element!" << endl;
                        exit(0);
                    }
                    if (fabs(u.second - vocabulary_vector[u.first]) > 1e-7) {
                        cout << "Vector element not the same: " << u.second << " != " << vocabulary_vector[u.first] << endl;
                        exit(0);
                    }
                }
            }*/
            //cout << voxel_centers->at(0) << ", " << voxel_centers->at(1) << endl; // seems normal

            /*cout << "Vocabulary norms size: " << vocabulary_norms.size() << endl;
            end = chrono::system_clock::now();
            chrono::duration<double> elapsed_seconds = end-start;
            cout << "Getting voxels took " << elapsed_seconds.count() << " seconds" << endl;*/

            double score = obr_scans.rvt.compute_min_combined_dist(features, vocabulary_vectors, vocabulary_norms, voxel_centers, mapping);
            //double score = obr_scans.rvt.compute_min_combined_dist(features, voxels, dummy_norms, dummy_centers);
            updated_scores.push_back(index_score(scores[i].first, score));
        }

        std::sort(updated_scores.begin(), updated_scores.end(), [](const index_score& s1, const index_score& s2) {
            return s1.second < s2.second; // find min elements!
        });

        updated_scores.resize(nbr_query);
        //scores.resize(nbr_query);

        int scan_ind = scan_ind_for_supervoxel(a.segment_id, obr);
        calculate_correct_ratio(instance_correct_ratios, a, scan_ind, updated_scores, obr_scans);
        calculate_correct_ratio(first_correct_ratios, a, scan_ind, scores, obr_scans, false);
        scores.resize(nbr_query);
        calculate_correct_ratio(usual_correct_ratios, a, scan_ind, scores, obr_scans);
        cout << "Number of features: " << features->size() << endl;

        instance_mean_features[a.annotation].first += features->size();
        instance_mean_features[a.annotation].second += 1;

        //obr.visualize_cloud(cloud);
    }

    cout << "First round correct ratios: " << endl;
    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
        cout << c.first << " usual correct ratio: " << usual_correct_ratios[c.first].first/float(usual_correct_ratios[c.first].second) << endl;
        cout << c.first << " first round correct ratio: " << first_correct_ratios[c.first].first/float(first_correct_ratios[c.first].second) << endl;
        cout << "Mean features: " << float(instance_mean_features[c.first].first)/float(instance_mean_features[c.first].second) << endl;
        cout << "Number of queries: " << instance_number_queries[c.first] << endl;
    }
}

vector<index_score> top_combined_similarities(map<int, double>& larger_map, map<int, double>& smaller_map, int nbr_query, float& ratio, int nbr_features)
{
    double larger_mean = 1.0;
    /*size_t counter = 2;
    for (const index_score& s : larger_map) {
        if (counter > 10) {
            break;
        }
        larger_mean += s.second;
        ++counter;
    }
    larger_mean /= double(counter);*/

    double smaller_mean = 1.0;
    /*counter = 2;
    for (const index_score& s : smaller_map) {
        if (counter > 10) {
            break;
        }
        smaller_mean += s.second;
        ++counter;
    }
    smaller_mean /= double(counter);*/

    int smaller_count = 0;
    int larger_count = 0;

    for (pair<const int, double>& s : smaller_map) {
        if (larger_map.count(s.first)) {
            //s.second = std::max(s.second/larger_mean, smaller_map[s.first]/smaller_mean);
            //s.second *= smaller_map[s.first];
            //float extra = 1.0 + float(nbr_features) / 500.0f;
            if (s.second/smaller_mean > larger_map[s.first]/larger_mean) {
                s.second = s.second/smaller_mean;
                ++smaller_count;
            }
            else {
                s.second = larger_map[s.first]/larger_mean;
                ++larger_count;
            }
        }
        else {
            s.second = 1000.0; // very larger
        }
    }

    vector<index_score> rtn;
    rtn.insert(rtn.end(), smaller_map.begin(), smaller_map.end());
    std::sort(rtn.begin(), rtn.end(), [](const index_score& s1, const index_score& s2) {
        return s1.second < s2.second; // find min elements!
    });

    rtn.resize(nbr_query);

    ratio = float(larger_count) / float(larger_count + smaller_count);

    return rtn;
}

void query_supervoxel_and_scan_annotations(vector<voxel_annotation>& annotations, object_retrieval& obr, object_retrieval& obr_scans)
{
    const int nbr_query = 11;

    /*if (obr.gvt.empty()) {
        obr.read_vocabulary(obr.gvt);
    }
    obr.gvt.set_min_match_depth(3);
    obr.gvt.compute_normalizing_constants();*/

    if (obr_scans.vt.empty()) {
        obr_scans.read_vocabulary(obr_scans.vt);
    }
    obr_scans.vt.set_min_match_depth(3);
    obr_scans.vt.compute_normalizing_constants();
    /*object_retrieval::my_vocabulary_tree svt;
    ifstream in("/home/nbore/Data/Instances/scan_segments/vocabulary_pfhrgb.cereal", std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(svt);
    }
    in.close();
    svt.set_min_match_depth(3);
    svt.compute_normalizing_constants();*/

    // read supervoxel vocabulary
    if (obr.vt.empty()) {
        obr.read_vocabulary(obr.vt);
    }
    obr.vt.set_min_match_depth(3);
    obr.vt.compute_normalizing_constants();

    // read scan vocabulary, could also use obr_scans for this

    map<string, int> nbr_full_instances;
    map<string, pair<float, int> > instance_correct_ratios;
    map<string, float> instance_max_ratios;

    for (voxel_annotation& a : annotations) {
        if (a.full) {
            nbr_full_instances[a.annotation] += 1;
        }
        if (!a.full || !a.annotation_covered || !a.segment_covered) {
            continue;
        }
        if (a.annotation != "ajax_1" && a.annotation != "ajax_2" && a.annotation != "ball_1") {
            continue;
        }
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(a.segment_file, *cloud);

        HistCloudT::Ptr features(new HistCloudT);
        obr.load_features_for_segment(features, a.segment_id);

        vector<index_score> larger_scores;
        //obr.gvt.top_grouped_similarities(scores, features, nbr_query);
        obr_scans.vt.top_larger_similarities(larger_scores, features, 0);
        //svt.top_partial_similarities(larger_scores, features, 0);

        int scan_ind = scan_ind_for_supervoxel(a.segment_id, obr);

        vector<index_score> smaller_scores;
        obr.vt.top_smaller_similarities(smaller_scores, features, 0);

        map<int, double> larger_map((larger_scores.begin()), larger_scores.end());
        map<int, double> smaller_map((smaller_scores.begin()), smaller_scores.end());

        // do some after-processing to put this in a longer vector with correct segments
        // just keep the smaller segments with the smallest distance?
        //unordered_map<int, int> scan_for_supervoxel_hash;
        vector<index_score> voxel_larger_scores;
        for (pair<const int, double>& s : larger_map) {
            if (s.first == scan_ind) {
                continue;
            }
            vector<int> supervoxel_indices;
            get_voxels_for_scan(supervoxel_indices, s.first, obr_scans);
            int min_ind = -1;
            float min_dist = 1000.0; // very large
            for (int i : supervoxel_indices) {
                if (smaller_map.count(i) && smaller_map[i] < min_dist) {
                    min_ind = i;
                    min_dist = smaller_map[i];
                }
                //voxel_larger_scores.push_back(index_score(i, s.second));
                //scan_for_supervoxel_hash[i] = s.first;
            }

            for (int i : supervoxel_indices) {
                if (i != min_ind) {
                    smaller_map.erase(i);
                }
            }

            if (min_ind != -1) {
                larger_map.insert(make_pair(min_ind, s.second));
            }
        }

        float max_ratio;
        vector<index_score> scores = top_combined_similarities(larger_map, smaller_map, nbr_query, max_ratio, features->size());

        bool found = false;
        int counter = 0;
        int partial_counter = 0;
        bool last_match;
        for (index_score s : scores) {
            //CloudT::Ptr segment(new CloudT);
            //obr.read_scan(segment, s.first);
            //obr.visualize_cloud(segment);
            string instance = annotation_for_supervoxel(s.first, obr);
            if (s.first == a.segment_id) {
                found = true;
                continue;
            }
            last_match = false;
            if (instance == a.annotation) {
                ++counter;
                last_match = true;
                if (!a.full) {
                    ++partial_counter;
                }
                cout << "This was true." << endl;
            }
            else {
                cout << "This was false." << endl;
            }
            HistCloudT::Ptr features_match(new HistCloudT);
            obr.load_features_for_segment(features_match, s.first);
            cout << "Score: " << s.second << endl;
            cout << "Number of features: " << features_match->size() << endl;
        }

        if (!found) {
            if (last_match) {
                --counter;
            }
        }

        float correct_ratio = float(counter)/float(scores.size()-1);
        instance_correct_ratios[a.annotation].first += correct_ratio;
        instance_correct_ratios[a.annotation].second += 1;
        instance_max_ratios[a.annotation] += max_ratio;

        cout << "Showing " << a.segment_file << " with annotation " << a.annotation << endl;
        cout << "Number of features: " << features->size() << endl;
        cout << "Correct ratio: " << correct_ratio << endl;
        cout << "Partial ratio: " << float(partial_counter)/float(counter) << endl;

        //obr.visualize_cloud(cloud);
    }

    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
        cout << "Max ratio: " << instance_max_ratios[c.first] / float(c.second.second) << endl;
    }
}

int main(int argc, char** argv)
{
    int nbr_results = 11;

    using entity = SimpleSummaryParser::EntityStruct;
    string root_path = "/home/nbore/Data/Instances/";
    string segments_path = root_path + "object_segments";
    string aggregate_path = root_path + "scan_segments";
    string voxels_path = root_path + "supervoxel_segments";

    map<int, string> annotated;
    map<int, string> full_annotated;

    //aggregate the features of the segments into features of scans
    object_retrieval obr_segments(segments_path);
    list_all_annotated_segments(annotated, full_annotated, segments_path);
    //aggregate_features(obr_segments, aggregate_path);

    object_retrieval obr(aggregate_path);
    obr.segment_name = "scan";
    //aggregate_pfhrgb_features(obr);

    Eigen::Matrix3f Kall;
    {
        CloudT::Ptr segment(new CloudT);
        NormalCloudT::Ptr normal(new NormalCloudT);
        CloudT::Ptr hd_segment(new CloudT);
        string metadata;
        obr_segments.read_segment(segment, normal, hd_segment, Kall, metadata, 0);
    }

    //save_supervoxel_features(obr);
    //save_sift_features(obr);
    //save_duplet_features(obr);
    //exit(0);

    object_retrieval obr_voxels(voxels_path);
    vector<voxel_annotation> annotations;
    string annotations_file = voxels_path + "/voxel_annotations.cereal";
    list_annotated_supervoxels(annotations, annotations_file, Kall, obr_voxels);
    //visualize_supervoxel_annotation(annotations, obr_voxels);
    //query_supervoxel_annotations(annotations, obr_voxels);
    //query_supervoxel_and_scan_annotations(annotations, obr_voxels, obr);
    //query_supervoxel_annotations_scans(annotations, Kall, obr_voxels, obr);
    query_supervoxel_annotations_oversegments(annotations, Kall, obr_voxels, obr);
    //save_sift_features_for_supervoxels(Kall, obr_voxels);
    //save_pfhrgb_features_for_supervoxels(obr_voxels);
    //convert_voxels_to_binary(obr);
    //save_oversegmented_points(obr);
    //save_supervoxel_centers(obr);

    cout << "Extracted all of the features" << endl;

    // do initial training of vocabulary, save vocabulary
    //obr.train_vocabulary_incremental(5000, false);
    //obr_voxels.train_vocabulary_incremental(12000, false);
    //save_split_features(obr_voxels);
    //obr_voxels.train_grouped_vocabulary(12000, false);
    exit(0);

    // make sure there is a vocabulary to query from
    if (obr.rvt.empty()) {
        obr.read_vocabulary(obr.rvt);
    }
    obr.rvt.set_min_match_depth(2);
    obr.rvt.compute_normalizing_constants();

    using node = vocabulary_tree<HistT, 8>::node;
    map<node*, double> original_weights;
    //obr.rvt.compute_pyramid_match_weights(original_weights); // not necessary for this kind of matching

    int true_hits = 0;
    int false_hits = 0;
    int nbr_annotated = 0;
    int first_hits = 0;

    int reweight_true_hits = 0;
    int reweight_false_hits = 0;
    int reweight_first_hits = 0;

    chrono::time_point<std::chrono::system_clock> start, end;
    start = chrono::system_clock::now();

    map<string, pair<int, int> > instance_true_false_hits;
    map<string, pair<int, int> > reweight_instance_true_false_hits;

    // ok, now we've trained the representation, let's query it
    // and simply count the scans with correct annotation
    // so, simply find the annotated segments and query them
    for (pair<const int, string>& v : full_annotated) {
        HistCloudT::Ptr features(new HistCloudT);
        obr_voxels.load_features_for_segment(features, v.first);
        //obr_segments.load_features_for_segment(features, v.first);
        cout << "Features length: " << features->size() << endl;
        cout << "Queried instance: " << v.second << endl;
        CloudT::Ptr segment(new CloudT);
        NormalCloudT::Ptr normal(new NormalCloudT);
        CloudT::Ptr hd_segment(new CloudT);
        CloudT::Ptr kp_cloud(new CloudT);
        Eigen::Matrix3f K;
        string metadata;
        cout << __FILE__ << ", " << __LINE__ << endl;
        //obr_segments.read_segment(segment, normal, hd_segment, K, metadata, v.first);
        cout << __FILE__ << ", " << __LINE__ << endl;
        //SiftCloudT::Ptr sift_features(new SiftCloudT);
        //sift_features_for_segment(sift_features, kp_cloud, hd_segment, K);
        //compute_feature_duplets(features, sift_features, kp_cloud);
        //shot_features_for_segment(features, hd_segment, K);
        //obr_segments.visualize_cloud(hd_segment);
        vector<index_score> scores;
        vector<index_score> reweight_scores;
        obr.rvt.top_similarities(scores, features, nbr_results);
        //obr.query_reweight_vocabulary(scores, reweight_scores, features, hd_segment, K, nbr_results);
        string first_annotation = annotation_for_scan(scores[0].first, obr);
        if (first_annotation == v.second) {
            ++first_hits;
        }
        bool found_query = false;
        bool reweight_found_query = false;
        bool last_hit;
        bool reweight_last_hit;
        for (index_score s : scores) {
            if (scan_contains_segment(s.first, v.first, obr)) {
                found_query = true;
                continue;
            }

            string annotation = annotation_for_scan(s.first, obr);
            last_hit = (annotation == v.second);
            cout << "Annotation: " << annotation << " with score: " << s.second << endl;
            if (annotation == v.second) {
                ++true_hits;
                instance_true_false_hits[v.second].first += 1;
            }
            else {
                ++false_hits;
                instance_true_false_hits[v.second].second += 1;
            }
        }
        for (index_score s : reweight_scores) {
            if (scan_contains_segment(s.first, v.first, obr)) {
                reweight_found_query = true;
                continue;
            }

            string annotation = annotation_for_scan(s.first, obr);
            reweight_last_hit = (annotation == v.second);
            cout << "Annotation: " << annotation << " with score: " << s.second << endl;
            if (annotation == v.second) {
                ++reweight_true_hits;
                reweight_instance_true_false_hits[v.second].first += 1;
            }
            else {
                ++reweight_false_hits;
                reweight_instance_true_false_hits[v.second].second += 1;
            }
        }
        if (!found_query) {
            if (last_hit) {
                --true_hits;
                instance_true_false_hits[v.second].first -= 1;
            }
            else {
                --false_hits;
                instance_true_false_hits[v.second].second -= 1;
            }
        }
        if (!reweight_found_query) {
            if (reweight_last_hit) {
                --reweight_true_hits;
                reweight_instance_true_false_hits[v.second].first -= 1;
            }
            else {
                --false_hits;
                reweight_instance_true_false_hits[v.second].second -= 1;
            }
        }

        ++nbr_annotated;
    }

    end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = end-start;

    cout << "True hits: " << true_hits << endl;
    cout << "False hits: " << false_hits << endl;
    cout << "Ratio: " << float(true_hits)/float(true_hits+false_hits) << endl;
    cout << "First hit ratio: " << float(first_hits)/float(nbr_annotated) << endl;

    cout << "Reweight True hits: " << reweight_true_hits << endl;
    cout << "Reweight False hits: " << reweight_false_hits << endl;
    cout << "Reweight Ratio: " << float(reweight_true_hits)/float(reweight_true_hits+reweight_false_hits) << endl;
    cout << "Reweight First hit ratio: " << float(reweight_first_hits)/float(nbr_annotated) << endl;

    cout << "Benchmark took " << elapsed_seconds.count() << " seconds" << endl;

    cout << "First round: " << endl;
    for (pair<const string, pair<int, int> >& a : instance_true_false_hits) {
        cout << a.first << " ratio: " << float(a.second.first)/float(a.second.first+a.second.second) << endl;
    }

    cout << "Reweight round: " << endl;
    for (pair<const string, pair<int, int> >& a : reweight_instance_true_false_hits) {
        cout << a.first << " ratio: " << float(a.second.first)/float(a.second.first+a.second.second) << endl;
    }

    // ratio 0.0875 for voc tree minus norms
    // ratio 0.0875 for voc tree
    // ratio 0.0875 for voc tree min comparison

    // vt normal and min comp
    // True hits: 161
    // False hits: 1679
    // Ratio: 0.0875
    // First hit ratio: 0.228261
    // Benchmark took 73.3449 seconds

    // pm normal
    // True hits: 42
    // False hits: 1798
    // Ratio: 0.0228261
    // First hit ratio: 0.0271739

    return 0;
}
