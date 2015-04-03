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
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/utility.hpp>

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
        pfhrgb_estimation::compute_pfhrgb_features(desc_cloud, kp_cloud, cloud, false);

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
void compute_and_save_segments(object_retrieval& obr_scans)
{
    using Graph = supervoxel_segmentation::Graph;

    string base_path = boost::filesystem::path(obr_scans.segment_path).parent_path().string();
    string segment_path = base_path + "/supervoxel_segments";
    boost::filesystem::create_directory(segment_path);

    Eigen::Matrix3f K; // dummy

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

// OK
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

// OK
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

// OK
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

// OK
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

// OK
void get_voxel_vectors_for_scan(CloudT::Ptr& voxel_centers, vector<double>& vocabulary_norms,
                                vector<map<int, double> >& vocabulary_vectors, int i, object_retrieval& obr_scans)
{
    string scan_path = obr_scans.get_folder_for_segment_id(i);

    string centers_file = scan_path + "/split_centers.pcd";
    if (pcl::io::loadPCDFile(centers_file, *voxel_centers) == -1) {
        cout << "Could not read file " << centers_file << endl;
        exit(0);
    }

    string norms_file = scan_path + "/grouped_vocabulary_norms.cereal";
    ifstream inn(norms_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(inn);
        archive_i(vocabulary_norms);
    }

    string vectors_file = scan_path + "/grouped_vocabulary_vectors.cereal";
    ifstream inv(vectors_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(inv);
        archive_i(vocabulary_vectors);
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
            string oversegment_path = voxel_path + "/split_features" + to_string(i) + ".pcd";
            if (!boost::filesystem::is_regular_file(oversegment_path)) {
                break;
            }
            voxels.push_back(HistCloudT::Ptr(new HistCloudT));
            if (pcl::io::loadPCDFile(oversegment_path, *voxels.back()) == -1) {
                cout << "Could not read oversegment file " << oversegment_path << endl;
                exit(0);
            }

            string points_path = voxel_path + "/split_points" + to_string(i) + ".pcd";
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
void save_oversegmented_grouped_vocabulary_index_vectors(object_retrieval& obr_scans, object_retrieval& obr_voxels)
{
    int min_features = 20;

    map<vocabulary_tree<HistT, 8>::node*, int> mapping;
    obr_voxels.gvt.get_node_mapping(mapping);

    for (int i = 0; ; ++i) {
        string path = obr_scans.get_folder_for_segment_id(i);
        if (!boost::filesystem::is_directory(path)) {
            break;
        }
        vector<HistCloudT::Ptr> voxels;
        vector<CloudT::Ptr> voxel_points;
        get_voxels_and_points_for_scan(voxels, voxel_points, i, obr_scans);

        vector<map<int, double> > vocabulary_vectors;
        vector<double> vocabulary_norms;

        for (HistCloudT::Ptr& voxel : voxels) {
            if (voxel->size() < min_features) {
                continue;
            }
            vocabulary_vectors.push_back(map<int, double>());
            double pnorm = obr_voxels.gvt.compute_query_index_vector(vocabulary_vectors.back(), voxel, mapping);
            vocabulary_norms.push_back(pnorm);
        }
        string vectors_file = path + "/grouped_vocabulary_vectors.cereal";
        ofstream outv(vectors_file, std::ios::binary);
        {
            cereal::BinaryOutputArchive archive_o(outv);
            archive_o(vocabulary_vectors);
        }
        string norms_file = path + "/grouped_vocabulary_norms.cereal";
        ofstream outn(norms_file, std::ios::binary);
        {
            cereal::BinaryOutputArchive archive_o(outn);
            archive_o(vocabulary_norms);
        }

        CloudT::Ptr centers_cloud(new CloudT);
        compute_voxel_centers(centers_cloud, voxel_points, min_features);
        string centers_file = path + "/split_centers.pcd";
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
    vector<HistCloudT::Ptr> voxels;
    for (pair<const int, int>& p : temp) {
        //cout << p.second << endl;
        int scan_ind = scan_ind_for_supervoxel(p.second, obr_voxels); // the second is the actual segment
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
    string group_file = root_path + "/group_subgroup.cereal";
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
    string group_file = root_path + "/group_subgroup.cereal";
    ifstream in(group_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(obr_voxels.gvt.group_subgroup);
    }
}

// OK
void query_supervoxel_oversegments(vector<voxel_annotation>& annotations, Eigen::Matrix3f& K,
                                   object_retrieval& obr, object_retrieval& obr_scans)
{
    const int nbr_query = 11;
    const int nbr_intial_query = 200;

    map<vocabulary_tree<HistT, 8>::node*, int> mapping;

    if (obr.gvt.empty()) {
        obr.read_vocabulary(obr.gvt);
    }
    obr.gvt.set_min_match_depth(2);
    obr.gvt.compute_normalizing_constants();

    read_supervoxel_groups(obr); // this will not be needed eventually, should be part of class init

    obr.gvt.compute_leaf_vocabulary_vectors(); // this will not be needed eventually

    obr.gvt.get_node_mapping(mapping);

    //save_oversegmented_grouped_vocabulary_index_vectors(obr_scans, obr);
    //exit(0);

    //get_max_supervoxel_features(obr_scans);
    //change_supervoxel_groups(obr);
    //exit(0);

    map<string, int> nbr_full_instances;
    map<string, pair<float, int> > instance_correct_ratios;
    //map<string, pair<float, int> > first_correct_ratios;
    map<string, pair<float, int> > usual_correct_ratios;
    //map<string, pair<float, int> > total_correct_ratios;

    map<string, pair<int, int> > instance_mean_features;

    map<string, int> instance_number_queries;

    chrono::time_point<std::chrono::system_clock> start, end;
    start = chrono::system_clock::now();

    double total_time1 = 0.0;
    double total_time2 = 0.0;
    double total_timec = 0.0;

    int counter = 0;
    for (voxel_annotation& a : annotations) {
        if (counter > 5) {
            //break;
        }
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
        vector<tuple<int, int, double> > tuple_scores;

        chrono::time_point<std::chrono::system_clock> start1, end1;
        start1 = chrono::system_clock::now();

        // this also takes care of the scan association
        obr.gvt.top_optimized_similarities(tuple_scores, features, nbr_intial_query);

        vector<index_score> scores;
        vector<int> hints;
        for (const tuple<int, int, double>& t : tuple_scores) {
            scores.push_back(index_score(get<0>(t), get<2>(t)));
            hints.push_back(get<1>(t));
        }

        end1 = chrono::system_clock::now();
        chrono::duration<double> elapsed_seconds1 = end1-start1;

        chrono::time_point<std::chrono::system_clock> start2, end2;
        start2 = chrono::system_clock::now();

        vector<index_score> updated_scores;
        //vector<index_score> total_scores;
        for (size_t i = 0; i < scores.size(); ++i) {
            CloudT::Ptr voxel_centers(new CloudT);
            vector<double> vocabulary_norms;
            vector<map<int, double> > vocabulary_vectors;

            get_voxel_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, get<0>(scores[i]), obr_scans);

            chrono::time_point<std::chrono::system_clock> startc, endc;
            startc = chrono::system_clock::now();

            /*vector<map<int, double> > vocabulary_vectors_copy = vocabulary_vectors;
            vector<double> vocabulary_norms_copy = vocabulary_norms;
            CloudT::Ptr voxel_centers_copy(new CloudT);
            *voxel_centers_copy = *voxel_centers;*/
            double score = obr.gvt.compute_min_combined_dist(features, vocabulary_vectors, vocabulary_norms, voxel_centers, mapping, hints[i]);
            updated_scores.push_back(index_score(get<0>(scores[i]), score));

            /*double total_score = obr.gvt.compute_min_combined_dist(features, vocabulary_vectors_copy, vocabulary_norms_copy, voxel_centers_copy, mapping, -1);
            total_scores.push_back(index_score(get<0>(scores[i]), total_score));*/
            endc = chrono::system_clock::now();
            chrono::duration<double> elapsed_secondsc = endc-startc;
            total_timec += elapsed_secondsc.count();

        }

        end2 = chrono::system_clock::now();
        chrono::duration<double> elapsed_seconds2 = end2-start2;

        total_time1 += elapsed_seconds1.count();
        total_time2 += elapsed_seconds2.count();

        std::sort(updated_scores.begin(), updated_scores.end(), [](const index_score& s1, const index_score& s2) {
            return s1.second < s2.second; // find min elements!
        });

        updated_scores.resize(nbr_query);

        /*std::sort(total_scores.begin(), total_scores.end(), [](const index_score& s1, const index_score& s2) {
            return s1.second < s2.second; // find min elements!
        });
        total_scores.resize(nbr_query);*/

        int scan_ind = scan_ind_for_supervoxel(a.segment_id, obr);
        calculate_correct_ratio(instance_correct_ratios, a, scan_ind, updated_scores, obr_scans);
        //calculate_correct_ratio(first_correct_ratios, a, scan_ind, scores, obr_scans, false);
        scores.resize(nbr_query);
        calculate_correct_ratio(usual_correct_ratios, a, scan_ind, scores, obr_scans);
        //calculate_correct_ratio(total_correct_ratios, a, scan_ind, total_scores, obr_scans);
        cout << "Number of features: " << features->size() << endl;

        instance_mean_features[a.annotation].first += features->size();
        instance_mean_features[a.annotation].second += 1;

        ++counter;
    }

    end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = end-start;

    cout << "Benchmark took " << elapsed_seconds.count() << " seconds" << endl;
    cout << "First part took " << total_time1 << " seconds" << endl;
    cout << "Second part took " << total_time2 << " seconds" << endl;
    cout << "Calculation of second part took: " << total_timec << " seconds" << endl;

    cout << "First round correct ratios: " << endl;
    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
        //cout << c.first << " total correct ratio: " << total_correct_ratios[c.first].first/float(total_correct_ratios[c.first].second) << endl;
        cout << c.first << " usual correct ratio: " << usual_correct_ratios[c.first].first/float(usual_correct_ratios[c.first].second) << endl;
        //cout << c.first << " first round correct ratio: " << first_correct_ratios[c.first].first/float(first_correct_ratios[c.first].second) << endl;
        cout << "Mean features: " << float(instance_mean_features[c.first].first)/float(instance_mean_features[c.first].second) << endl;
        cout << "Number of queries: " << instance_number_queries[c.first] << endl;
    }
}

int main(int argc, char** argv)
{
    string root_path = "/home/nbore/Data/Instances/";
    string segments_path = root_path + "object_segments";
    string aggregate_path = root_path + "scan_segments";
    string voxels_path = root_path + "supervoxel_segments";

    string noise_root_path = "/home/nbore/Data/semantic_map/";
    string noise_scan_path = noise_root_path + "scan_segments";
    string noise_segment_path = noise_root_path + "supervoxel_segments";

    map<int, string> annotated;
    map<int, string> full_annotated;

    //aggregate the features of the segments into features of scans
    object_retrieval obr_segments(segments_path);
    list_all_annotated_segments(annotated, full_annotated, segments_path);
    //aggregate_features(obr_segments, aggregate_path);

    object_retrieval obr(aggregate_path);
    obr.segment_name = "scan";
    //aggregate_pfhrgb_features(obr);

    object_retrieval obr_scans_noise(noise_scan_path);
    obr_scans_noise.segment_name = "scan";
    object_retrieval obr_segments_noise(noise_segment_path);

    compute_and_save_segments(obr_scans_noise);
    exit(0);

    save_pfhrgb_features_for_supervoxels(obr_segments_noise);

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
    query_supervoxel_oversegments(annotations, Kall, obr_voxels, obr);
    //save_sift_features_for_supervoxels(Kall, obr_voxels);
    //save_pfhrgb_features_for_supervoxels(obr_voxels);

    //save_oversegmented_points(obr);
    //save_supervoxel_centers(obr);

    cout << "Extracted all of the features" << endl;

    // do initial training of vocabulary, save vocabulary
    //save_split_features(obr_voxels);
    //obr_voxels.train_grouped_vocabulary(4000, false);

    return 0;
}
