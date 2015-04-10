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

// OK
int scan_ind_for_segment(int i, object_retrieval& obr_segments)
{
    string segment_folder = obr_segments.get_folder_for_segment_id(i);
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

// OK
void save_split_features(object_retrieval& obr_segments)
{
    for (int i = 0; ; ++i) {
        string segment_folder = obr_segments.get_folder_for_segment_id(i);
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
        vector<CloudT::Ptr> split_keypoints;
        pfhrgb_estimation::split_descriptor_points(split_features, split_keypoints, desc_cloud, kp_cloud, 30);
        cout << "Split features size: " << split_features.size() << endl;
        cout << "Split keypoint size: " << split_keypoints.size() << endl;
        int counter = 0;
        int j = 0;
        for (PfhRgbCloudT::Ptr& split_cloud : split_features) {
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
            string split_file = segment_folder + "/split_features" + to_string(counter) + ".pcd";
            string split_points_file = segment_folder + "/split_points" + to_string(counter) + ".pcd";
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
                             int scan_ind, vector<index_score>& scores, object_retrieval& obr_scans, int noise_scans_size, const bool verbose = true)
{
    bool found = false;
    int counter = 0;
    int partial_counter = 0;
    bool last_match;
    for (index_score s : scores) {
        if (s.first < noise_scans_size) { // is noise
            cout << "This was noise, false" << endl;
            cout << "Score: " << s.second << endl;
            last_match = false;
            continue;
        }
        int query_ind = s.first - noise_scans_size;
        string instance = annotation_for_scan(query_ind, obr_scans);
        if (query_ind == scan_ind) {
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
        obr_scans.load_features_for_segment(features_match, query_ind);
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
void compute_decay_correct_ratios(vector<pair<float, int> >& decay_correct_ratios, vector<int>& intermediate_points,
                                  voxel_annotation& a, int scan_ind, vector<index_score>& scores, object_retrieval& obr_scans,
                                  int nbr_query, int noise_scans_size)
{
    // do this, but until we have found enough before the first intermediate point
    vector<int> counters(intermediate_points.size(), 0);
    vector<int> comparisons(intermediate_points.size(), 0);
    vector<bool> last_matches(intermediate_points.size());
    vector<bool> founds(intermediate_points.size(), false);
    for (index_score s : scores) {
        if (comparisons[0] >= nbr_query) {
            break;
        }
        string instance;
        if (s.first >= noise_scans_size) {
            instance = annotation_for_scan(s.first - noise_scans_size, obr_scans);
        }

        for (int i = 0; i < intermediate_points.size(); ++i) {
            if (comparisons[i] >= nbr_query) {
                continue;
            }
            if (s.first < noise_scans_size && s.first >= intermediate_points[i]) {
                continue;
            }

            if (s.first < noise_scans_size) { // is noise
                last_matches[i] = false;
                ++comparisons[i];
                continue;
            }
            int query_ind = s.first - noise_scans_size;
            if (query_ind == scan_ind) {
                founds[i] = true;
                ++comparisons[i];
                continue;
            }
            last_matches[i] = false;
            if (instance == a.annotation) {
                ++counters[i];
                last_matches[i] = true;
            }
            ++comparisons[i];
        }
    }

    for (int i = 0; i < intermediate_points.size(); ++i) {
        if (!founds[i]) {
            if (last_matches[i]) {
                --counters[i];
            }
        }

        float correct_ratio = float(counters[i])/float(nbr_query-1);
        decay_correct_ratios[i].first += correct_ratio;
        decay_correct_ratios[i].second += 1;
    }
}

// OK
void get_voxel_vectors_for_scan(CloudT::Ptr& voxel_centers, vector<double>& vocabulary_norms,
                                vector<map<int, double> >& vocabulary_vectors, int i, object_retrieval& obr_scans)
{
    string scan_path = obr_scans.get_folder_for_segment_id(i);

    string centers_file = scan_path + "/split_centers_1.pcd";
    if (pcl::io::loadPCDFile(centers_file, *voxel_centers) == -1) {
        cout << "Could not read file " << centers_file << endl;
        exit(0);
    }

    string norms_file = scan_path + "/grouped_vocabulary_norms_1.cereal";
    ifstream inn(norms_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(inn);
        archive_i(vocabulary_norms);
    }

    string vectors_file = scan_path + "/grouped_vocabulary_vectors_1.cereal";
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
        vector<double> vocabulary_norms;

        for (HistCloudT::Ptr& voxel : voxels) {
            if (voxel->size() < min_features) {
                continue;
            }
            vocabulary_vectors.push_back(map<int, double>());
            double pnorm = obr_segments.gvt.compute_query_index_vector(vocabulary_vectors.back(), voxel, mapping);
            vocabulary_norms.push_back(pnorm);
        }
        string vectors_file = path + "/grouped_vocabulary_vectors_1.cereal";
        ofstream outv(vectors_file, std::ios::binary);
        {
            cereal::BinaryOutputArchive archive_o(outv);
            archive_o(vocabulary_vectors);
        }
        string norms_file = path + "/grouped_vocabulary_norms_1.cereal";
        ofstream outn(norms_file, std::ios::binary);
        {
            cereal::BinaryOutputArchive archive_o(outn);
            archive_o(vocabulary_norms);
        }

        CloudT::Ptr centers_cloud(new CloudT);
        compute_voxel_centers(centers_cloud, voxel_points, min_features);
        string centers_file = path + "/split_centers_1.pcd";
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
    string group_file = root_path + "/group_subgroup_1.cereal";
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
void compute_grown_segment_score(HistCloudT::Ptr& query_features, CloudT::Ptr& query_keypoints, CloudT::Ptr& query_cloud,
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
        ro.register_using_features(query_features, query_keypoints, result_features, result_keypoints);
    }
}

// OK
void compute_grow_subsegment_scores(vector<index_score>& updated_scores, vector<vector<int> >& oversegment_indices, vector<index_score>& scores,
                                    vector<int>& hints, HistCloudT::Ptr& features, map<vocabulary_tree<HistT, 8>::node*, int>& mapping, object_retrieval& obr_segments,
                                    object_retrieval& obr_scans, object_retrieval& obr_scans_annotations, int nbr_query, int noise_scans_size)
{
    //vector<index_score> total_scores;
    for (size_t i = 0; i < scores.size(); ++i) {
        CloudT::Ptr voxel_centers(new CloudT);
        vector<double> vocabulary_norms;
        vector<map<int, double> > vocabulary_vectors;

        if (scores[i].first < noise_scans_size) {
            get_voxel_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, scores[i].first, obr_scans);
        }
        else {
            get_voxel_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, scores[i].first-noise_scans_size, obr_scans_annotations);
        }

        /*vector<map<int, double> > vocabulary_vectors_copy = vocabulary_vectors;
        vector<double> vocabulary_norms_copy = vocabulary_norms;
        CloudT::Ptr voxel_centers_copy(new CloudT);
        *voxel_centers_copy = *voxel_centers;*/
        vector<int> selected_indices;
        double score = obr_segments.gvt.compute_min_combined_dist(selected_indices, features, vocabulary_vectors, vocabulary_norms, voxel_centers, mapping, hints[i]);
        updated_scores.push_back(index_score(scores[i].first, score));
        oversegment_indices.push_back(selected_indices);

        //double total_score = obr_segments.gvt.compute_min_combined_dist(features, vocabulary_vectors_copy, vocabulary_norms_copy, voxel_centers_copy, mapping, -1);
        //total_scores.push_back(index_score(get<0>(scores[i]), total_score));

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
void query_supervoxel_oversegments(vector<voxel_annotation>& annotations, Eigen::Matrix3f& K,
                                   object_retrieval& obr_segments, object_retrieval& obr_scans,
                                   object_retrieval& obr_segments_annotations, object_retrieval& obr_scans_annotations,
                                   int noise_scans_size)
{
    const int nbr_query = 11;
    const int nbr_initial_query = 1500;

    map<vocabulary_tree<HistT, 8>::node*, int> mapping;

    if (obr_segments.gvt.empty()) {
        obr_segments.read_vocabulary(obr_segments.gvt);
    }
    obr_segments.gvt.set_min_match_depth(2);
    obr_segments.gvt.compute_normalizing_constants();

    read_supervoxel_groups(obr_segments); // this will not be needed eventually, should be part of class init

    obr_segments.gvt.compute_leaf_vocabulary_vectors(); // this will not be needed eventually

    obr_segments.gvt.get_node_mapping(mapping);

    //save_oversegmented_grouped_vocabulary_index_vectors(obr_scans, obr_segments);
    //save_oversegmented_grouped_vocabulary_index_vectors(obr_scans_annotations, obr_segments);
    //exit(0);

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
        //obr_segments_annotations.visualize_cloud(cloud);

        HistCloudT::Ptr features(new HistCloudT);
        CloudT::Ptr keypoints(new CloudT);
        obr_segments_annotations.load_features_for_segment(features, keypoints, a.segment_id); // also load keypoints
        vector<tuple<int, int, double> > tuple_scores;

        chrono::time_point<std::chrono::system_clock> start1, end1;
        start1 = chrono::system_clock::now();

        // this also takes care of the scan association
        obr_segments.gvt.top_optimized_similarities(tuple_scores, features, nbr_initial_query);

        //vector<tuple<int, int, double> > reweighted_scores;
        //reweight_query_vocabulary(reweighted_scores, obr_segments, obr_scans, obr_scans_annotations, K,
        //                          tuple_scores, noise_scans_size, features, cloud, nbr_initial_query);
        //tuple_scores = reweighted_scores;

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
        vector<vector<int> > oversegment_indices;
        // this function call indicates that we need some better abstractions
        compute_grow_subsegment_scores(updated_scores, oversegment_indices, scores, hints, features, mapping,
                                       obr_segments, obr_scans, obr_scans_annotations, nbr_query, noise_scans_size);
        // not returning anything yet
        compute_grown_segment_score(features, keypoints, cloud, updated_scores, oversegment_indices, obr_scans, obr_scans_annotations, noise_scans_size);

        end2 = chrono::system_clock::now();
        chrono::duration<double> elapsed_seconds2 = end2-start2;

        total_time1 += elapsed_seconds1.count();
        total_time2 += elapsed_seconds2.count();

        // need to check that it's correct here
        int scan_ind = scan_ind_for_segment(a.segment_id, obr_segments_annotations);
        calculate_correct_ratio(instance_correct_ratios, a, scan_ind, updated_scores, obr_scans_annotations, noise_scans_size);
        //calculate_correct_ratio(first_correct_ratios, a, scan_ind, scores, obr_scans, false);
        scores.resize(nbr_query);
        calculate_correct_ratio(usual_correct_ratios, a, scan_ind, scores, obr_scans_annotations, noise_scans_size);
        //calculate_correct_ratio(total_correct_ratios, a, scan_ind, total_scores, obr_scans_annotations, noise_scans_size);
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
    pfhrgb_estimation::compute_pfhrgb_features(features, kp_cloud, cloud, false);

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

        if (scores[i].first < noise_scans_size) {
            get_voxel_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, scores[i].first, obr_scans);
        }
        else {
            get_voxel_vectors_for_scan(voxel_centers, vocabulary_norms, vocabulary_vectors, scores[i].first-noise_scans_size, obr_scans_annotations);
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
void query_supervoxels(vector<voxel_annotation>& annotations, object_retrieval& obr_segments, object_retrieval& obr_segments_annotations,
                       object_retrieval& obr_scans_annotations, int noise_scans_size, int noise_segments_size)
{
    const int nbr_query = 11;

    if (obr_segments.vt.empty()) {
        obr_segments.read_vocabulary(obr_segments.vt);
    }
    obr_segments.vt.set_min_match_depth(2);
    obr_segments.vt.compute_normalizing_constants();

    map<string, int> nbr_full_instances;
    map<string, pair<float, int> > instance_correct_ratios;
    vector<pair<float, int> > decay_correct_ratios;
    vector<int> intermediate_points;
    for (int i = 0; i < noise_scans_size; i += 100) {
        intermediate_points.push_back(i);
        decay_correct_ratios.push_back(make_pair(0.0f, 0));
    }

    chrono::time_point<std::chrono::system_clock> start, end;
    start = chrono::system_clock::now();

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
        obr_segments_annotations.load_features_for_segment(features, a.segment_id);

        vector<index_score> scores;
        //obr_segments.vt.top_combined_similarities(scores, features, nbr_query);
        obr_segments.vt.top_similarities(scores, features, nbr_query);

        for (index_score& s : scores) {
            if (s.first < noise_segments_size) {
                s.first = s.first = 0;//scan_ind_for_segment(s.first, obr_segments);
            }
            else {
                s.first = scan_ind_for_segment(s.first-noise_segments_size, obr_segments_annotations) + noise_scans_size;
            }
        }

        int scan_ind = scan_ind_for_segment(a.segment_id, obr_segments_annotations);
        calculate_correct_ratio(instance_correct_ratios, a, scan_ind, scores, obr_scans_annotations, noise_scans_size);
        //compute_decay_correct_ratios(decay_correct_ratios, intermediate_points, a, scan_ind, scores, obr_scans_annotations, nbr_query, noise_scans_size);

        cout << "Number of features: " << features->size() << endl;
    }

    end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = end-start;

    cout << "Benchmark took " << elapsed_seconds.count() << " seconds" << endl;

    for (pair<const string, pair<float, int> > c : instance_correct_ratios) {
        cout << c.first << " correct ratio: " << c.second.first/float(c.second.second) << endl;
    }

    /*for (int i = 0; i < intermediate_points.size(); ++i) {
        cout << intermediate_points[i] << " ";
    }
    cout << endl;

    for (int i = 0; i < intermediate_points.size(); ++i) {
        cout << decay_correct_ratios[i].first/float(decay_correct_ratios[i].second) << " ";
    }
    cout << endl;*/
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

    //save_split_features(obr_segments);
    //save_split_features(obr_segments_noise);
    //exit(0);

    // probably train using the noise segments
    //obr_segments_noise.train_grouped_vocabulary(12000, false);

    // TODO: add something like obr_segments_noise.get_scan_count()
    int noise_scans_size = 3526;
    //obr_segments_noise.add_others_to_grouped_vocabulary(30000, obr_segments, noise_scans_size);

    //obr_segments_noise.train_vocabulary_incremental(12000, false);
    int noise_segments_size = 63136;
    //obr_segments_noise.add_others_to_vocabulary(30000, obr_segments.segment_path, noise_segments_size);
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

    vector<voxel_annotation> annotations;
    string annotations_file = segment_path + "/voxel_annotations.cereal";
    list_annotated_supervoxels(annotations, annotations_file, K, obr_segments);

    //query_supervoxel_oversegments(annotations, K, obr_segments, obr_scans, obr_segments, obr_scans, 0);

    query_supervoxel_oversegments(annotations, K, obr_segments_noise, obr_scans_noise, obr_segments, obr_scans, noise_scans_size);

    //query_supervoxels(annotations, obr_segments_noise, obr_segments, obr_scans, noise_scans_size, noise_segments_size);

    /*CloudT::Ptr query_cloud_larger(new CloudT);
    pcl::io::loadPCDFile("/home/nbore/Data/rgb_0015_label_0.pcd", *query_cloud_larger);

    query_cloud(query_cloud_larger, obr_segments_noise, obr_scans_noise, obr_scans, noise_scans_size);*/

    cout << "Program finished..." << endl;

    return 0;
}
