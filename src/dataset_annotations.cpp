#include "object_3d_retrieval/dataset_annotations.h"

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <cereal/types/string.hpp>
#include <cereal/archives/binary.hpp>

using namespace std;

namespace dataset_annotations {

// OK
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
                             int scan_ind, vector<index_score>& scores, object_retrieval& obr_scans, int noise_scans_size, const bool verbose)
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
        if (verbose) {
            HistCloudT::Ptr features_match(new HistCloudT);
            obr_scans.load_features_for_segment(features_match, query_ind);
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
void calculate_correct_ratio(map<string, pair<float, int> >& instance_correct_ratios, const string& annotation,
                             int scan_ind, vector<index_score>& scores, object_retrieval& obr_scans, int noise_scans_size, const bool verbose)
{
    bool found = false;
    int counter = 0;
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
        if (instance == annotation) {
            ++counter;
            last_match = true;
            if (verbose) cout << "This was true." << endl;
        }
        else {
            if (verbose) cout << "This was false." << endl;
        }
        if (verbose) {
            //HistCloudT::Ptr features_match(new HistCloudT);
            //obr_scans.load_features_for_segment(features_match, query_ind);
            cout << "Score: " << s.second << endl;
            //cout << "Number of features in scan: " << features_match->size() << endl;
        }
    }

    if (!found) {
        if (last_match) {
            --counter;
        }
    }

    float correct_ratio = float(counter)/float(scores.size()-1);
    instance_correct_ratios[annotation].first += correct_ratio;
    instance_correct_ratios[annotation].second += 1;

    if (verbose) {
        cout << "Showing annotation: " << annotation << endl;
        cout << "Correct ratio: " << correct_ratio << endl;
    }
}

// OK
void calculate_correct_ratio_exclude_sweep(map<string, pair<float, int> >& instance_correct_ratios, const string& annotation, int scan_ind,
                                           vector<index_score>& scores, object_retrieval& obr_scans, int noise_scans_size, const bool verbose)
{
    int possible_query_inds = 5;

    int true_count = 0;
    int valid_queries = 0;
    for (int i = 0; i < scores.size() && valid_queries < scores.size()-possible_query_inds; ++i) {
        index_score s = scores[i];
        if (s.first < noise_scans_size) { // is noise
            cout << "This was noise, false" << endl;
            cout << "Score: " << s.second << endl;
            ++valid_queries;
            continue;
        }
        int query_ind = s.first - noise_scans_size;
        string instance = annotation_for_scan(query_ind, obr_scans);
        if (std::abs(query_ind - scan_ind) < 17) { // might be part of the same sweep
            // replace this with something checking if they are from the same sweep
            if (std::abs(query_ind - scan_ind) <= 2) { // two apart will be excluded, they might contain the same
                continue;
            }
        }
        ++valid_queries;
        if (instance == annotation) {
            ++true_count;
            if (verbose) cout << "This was true." << endl;
        }
        else {
            if (verbose) cout << "This was false." << endl;
        }
        if (verbose) {
            cout << "Scan: " << query_ind << ", Score: " << s.second << endl;
        }
    }

    float correct_ratio = float(true_count)/float(valid_queries);
    instance_correct_ratios[annotation].first += correct_ratio;
    instance_correct_ratios[annotation].second += 1;

    if (verbose) {
        cout << "Showing annotation: " << annotation << endl;
        cout << "Correct ratio: " << correct_ratio << endl;
    }
}

// OK
void calculate_correct_ratio_exclude_sweep_precise(map<string, pair<float, int> >& instance_correct_ratios, const string& annotation, int scan_ind,
                                                   vector<index_score>& scores, object_retrieval& obr_scans, int noise_scans_size, const bool verbose)
{
    string scan_file = obr_scans.get_scan_file(scan_ind);
    string query_sweep = boost::filesystem::path(scan_file).parent_path().string();

    int true_count = 0;
    int valid_queries = 0;
    for (int i = 0; i < scores.size() && valid_queries < 10; ++i) {
        index_score s = scores[i];
        if (s.first < noise_scans_size) { // is noise
            cout << "This was noise, false" << endl;
            cout << "Score: " << s.second << endl;
            ++valid_queries;
            continue;
        }
        int match_ind = s.first - noise_scans_size;
        if (std::abs(match_ind - scan_ind) < 17) { // might be part of the same sweep
            // replace this with something checking if they are from the same sweep
            /*if (std::abs(query_ind - scan_ind) <= 2) { // two apart will be excluded, they might contain the same
                continue;
            }*/
            string match_file = obr_scans.get_scan_file(match_ind);
            string match_sweep = boost::filesystem::path(match_file).parent_path().string();
            if (match_sweep == query_sweep) {
                //cout << match_sweep << endl;
                //cout << query_sweep << endl;
                //exit(0);
                continue;
            }
        }
        string instance = annotation_for_scan(match_ind, obr_scans);
        ++valid_queries;
        if (instance == annotation) {
            ++true_count;
            if (verbose) cout << "This was true." << endl;
        }
        else {
            if (verbose) cout << "This was false." << endl;
        }
        if (verbose) {
            cout << "Scan: " << match_ind << ", Score: " << s.second << endl;
        }
    }

    if (valid_queries < 10) {
        cout << "There were not enough entries in query vector, exiting..." << endl;
        exit(-1);
    }

    float correct_ratio = float(true_count)/float(valid_queries);
    instance_correct_ratios[annotation].first += correct_ratio;
    instance_correct_ratios[annotation].second += 1;

    if (verbose) {
        cout << "Showing annotation: " << annotation << endl;
        cout << "Correct ratio: " << correct_ratio << endl;
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

} // namespace dataset_annotations
