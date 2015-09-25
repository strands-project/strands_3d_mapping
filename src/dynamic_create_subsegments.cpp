#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "object_3d_retrieval/pfhrgb_estimation.h"

using namespace std;

// we need to put all of this in a nice library and link properly
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

void save_sweep_data()
{

}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the sweeps..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    dynamic_object_retrieval::convex_feature_cloud_map segment_features(data_path);
    dynamic_object_retrieval::convex_keypoint_cloud_map segment_keypoints(data_path);
    dynamic_object_retrieval::convex_segment_sweep_path_map segment_sweep_paths(data_path);

    dynamic_object_retrieval::data_summary summary;
    summary.load(data_path);

    summary.index_subsegment_paths.clear();
    summary.nbr_subsegments = 0;

    boost::filesystem::path current_path;
    dynamic_object_retrieval::sweep_summary sweep_data;
    vector<string> segment_paths;
    int counter;
    int vt_index = 0;
    for (auto tup : dynamic_object_retrieval::zip(segment_features, segment_keypoints, segment_sweep_paths)) {
        HistCloudT::Ptr features;
        CloudT::Ptr keypoints;
        boost::filesystem::path sweep_path;
        tie(features, keypoints, sweep_path) = tup;
        // maybe we should have an iterator for getting the number as well
        if (current_path != sweep_path) {
            if (!current_path.empty()) {
                // save the counter goddammit
                sweep_data.nbr_segments = counter;
                sweep_data.save(current_path / "subsegments");
            }
            sweep_data.load(sweep_path / "subsegments");
            // maybe check here if subsegment were already extracted?????? should be easy, can just use summary_iterators again
            current_path = sweep_path;
            counter = 0;
        }
        boost::filesystem::path subsegment_folder_path = sweep_path / "subsegments";

        cout << "Found cloud of size: " << features->size() << endl;
        cout << "Found feature path: " << sweep_path.string() << endl;

        vector<HistCloudT::Ptr> split_features;
        vector<CloudT::Ptr> split_keypoints;
        pfhrgb_estimation::split_descriptor_points(split_features, split_keypoints, features, keypoints, 30);
        cout << "Split features size: " << split_features.size() << endl;
        cout << "Split keypoint size: " << split_keypoints.size() << endl;

        for (auto feature_keypoint : dynamic_object_retrieval::zip(split_features, split_keypoints)) {
            HistCloudT::Ptr subsegment_features;
            CloudT::Ptr subsegment_keypoints;
            tie(subsegment_features, subsegment_keypoints) = feature_keypoint;
            if (subsegment_features->empty()) {
                continue;
            }
            if (features->size() >= 20 && subsegment_features->size() < 20) {
                cout << "Error, doing folder: " << subsegment_folder_path.string() << endl;
                cout << "Split cloud had size: " << subsegment_features->size() << endl;
                exit(0);
            }

            cout << "Saving features and keypoints..." << endl;
            stringstream ss;
            ss << setw(4) << setfill('0') << counter;
            string identifier = ss.str() + ".pcd";
            boost::filesystem::path feature_path = subsegment_folder_path / (string("feature") + identifier);
            boost::filesystem::path keypoint_path = subsegment_folder_path / (string("keypoint") + identifier);
            pcl::io::savePCDFileBinary(feature_path.string(), *subsegment_features);
            pcl::io::savePCDFileBinary(keypoint_path.string(), *subsegment_keypoints);
            segment_paths.push_back(keypoint_path.string());
            sweep_data.segment_indices.push_back(vt_index);
            cout << "Done saving..." << endl;
            ++counter;
            ++vt_index;
        }
    }

    if (!current_path.empty()) {
        sweep_data.nbr_segments = counter;
        sweep_data.save(current_path / "subsegments");
    }

    summary.index_subsegment_paths.insert(summary.index_subsegment_paths.end(), segment_paths.begin(), segment_paths.end());
    summary.nbr_subsegments = vt_index;

    summary.save(data_path);

    return 0;
}
