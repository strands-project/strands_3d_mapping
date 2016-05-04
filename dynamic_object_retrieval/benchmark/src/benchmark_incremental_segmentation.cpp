#include "object_3d_benchmark/benchmark_segmentation.h"
#include "object_3d_benchmark/benchmark_overlap.h"

#define VT_PRECOMPILE
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/extract_surfel_features.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

#define WITH_SURFEL_NORMALS 1

using namespace std;

using PointT = pcl::PointXYZRGB;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

grouped_vocabulary_tree<HistT, 8> vt;
map<grouped_vocabulary_tree<HistT, 8>::node*, int> mapping;
map<int, grouped_vocabulary_tree<HistT, 8>::node*> inverse_mapping;

void visualize_adjacencies(vector<CloudT::Ptr>& segments, const set<pair<int, int> >& adjacencies)
{
    CloudT::Ptr visualization_cloud(new CloudT);

    CloudT::Ptr centroids(new CloudT);
    for (CloudT::Ptr& c : segments) {
        *visualization_cloud += *c;
        Eigen::Vector4f point;
        pcl::compute3DCentroid(*c, point);
        centroids->push_back(PointT());
        centroids->back().getVector4fMap() = point;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(visualization_cloud);
    viewer->addPointCloud<PointT>(visualization_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    int i = 0;
    for (const pair<int, int>& a : adjacencies) {
        string line_id = string("line") + to_string(i);
        cout << "Centroids size: " << centroids->size() << endl;
        cout << "Adjacency: (" << a.first << ", " << a.second << ")" << endl;
        viewer->addLine<PointT>(centroids->at(a.first), centroids->at(a.second), 1.0, 0.0, 0.0, line_id);
        ++i;
    }

    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

vector<CloudT::Ptr> perform_incremental_segmentation(CloudT::Ptr& training_map, CloudT::Ptr& training_object,
                                                     NormalCloudT::Ptr& training_object_normals,
                                                     CloudT::Ptr& query_map, CloudT::Ptr& query_object,
                                                     const boost::filesystem::path& query_map_path)
{
    cout << "Data path: " << query_map_path.parent_path().parent_path().parent_path().string() << endl;
    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(query_map_path.parent_path().parent_path().parent_path().string(), true);

    cout << query_map_path.string() << endl;

    // DEBUG: only for chair comparison
    //int sweep_index = 88;
    //bool found;
    int sweep_index;
    bool found = false;
    for (sweep_index = 0; sweep_index < folder_xmls.size(); ++sweep_index) {
        boost::filesystem::path sweep_path = boost::filesystem::path(folder_xmls[sweep_index]).parent_path();
        if (sweep_path == query_map_path) {
            found = true;
            break;
        }
    }
    if (!found) {
        cout << "WTF? Did not find any matching sweeps..." << endl;
        exit(-1);
    }

    dynamic_object_retrieval::sweep_convex_segment_cloud_map segment_map(query_map_path);
    vector<CloudT::Ptr> segments;
    for (CloudT::Ptr& segment : segment_map) {
        segments.push_back(CloudT::Ptr(new CloudT(*segment)));
    }

    // first, we need to extract some features for the query cloud
    HistCloudT::Ptr query_cloud(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT);
#if WITH_SURFEL_NORMALS
    dynamic_object_retrieval::compute_features(query_cloud, keypoints, training_object, training_object_normals, false, true);
#else
    pfhrgb_estimation::compute_surfel_features(query_cloud, keypoints, training_object, false, true);
#endif

    vector<vocabulary_vector> vectors;
    set<pair<int, int> > adjacencies;
    vt.load_cached_vocabulary_vectors_for_group(vectors, adjacencies, sweep_index); // sweep index needed!

    //visualize_adjacencies(segments, adjacencies);

    std::vector<grouped_vocabulary_tree<HistT, 8>::result_type> scores;
    vt.top_combined_similarities(scores, query_cloud, 0);

    cout << "Scores size: " << scores.size() << endl;

    found = false;
    int start_index;
    float min_score = 1000.0f;//std::numeric_limits<float>::infinity();
    for (int i = 0; i < scores.size(); ++i) {
        if (scores[i].group_index == sweep_index) {
            //cout << "Score: " << scores[i].score << endl;
            //cout << "Subgroup index: " << scores[i].subgroup_index << endl;
            if (scores[i].score < min_score && benchmark_retrieval::compute_overlap(query_object, segments[scores[i].subgroup_index]) > 0.1) {
                found = true;
                start_index = scores[i].subgroup_index;
                min_score = scores[i].score;
            }
        }
    }
    cout << "Chose: " << min_score << " with subgroup index " << start_index << endl;
    if (!found) {
        cout << "WTF? Did not find any matching min segment..." << endl;
        //start_index = -1;
        //exit(-1);
        return segments;
    }

    vector<int> selected_indices;
    // get<1>(scores[i])) is actually the index within the group!
    // if I use -1 here, I get correct results. this means that sweep_index must be correct
    // however, start index is never correct, so maybe group_subgroup map is not correct?
    // e.g. when I get the segments back from selected_indices when using -1, this also returns
    // correct results so it seems like that is the correct way to decide subgroup_index
    double score = vt.compute_min_combined_dist(selected_indices, query_cloud, vectors, adjacencies,
                                                mapping, inverse_mapping, start_index); // segment index in sweep needed!
    //vector<int> selected_indices = {start_index};

    vector<CloudT::Ptr> single_cloud;
    single_cloud.push_back(CloudT::Ptr(new CloudT));

    for (int segment_index : selected_indices) {
        *single_cloud[0] += *segments[segment_index];
    }
    //*single_cloud[0] += *segments[selected_indices[0]];

    cout << "Scores size: " << scores.size() << endl;
    cout << "Start index: " << start_index << endl;
    cout << "Selected indices at 0: " << selected_indices[0] << endl;
    /*CloudT::Ptr vis_cloud(new CloudT);
    *vis_cloud += *query_object;
    for (int i = 0; i < segments[selected_indices[0]]->size(); ++i) {
        PointT p = segments[selected_indices[0]]->at(i);
        p.x += 1.0f;
        vis_cloud->push_back(p);
    }*/
    //*vis_cloud += *;
    //dynamic_object_retrieval::visualize(query_object);
    //dynamic_object_retrieval::visualize(single_cloud[0]);

    return single_cloud;
}

// ./benchmark_incremental_segmentation ~/Data/G4S_Working/Waypoint26/20150507/patrol_run_20/ ~/Data/G4S_Working/vocabulary_inc/
// ~/Workspace/dynamic_objects/object_3d_retrieval/comparisons/notes/training_cloud.pcd ~/Data/G4S_Working/Waypoint26/20150507/patrol_run_20/room_2/rgb_0002_label_4.pcd
// ~/Data/G4S_Working/Waypoint26/20150507/patrol_run_20/room_2

// ./benchmark_incremental_segmentation ~/Data/chair_comparison/20160107/patrol_run_17 ~/Data/KTH_longterm_surfels/vocabulary_chair2/
// ~/Data/chair_comparison/20160108/patrol_run_20/query_chair/chair.pcd ~/Data/chair_comparison/20160107/patrol_run_17/query_chair/chair.pcd
// ~/Data/chair_comparison/20160107/patrol_run_17/room_5/

// ./benchmark_incremental_segmentation ~/Data/chair_comparison/20160107/patrol_run_17 ~/Data/KTH_longterm_surfels/vocabulary_chair2/
// ~/Data/chair_comparison/20160107/patrol_run_15/dest_chair/chair.pcd ~/Data/chair_comparison/20160107/patrol_run_17/query_chair/chair.pcd
// ~/Data/chair_comparison/20160107/patrol_run_17/room_5/

// ./benchmark_incremental_segmentation ~/Data/chair_comparison/20160107/patrol_run_15/ ~/Data/KTH_longterm_surfels/vocabulary_chair/
// ~/Data/chair_comparison/20160107/patrol_run_17/query_chair/chair.pcd ~/Data/chair_comparison/20160107/patrol_run_15/dest_chair/chair.pcd
// ~/Data/chair_comparison/20160107/patrol_run_15/room_4/
/*
int main(int argc, char** argv)
{
    if (argc < 6) {
        cout << "Please provide the path to the annotated data and the vocabulary path..." << endl;
        cout << "And the path to the training object cloud and the query object cloud..." << endl;
        cout << "And the sweep path for the query object.." << endl;
        return 0;
    }

    boost::filesystem::path data_path(argv[1]);
    boost::filesystem::path vocabulary_path(argv[2]);
    boost::filesystem::path training_object_path(argv[3]);
    boost::filesystem::path query_object_path(argv[4]);
    boost::filesystem::path query_map_path(argv[5]);

    CloudT::Ptr training_object(new CloudT);
    pcl::io::loadPCDFile(training_object_path.string(), *training_object);
    CloudT::Ptr query_object(new CloudT);
    pcl::io::loadPCDFile(query_object_path.string(), *query_object);

    // we're not using these here so set them to 0 for now
    CloudT::Ptr training_map(new CloudT);
    CloudT::Ptr query_map(new CloudT);

    vt.set_cache_path(vocabulary_path.string());
    dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
    vt.set_min_match_depth(3);
    vt.compute_normalizing_constants();
    vt.get_node_mapping(mapping);
    for (const pair<grouped_vocabulary_tree<HistT, 8>::node*, int>& u : mapping) {
        inverse_mapping.insert(make_pair(u.second, u.first));
    }

    vector<CloudT::Ptr> top_segment = perform_incremental_segmentation(training_map, training_object, query_map, query_object, query_map_path);

    //boost::filesystem::path segmented_path = query_object_path.parent_path() / (query_object_path.stem().string() + "_segmented.pcd");
    boost::filesystem::path segmented_path("incremental_top_match.pcd");
    pcl::io::savePCDFileBinary(segmented_path.string(), *top_segment[0]);

    return 0;
}
*/

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide the path to the annotated data and the vocabulary path..." << endl;
        return 0;
    }

    boost::filesystem::path data_path(argv[1]);
    boost::filesystem::path vocabulary_path(argv[2]);

    vt.set_cache_path(vocabulary_path.string());
    dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
    vt.set_cache_path(vocabulary_path.string());
    vt.set_min_match_depth(3);
    vt.compute_normalizing_constants();
    vt.get_node_mapping(mapping);
    for (const pair<grouped_vocabulary_tree<HistT, 8>::node*, int>& u : mapping) {
        inverse_mapping.insert(make_pair(u.second, u.first));
    }

    map<string, pair<float, int> > overlap_ratios = benchmark_retrieval::get_segmentation_scores_for_data(&perform_incremental_segmentation, data_path);

    std::vector<std::string> objects_to_check = {"backpack", "trash_bin", "lamp", "chair", "desktop", "pillow", "hanger_jacket", "water_boiler"};

    pair<float, int> total_ratio;
    map<string, pair<float, int> > category_ratios;
    for (const pair<string, pair<float, int> >& ratio : overlap_ratios) {
        bool found = false;
        string category;
        for (const std::string& is_check : objects_to_check) {
            if (ratio.first.compare(0, is_check.size(), is_check) == 0) {
                found = true;
                category = is_check;
                break;
            }
        }
        if (!found) {
            continue;
        }
        total_ratio.first += ratio.second.first; total_ratio.second += ratio.second.second;

        pair<float, int>& category_ratio = category_ratios[category];
        category_ratio.first += ratio.second.first;
        category_ratio.second += ratio.second.second;
    }

    total_ratio.first /= float(total_ratio.second);
    cout << "Total ratio: " << total_ratio.first << " in " << total_ratio.second << " places" << endl;
    for (pair<const string, pair<float, int> >& ratio : category_ratios) {
        ratio.second.first /= float(ratio.second.second);
        cout << ratio.first << ": " << ratio.second.first << " in " << ratio.second.second << " places" << endl;
    }

    return 0;
}
