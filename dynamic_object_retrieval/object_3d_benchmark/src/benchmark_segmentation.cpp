#include <object_3d_retrieval/supervoxel_segmentation.h>

#include "object_3d_benchmark/benchmark_segmentation.h"
#include "object_3d_benchmark/benchmark_retrieval.h"
#include "object_3d_benchmark/benchmark_overlap.h"

#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/extract_surfel_features.h>

using namespace std;

namespace benchmark_retrieval {

float compute_cloud_volume(CloudT::Ptr& cloud)
{
    float resolution = 0.05f;
    pcl::octree::OctreePointCloud<PointT> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    std::vector<PointT, Eigen::aligned_allocator<PointT> > dummy;
    float centers = octree.getOccupiedVoxelCenters(dummy);
    return centers*resolution*resolution*resolution;
}

vector<CloudT::Ptr> perform_convex_segmentation(CloudT::Ptr& training_map, CloudT::Ptr& training_object,
                                                NormalCloudT::Ptr& training_object_normals,
                                                CloudT::Ptr& query_map, CloudT::Ptr& query_object,
                                                const boost::filesystem::path& query_map_path)
{
    /*
    supervoxel_segmentation ss(0.02f, 0.2f, 0.4f, false);
    supervoxel_segmentation::Graph* g;
    supervoxel_segmentation::Graph* convex_g;
    vector<CloudT::Ptr> supervoxels;
    vector<CloudT::Ptr> segments;
    map<size_t, size_t> indices;
    std::tie(g, convex_g, supervoxels, convex_segments, indices) = ss.compute_convex_oversegmentation(query_map, false);

    delete g;
    delete convex_g;
    */

    dynamic_object_retrieval::sweep_convex_segment_cloud_map sweep_segments(query_map_path.string());

    vector<CloudT::Ptr> segments;
    for (CloudT::Ptr& segment : sweep_segments) {
        segments.push_back(CloudT::Ptr(new CloudT(*segment)));
    }

    return segments;
}

map<string, pair<float, int> > get_segmentation_scores_for_data(const std::function<vector<CloudT::Ptr>(CloudT::Ptr&, CloudT::Ptr&,
                                                                                                        NormalCloudT::Ptr&,
                                                                                                        CloudT::Ptr&, CloudT::Ptr&,
                                                                                                        const boost::filesystem::path&)>& sfunc,
                                                                const boost::filesystem::path& data_path)
{
    // actually we need something more here, we can't do it just for a sweep as we need the previous sweep cloud and object
    // for the training of an objects, will we keep that in memory in a map structure?

    std::vector<std::string> objects_to_check = {"backpack", "trash_bin", "lamp", "chair", "desktop", "pillow", "hanger_jacket", "water_boiler"};

    map<string, pair<CloudT::Ptr, CloudT::Ptr> > map_object_for_instance;
    map<string, NormalCloudT::Ptr> map_normal_for_instance;

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    map<string, pair<float, int> > overlap_ratios;
    map<string, pair<float, int> > volumes;

    for (const string& sweep_xml : folder_xmls) {

        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);
        Eigen::Matrix3f K;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
        CloudT::Ptr sweep_cloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointT>(sweep_xml);
        std::tie(K, camera_transforms) = benchmark_retrieval::get_camera_matrix_and_transforms(sweep_xml);
        boost::filesystem::path sweep_path = boost::filesystem::path(sweep_xml).parent_path();
        SurfelCloudT::Ptr surfel_map(new SurfelCloudT);
        pcl::io::loadPCDFile((sweep_path / "surfel_map.pcd").string(), *surfel_map);

        for (auto tup : dynamic_object_retrieval::zip(labels.objectClouds, labels.objectLabels, labels.objectMasks, labels.objectScanIndices)) {
            CloudT::Ptr cloud;
            string label;
            cv::Mat query_mask;
            size_t scan_index;
            tie(cloud, label, query_mask, scan_index) = tup;
            bool found = false;
            string category;
            for (const std::string& is_check : objects_to_check) {
                if (label.compare(0, is_check.size(), is_check) == 0) {
                    found = true;
                    category = is_check;
                    break;
                }
            }
            if (!found) {
                continue;
            }

            CloudT::Ptr query_cloud = benchmark_retrieval::get_cloud_from_sweep_mask(sweep_cloud, query_mask, camera_transforms[scan_index], K);
            float cloud_volume = compute_cloud_volume(query_cloud);
            pair<float, int>& volume = volumes[category];
            volume.first += cloud_volume;
            volume.second += 1;

            if (map_object_for_instance.count(label) > 0) {
                CloudT::Ptr train_cloud;
                CloudT::Ptr train_map;
                tie(train_cloud, train_map) = map_object_for_instance[label];
                NormalCloudT::Ptr train_normals = map_normal_for_instance[label];

                vector<CloudT::Ptr> segments = sfunc(train_map, train_cloud, train_normals, sweep_cloud, query_cloud, sweep_path);
                double max_overlap = 0.0;
                for (CloudT::Ptr& c : segments) {
                    double overlap = compute_overlap(query_cloud, c, 0.02);
                    if (overlap > max_overlap) {
                        max_overlap = overlap;
                    }
                }

                pair<float, int>& ratio = overlap_ratios[label];
                ratio.first += max_overlap;
                ratio.second += 1;
            }

            NormalCloudT::Ptr query_normals = dynamic_object_retrieval::compute_surfel_normals(surfel_map, query_cloud);
            map_object_for_instance[label] = make_pair(query_cloud, sweep_cloud);
            map_normal_for_instance[label] = query_normals;
        }
    }

    for (pair<const string, pair<float, int> >& volume : volumes) {
        cout << volume.first << " has mean volume: " << volume.second.first / float(volume.second.second) << endl;
    }

    return overlap_ratios;
}

} // namespace benchmark_retrieval
