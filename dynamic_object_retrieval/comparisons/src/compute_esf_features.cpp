#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>

#include <pcl/features/cvfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/features/esf.h>

#include <flann/io/hdf5.h>

#include "eigen_cereal/eigen_cereal.h"
#include <cereal/archives/binary.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalCloudT = pcl::PointCloud<pcl::Normal>;
using EFSs = pcl::PointCloud<pcl::ESFSignature640>;
using esf_model = pair<string, vector<float> > ;

inline void nearestKSearch(flann::Index<flann::L1<float> > &index, const esf_model &model,
                           int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
    // Query point
    flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
    memcpy(&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));
    indices = flann::Matrix<int>(new int[k], 1, k);
    distances = flann::Matrix<float>(new float[k], 1, k);
    index.knnSearch(p, indices, distances, k, flann::SearchParams(512));
    delete[] p.ptr();
}

void compute_esf_features(EFSs::Ptr& efss, CloudT::ConstPtr cloud)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // Compute ESF
    pcl::ESFEstimation<PointT, pcl::ESFSignature640> EFS;
    EFS.setInputCloud(cloud);
    EFS.setSearchMethod(tree);
    EFS.compute(*efss);

    if (efss->size() != 1) {
        cout << "EFH feature has size: " << efss->size() << endl;
        exit(-1);
    }
}

void compute_and_save(CloudT::ConstPtr cloud, const boost::filesystem::path& segments_path, size_t current_segment)
{
    cout << segments_path.string() << endl;
    cout << current_segment << endl;
    EFSs::Ptr efss(new EFSs);
    if ((segments_path.string() == "/home/nbore/Data/KTH_longterm_surfels//20140912/patrol_run_100/room_3/convex_segments" &&
            current_segment == 79) ||
        (segments_path.string() == "/home/nbore/Data/KTH_longterm_surfels//20140912/patrol_run_104/room_3/convex_segments" &&
            current_segment == 95) ||
        (segments_path.string() == "/home/nbore/Data/KTH_longterm_surfels//20140921/patrol_run_112/room_2/convex_segments" &&
                current_segment == 24)) {
        efss->push_back(pcl::ESFSignature640());
        for (size_t j = 0; j < 640; ++j) {
            efss->at(0).histogram[j] = 0.0f;
        }
    }
    else {
        compute_esf_features(efss, cloud);
    }
    std::stringstream ss;
    ss << "esf_feature" << std::setw(4) << std::setfill('0') << current_segment;
    boost::filesystem::path esf_file = segments_path / (ss.str() + ".cereal");
    esf_model histogram;
    histogram.first = esf_file.string();
    histogram.second.resize(640);
    for (size_t j = 0; j < 640; ++j) {
        histogram.second[j] = efss->at(0).histogram[j];
    }
    ofstream out(esf_file.string(), std::ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(histogram);
    }
    out.close();
}

void load_signatures(vector<esf_model>& models, const boost::filesystem::path& data_path)
{
    dynamic_object_retrieval::convex_segment_cloud_map segment_clouds(data_path);
    dynamic_object_retrieval::convex_segment_map segment_paths(data_path);
    dynamic_object_retrieval::convex_sweep_index_map sweep_indices(data_path);

    int last_sweep = -1;
    size_t counter = 0;
    // compute and save ESF signatires
    for (auto tup : dynamic_object_retrieval::zip(segment_clouds, segment_paths, sweep_indices)) {
        CloudT::Ptr cloud;
        boost::filesystem::path path;
        size_t sweep_index;
        tie(cloud, path, sweep_index) = tup;
        if (sweep_index != last_sweep) {
            last_sweep = sweep_index;
            counter = 0;
        }

        std::stringstream ss;
        ss << "esf_feature" << std::setw(4) << std::setfill('0') << counter;
        boost::filesystem::path esf_file = path.parent_path() / (ss.str() + ".cereal");
        models.push_back(esf_model());
        ifstream in(esf_file.string(), std::ios::binary);
        {
            cereal::BinaryInputArchive archive_i(in);
            archive_i(models.back()); // Is reading of the K matrix slowing this down?
        }
        in.close();

        ++counter;
    }
}

void build_and_save_tree(const boost::filesystem::path& data_path)
{
    boost::filesystem::path kdtree_idx_file_name = data_path / "kdtree_esf.idx";
    boost::filesystem::path training_data_h5_file_name = data_path / "training_data_esf.h5";
    boost::filesystem::path training_data_list_file_name = data_path / "training_data_esf.list";

    vector<esf_model> models;
    load_signatures(models, data_path);

    cout << __FILE__ << ", " << __LINE__ << endl;
    cout << models.size () << endl;
    cout << models[0].second.size () << endl;

    // Convert data into FLANN format
    flann::Matrix<float> data(new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

    cout << __FILE__ << ", " << __LINE__ << endl;

    for (size_t i = 0; i < data.rows; ++i) {
        for (size_t j = 0; j < data.cols; ++j) {
            data[i][j] = models[i].second[j];
        }
    }

    cout << __FILE__ << ", " << __LINE__ << endl;

    // Save data to disk (list of models)
    flann::save_to_file(data, training_data_h5_file_name.string(), "training_data_esf");
    ofstream fs;
    fs.open(training_data_list_file_name.string());
    for (size_t i = 0; i < models.size (); ++i) {
        fs << models[i].first << "\n";
    }
    fs.close ();

    cout << __FILE__ << ", " << __LINE__ << endl;

    // Build the tree index and save it to disk
    pcl::console::print_error("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str(), (int)data.rows);
    flann::Index<flann::L1<float> > index(data, flann::LinearIndexParams());
    index.buildIndex();
    index.save(kdtree_idx_file_name.string());
    delete[] data.ptr();
}

void compute_esf_features(const boost::filesystem::path& data_path)
{
    dynamic_object_retrieval::convex_segment_cloud_map segment_clouds(data_path);
    dynamic_object_retrieval::convex_segment_map segment_paths(data_path);
    dynamic_object_retrieval::convex_sweep_index_map sweep_indices(data_path);

    int last_sweep = -1;
    size_t counter = 0;
    // compute and save ESF signatires
    for (auto tup : dynamic_object_retrieval::zip(segment_clouds, segment_paths, sweep_indices)) {
        CloudT::Ptr cloud;
        boost::filesystem::path path;
        size_t sweep_index;
        tie(cloud, path, sweep_index) = tup;
        if (sweep_index != last_sweep) {
            last_sweep = sweep_index;
            counter = 0;
        }
        compute_and_save(cloud, path.parent_path(), counter);
        ++counter;
    }

    build_and_save_tree(data_path);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide the path to the data..." << endl;
        return 0;
    }

    boost::filesystem::path data_path(argv[1]);
    compute_esf_features(data_path);

    return 0;
}
