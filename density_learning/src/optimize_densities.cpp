#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "dynamic_object_retrieval/visualize.h"

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <eigen_cereal/eigen_cereal.h>
#include <pcl/common/centroid.h>
#include <pcl/octree/octree.h>
#include "dynamic_object_retrieval/definitions.h"

/**
 * Notes on what is done in this file:
 *
 * We basically iterate over segment sizes / keypoint density given size / size of rest of vocabulary
 * and for all of these combinations we compute a score that is inserted into a 3-dimensional matrix
 *
 * This matrix is then used to optimize over the best comibination of segment size -> keypoint density
 * for maximizing the overall retrieval score
 *
 * The working hypothesis here is that the success of the retrieval for one given size and density
 * depends mostly on the overall number of features of the other sizes in the vocabulary and not so
 * much on the exact distribution of features between the different sizes. This is the main
 * assumption here to make the optimization tractable
 */

using namespace std;
using namespace dynamic_object_retrieval;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

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

template <typename DataT>
void save_data(const boost::filesystem::path& path, const DataT& data)
{
    ofstream out(path.string());
    {
        cereal::JSONOutputArchive archive_o(out);
        archive_o(data);
    }
    out.close();
}

template <typename DataT>
void save_binary_data(const boost::filesystem::path& path, const DataT& data)
{
    ofstream out(path.string(), std::ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(data);
    }
    out.close();
}

template <typename DataT>
void load_data(const boost::filesystem::path& path, DataT& data)
{
    ifstream in(path.string());
    {
        cereal::JSONInputArchive archive_i(in);
        archive_i(data);
    }
    in.close();
}

template <typename DataT>
void load_binary_data(const boost::filesystem::path& path, DataT& data)
{
    ifstream in(path.string(), std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(data);
    }
    in.close();
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the vocabulary..." << endl;
        return 0;
    }

    //train_vocabulary(boost::filesystem::path(argv[1]));
    boost::filesystem::path vocabulary_path(argv[1]); // DEBUG
    vocabulary_summary summary;
    summary.load(vocabulary_path);
    boost::filesystem::path data_path(summary.noise_data_path);

    boost::filesystem::path training_path = vocabulary_path / "training";
    boost::filesystem::create_directory(training_path);

    vector<float> densities;
    load_data(training_path / "densities.json", densities);

    cout << "densities: " << endl;
    for (int dens_ind = 0; dens_ind < densities.size(); ++dens_ind) {
        cout << densities[dens_ind] << " ";
    }
    cout << endl;

    vector<float> dividers;
    load_data(training_path / "dividers.json", dividers);

    cout << "dividers: " << endl;
    for (int dens_ind = 0; dens_ind < dividers.size(); ++dens_ind) {
        cout << dividers[dens_ind] << " ";
    }
    cout << endl;

    vector<float> density_threshold_mapping;
    load_data(training_path / "density_threshold_mapping.json", density_threshold_mapping);

    cout << "density_threshold_mapping: " << endl;
    for (int dens_ind = 0; dens_ind < density_threshold_mapping.size(); ++dens_ind) {
        cout << density_threshold_mapping[dens_ind] << " ";
    }
    cout << endl;

    vector<int> prior;
    load_data(training_path / "prior.json", prior);

    vector<int> size_numbers;
    load_data(training_path / "size_numbers.json", size_numbers);

    map<string, string> path_labels;
    load_binary_data(data_path / "segment_path_labels.cereal", path_labels);
    for (const pair<string, string>& p : path_labels) {
        cout << p.first << " -> " << p.second << endl;
    }

    Eigen::MatrixXf mean_keypoints;
    load_binary_data(training_path / "mean_keypoints.cereal", mean_keypoints);

    vector<float> mean_sizes;
    load_data(training_path / "mean_sizes.json", mean_sizes);

    vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf> > size_errors(dividers.size());
    vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf> > size_times(dividers.size());
    vector<Eigen::MatrixXi, Eigen::aligned_allocator<Eigen::MatrixXi> > size_nbr_features(dividers.size());
    vector<Eigen::MatrixXi, Eigen::aligned_allocator<Eigen::MatrixXi> > size_nbr_others(dividers.size());
    for (int size_ind = 0; size_ind < dividers.size(); ++size_ind) {
        load_binary_data(training_path / (string("errors") + to_string(size_ind) + ".cereal"), size_errors[size_ind]);
        load_binary_data(training_path / (string("times") + to_string(size_ind) + ".cereal"), size_times[size_ind]);
        load_binary_data(training_path / (string("features") + to_string(size_ind) + ".cereal"), size_nbr_features[size_ind]);
        load_binary_data(training_path / (string("others") + to_string(size_ind) + ".cereal"), size_nbr_others[size_ind]);
        cout << "size_errors: " << endl;
        cout << size_errors[size_ind] << endl;
        cout << "size_times: " << endl;
        cout << size_times[size_ind] << endl;
        cout << "size_nbr_features: " << endl;
        cout << size_nbr_features[size_ind] << endl;
        cout << "size_nbr_other_features: " << endl;
        cout << size_nbr_others[size_ind] << endl;
    }

    float alpha = 1e-2f;//0.00000001f;
    float beta = 8e-8f;

    float real_priors[6];
    int prior_sum = std::accumulate(prior.begin(), prior.end(), 0, std::plus<int>());
    for (int i = 0; i < 6; ++i) {
        real_priors[i] = float(prior[i])/float(prior_sum);
    }

    const int ds[] = {0, 1, 2, 3, 4};

    float min_val = std::numeric_limits<float>::infinity();
    int min_combo[6];
    float tilde_err[6];
    float est_feature_diff[6];
    for (int i1 : ds) for (int i2 : ds) for (int i3 : ds) for (int i4 : ds) for (int i5 : ds)
        for (int i6 : ds) { //for (int i7 : ds) for (int i8 : ds) for (int i9 : ds) for (int i10 : ds) {
            // for every size, need to go through the others' densitites,
            // the density of the own one for every index is already fixed
            //int inds[] = {i1, i2, i3, i4, i5, i6, i7, i8, i9, i10};
            int inds[] = {i1, i2, i3, i4, i5, i6};
            float err[6];
            float feat_diff[6];
            //Eigen::Matrix<float, 10, 5> local_scores;
            //local_scores.setZero();
            float est_features = 0.0f;
            for (int size_ind = 0; size_ind < 6; ++size_ind) {
                //est_features += densities[inds[size_ind]]*dividers[size_ind]*size_numbers[size_ind];
                est_features += float(size_numbers[size_ind])*mean_keypoints(size_ind, inds[size_ind]);
            }
            float val = 0.0f;
            for (int size_ind = 0; size_ind < 6; ++size_ind) {
                float local_min = std::numeric_limits<float>::infinity();
                float local_err;
                for (int dens_ind = 0; dens_ind < 5; ++dens_ind) {
                    // I need the numbers here for all the different sizes as well (the actual ones in the vocabulary)
                    float tilde_features = float(size_nbr_features[size_ind](inds[size_ind], dens_ind) + size_nbr_others[size_ind](inds[size_ind], dens_ind));
                    float error = size_errors[size_ind](inds[size_ind], dens_ind);
                    //float time = size_times[size_ind](inds[size_ind], dens_ind);
                    float p = real_priors[size_ind]*error + alpha*std::abs(est_features - tilde_features)/est_features;
                    if (p < local_min) {
                        local_min = p;
                        local_err = size_errors[size_ind](inds[size_ind], dens_ind);
                        feat_diff[size_ind] = std::abs(est_features - tilde_features)/est_features;
                    }
                }

                err[size_ind] = local_err;
                val += local_min;
            }

            val += beta*est_features;
            //val += 0.000000042f*est_features;

            if (val < min_val) {
                min_val = val;
                std::copy(std::begin(inds), std::end(inds), std::begin(min_combo));
                std::copy(std::begin(err), std::end(err), std::begin(tilde_err));
                std::copy(std::begin(feat_diff), std::end(feat_diff), std::begin(est_feature_diff));
            }

        }

    cout << "Best value: " << min_val;
    cout << "Min combo:" << endl;
    for (int i = 0; i < 6; ++i) {
        cout << min_combo[i] << " ";
    }
    cout << endl;
    save_data(training_path / "min_combo.json", min_combo);

    // what do we need to turn this into a nice graph?

    // dividers on x axis
    // actual errors on Y axis (aha, we need to benchmark one more time, maybe later)
    // approximated errors on Y axis (need to save this in the loop)
    // number of keypoints (for this we also need the dividers)
    // alright, that should be it

    cout << "Alpha: " << alpha << endl;
    cout << "Beta: " << beta << endl;

    cout << "Dividers: " << endl;
    for (int dens_ind = 0; dens_ind < dividers.size(); ++dens_ind) {
        cout << dividers[dens_ind] << " ";
    }
    cout << endl;

    cout << "Densities: " << endl;
    for (int dens_ind = 0; dens_ind < dividers.size(); ++dens_ind) {
        cout << densities[min_combo[dens_ind]] << " ";
    }
    cout << endl;

    cout << "Number keypoints: " << endl;
    for (int size_ind = 0; size_ind < dividers.size(); ++size_ind) {
        cout << mean_keypoints(size_ind, min_combo[size_ind]) << " ";
        //cout << dividers[dens_ind]*densities[min_combo[dens_ind]] << " ";
    }
    cout << endl;

    cout << "Mean sizes: " << endl;
    for (int size_ind = 0; size_ind < dividers.size(); ++size_ind) {
        cout << mean_sizes[size_ind] << " ";
        //cout << dividers[dens_ind]*densities[min_combo[dens_ind]] << " ";
    }
    cout << endl;

    cout << "Tilde errors: " << endl;
    for (int dens_ind = 0; dens_ind < dividers.size(); ++dens_ind) {
        cout << tilde_err[dens_ind] << " ";
    }
    cout << endl;

    cout << "Estimated feature difference: " << endl;
    for (int dens_ind = 0; dens_ind < dividers.size(); ++dens_ind) {
        cout << est_feature_diff[dens_ind] << " ";
    }
    cout << endl;

    return 0;
}
