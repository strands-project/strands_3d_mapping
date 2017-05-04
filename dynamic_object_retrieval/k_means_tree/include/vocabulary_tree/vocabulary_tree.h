#ifndef VOCABULARY_TREE_H
#define VOCABULARY_TREE_H

#include "k_means_tree/k_means_tree.h"
#include <cereal/types/map.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>

/*
 * vocabulary_tree
 *
 * This class describes a collection of points that occurr together
 *
 */

struct inverted_file {
    std::map<int, int> source_id_freqs; // change this to uint32_t, uint32_t
    template <class Archive> void serialize(Archive& archive)
    {
        archive(source_id_freqs);
    }
};

// this is used for storing vocabulary vectors outside of the voc tree
struct vocabulary_vector
{
    // the norm of the vector
    double norm;
    size_t subgroup;

    // the values of the vector, both unnormalized histogram and normalized
    // the normalized value is redundant as it can be computes from 1st+norm
    std::unordered_map<int, std::pair<int, double> > vec;

    // for cereal serialization
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(norm, subgroup, vec);
    }
};

struct vocabulary_result {
    int index;
    float score;

    vocabulary_result(int index, float score) : index(index), score(score) {}
    vocabulary_result() {}
};

template <typename Point, size_t K>
class vocabulary_tree : public k_means_tree<Point, K, inverted_file> {
protected:

    using super = k_means_tree<Point, K, inverted_file>;
    using typename super::PointT;
    using typename super::CloudT;
    using typename super::CloudPtrT;

public:

    using result_type = vocabulary_result; //cloud_idx_score;
    using typename super::leaf;
    using typename super::node;

protected:

    std::vector<int> indices; // the source indices of the points (image ids of features), change this to uint32_t
    std::map<int, double> db_vector_normalizing_constants; // normalizing constants for the p vectors
    double N; // number of sources (images) in database
    static const bool normalized = true;
    int matching_min_depth;

protected:

    double pexp(const double v) const;
    double proot(const double v) const;
    void compute_query_vector(std::map<node*, int>& query_id_freqs, CloudPtrT& query_cloud);
    double compute_query_vector(std::map<node*, std::pair<double, int> >& query_id_freqs, CloudPtrT& query_cloud);
    void source_freqs_for_node(std::map<int, int>& source_id_freqs, node* n) const;
    void normalizing_constants_for_node(std::map<int, int>& normalizing_constants, node* n, int current_depth);

    void unfold_nodes(std::vector<node*>& path, node* n, const PointT& p, std::map<node*, double>& active);
    void get_path_for_point(std::vector<node*>& path, const PointT& point, std::map<node*, double>& active);
    void compute_vocabulary_vector(std::map<node*, double>& query_id_freqs,
                                   CloudPtrT& query_cloud, std::map<node*, double>& active);

public:

    void query_vocabulary(std::vector<result_type>& results, CloudPtrT& query_cloud, size_t nbr_results);
    void compute_new_weights(std::map<int, double>& original_norm_constants, std::map<node*, double>& original_weights,
                             std::vector<std::pair<int, double> >& weighted_indices, CloudPtrT& query_cloud);
    void compute_new_weights(std::map<int, double>& original_norm_constants,
                             std::map<node*, double>& original_weights,
                             std::vector<std::pair<std::set<int>, double> >& weighted_indices,
                             CloudPtrT& query_cloud);
    void restore_old_weights(std::map<int, double>& original_norm_constants, std::map<node*, double>& original_weights);

    double compute_vocabulary_norm(CloudPtrT& cloud);
    double compute_min_combined_dist(std::vector<int>& smallest_ind_combination, CloudPtrT& cloud, std::vector<vocabulary_vector>& smaller_freqs,
                                     std::set<std::pair<int, int> >& adjacencies, std::map<node*, int>& mapping, std::map<int, node*>& inverse_mapping, int hint);

    void set_min_match_depth(int depth);
    void compute_normalizing_constants(); // this also computes the weights
    bool empty() const { return indices.empty(); }

    void clear()
    {
        super::clear();
        indices.clear();
        db_vector_normalizing_constants.clear();
        N = 0;
        for (leaf* l : super::leaves) {
            l->data->source_id_freqs.clear();
        }
    }

    int max_ind() const { return *(std::max_element(indices.begin(), indices.end())) + 1; }
    void set_input_cloud(CloudPtrT& new_cloud, std::vector<int>& new_indices);
    void append_cloud(CloudPtrT& extra_cloud, std::vector<int>& extra_indices, bool store_points = true);
    void add_points_from_input_cloud(bool save_cloud = true);

    void top_combined_similarities(std::vector<result_type>& scores, CloudPtrT& query_cloud, size_t nbr_results);
    void debug_similarities(std::vector<result_type>& scores, CloudPtrT& query_cloud, size_t nbr_results);

    double compute_query_index_vector(std::map<int, double>& query_index_freqs, CloudPtrT& query_cloud, std::map<node*, int>& mapping);
    void compute_query_index_vector(std::map<int, int>& query_index_freqs, CloudPtrT& query_cloud, std::map<node*, int>& mapping);
    vocabulary_vector compute_query_index_vector(CloudPtrT& query_cloud, std::map<node *, int> &mapping);

    double compute_query_vector(std::map<node*, double>& query_id_freqs, CloudPtrT& query_cloud);
    double compute_distance(const std::map<node*, double>& vec1, double norm1, const std::map<node*, double>& vec2, double norm2) const;

    /*
    template <class Archive> void save(Archive& archive) const;
    template <class Archive> void load(Archive& archive);
    */
    template <class Archive>
    void save(Archive& archive) const
    {
        super::save(archive);
        archive(indices);
        archive(db_vector_normalizing_constants);
        archive(N);
    }

    template <class Archive>
    void load(Archive& archive)
    {
        super::load(archive);
        archive(indices);
        archive(db_vector_normalizing_constants);
        archive(N);
        std::cout << "Finished loading vocabulary_tree" << std::endl;
    }

    vocabulary_tree() : super(5), matching_min_depth(1) {} // DEBUG: depth = 5 always used

};

#ifndef VT_PRECOMPILE
#include "vocabulary_tree.hpp"
#endif

#endif // VOCABULARY_TREE_H
