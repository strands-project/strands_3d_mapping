#ifndef GROUPED_VOCABULARY_TREE_H
#define GROUPED_VOCABULARY_TREE_H

#include "vocabulary_tree/vocabulary_tree.h"

#include <unordered_map>
#include <Eigen/Sparse>


/*
 * k_means_tree
 *
 * This class describes a collection of points that occurr together
 *
 */

template <typename Point, size_t K>
class grouped_vocabulary_tree : public vocabulary_tree<Point, K> {
protected:

    using super = vocabulary_tree<Point, K>;
    using typename super::leaf;
    using typename super::node;
    using typename super::PointT;
    using typename super::CloudT;
    using typename super::CloudPtrT;
    using typename super::leaf_range;

public:

    using cloud_idx_score = std::pair<int, double>;

public: // protected:

    std::unordered_map<int, int> index_group; // why isn't this a vector????
    std::unordered_map<int, std::pair<int, int> > group_subgroup;
    size_t nbr_points;
    size_t nbr_groups;

protected:

    //std::vector<Eigen::SparseVector<int>, Eigen::aligned_allocator<Eigen::SparseVector<int> > > leaf_vocabulary_vectors;
    std::vector<std::unordered_map<int, int> > leaf_vocabulary_vectors;
    std::vector<std::unordered_map<int, int> > intermediate_node_vocabulary_vectors;
    std::map<node*, leaf_range> leaf_ranges;
    unordered_map<int, int> min_group_index;

protected:

    int max_index;
    double calculate_similarity(int i, int j);
    //void compute_node_vocabulary_vector(Eigen::SparseVector<int> &node_vector, node* n);
    void compute_node_vocabulary_vector(unordered_map<int, double>& node_vector, node* n);
    leaf_range compute_vector_for_nodes_at_depth(node* n, int depth, int current_depth);
    void group_normalizing_constants_for_node(std::map<int, int>& normalizing_constants, node* n, int current_depth);

    void recursive_create_vocabulary_vector_from_ind(std::map<node*, double>& vvector, int i, node* n, int current_depth);

public:

    void get_subgroups_for_group(std::set<int>& subgroups, int group_id);
    int get_id_for_group_subgroup(int group_id, int subgroup_id);
    double get_norm_for_group_subgroup(int group_id, int subgroup_id);
    void create_vocabulary_vector_from_ind(std::map<node*, double>& vvector, int i);
    void compute_vocabulary_vector_for_group_with_subgroup(std::map<node*, double>& vvector, int group_id, int subgroup_id);

    void merge_maps(std::map<int, int>& map1, const std::map<int, int>& map2); // should be protected

    void compute_leaf_vocabulary_vectors();
    void compute_intermediate_node_vocabulary_vectors();
    void compute_min_group_indices();
    void compute_group_normalizing_constants();

    void set_input_cloud(CloudPtrT& new_cloud, std::vector<pair<int, int> >& indices);
    void append_cloud(CloudPtrT& extra_cloud, vector<pair<int, int> >& indices, bool store_points = true);
    void add_points_from_input_cloud(bool save_cloud = true);
    void top_grouped_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results = 20);
    void top_combined_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results);
    void top_optimized_similarities(std::vector<std::tuple<int, int, double> >& scores, CloudPtrT& query_cloud, size_t nbr_results);
    //void compute_node_difference(std::map<int, int>& sub_freqs, std::map<int, double>& map_scores,
    //                             node* n, std::map<node*, double>& qvec, int current_depth);

    template <class Archive> void save(Archive& archive) const;
    template <class Archive> void load(Archive& archive);

    void save_group_associations(const string& group_file);
    void load_group_associations(const string& group_file);

    grouped_vocabulary_tree() : super(), nbr_points(0), nbr_groups(0) {}
};

#include "grouped_vocabulary_tree.hpp"

#endif // GROUPED_VOCABULARY_TREE_H
