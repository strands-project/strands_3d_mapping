#ifndef GROUPED_VOCABULARY_TREE_H
#define GROUPED_VOCABULARY_TREE_H

#include "vocabulary_tree/vocabulary_tree.h"

#include <unordered_map>
#include <Eigen/Sparse>

/*
 * grouped_vocabulary_tree
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
    using result_type = std::tuple<int, int, double>;

public: // protected:

    // maps from global index to group index, so this is completely superfluous
    // DEPRECATED:
    std::unordered_map<int, int> index_group; // why isn't this a vector????

    // maps from global index to (group index, index within group)
    std::unordered_map<int, std::pair<int, int> > group_subgroup;
    size_t nbr_points;
    size_t nbr_groups;

protected:

    // TODO: check if any of these are necessary, clean up this mess of a class!
    std::vector<std::unordered_map<int, int> > leaf_vocabulary_vectors; // are these really necessary?
    std::vector<std::unordered_map<int, int> > intermediate_node_vocabulary_vectors; // or these
    std::map<node*, leaf_range> leaf_ranges; // or these?
    unordered_map<int, int> min_group_index; // or these?

    std::string save_state_path;

    map<node*, int> mapping; // for mapping to unique node IDs that can be used in the next run, might be empty

protected:

    int max_index;
    double calculate_similarity(int i, int j);
    //void compute_node_vocabulary_vector(Eigen::SparseVector<int> &node_vector, node* n);
    void compute_node_vocabulary_vector(unordered_map<int, double>& node_vector, node* n);
    leaf_range compute_vector_for_nodes_at_depth(node* n, int depth, int current_depth);
    void group_normalizing_constants_for_node(std::map<int, int>& normalizing_constants, node* n, int current_depth);

    void recursive_create_vocabulary_vector_from_ind(std::map<node*, double>& vvector, int i, node* n, int current_depth);

    void cache_group_adjacencies(int start_ind, std::vector<std::set<std::pair<int, int> > >& adjacencies);
    void cache_vocabulary_vectors(int start_ind, CloudPtrT& cloud);
    void save_cached_vocabulary_vectors_for_group(std::vector<vocabulary_vector> &vectors, int i);
    void load_cached_vocabulary_vectors_for_group(std::vector<vocabulary_vector> &vectors, std::set<std::pair<int, int> > &adjacencies, int i);

public:

    void query_vocabulary(std::vector<result_type>& results, CloudPtrT& query_cloud, size_t nbr_query);

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
    void append_cloud(CloudPtrT& extra_cloud, vector<pair<int, int> >& indices, std::vector<std::set<std::pair<int, int> > >& adjacencies, bool store_points = true);
    void add_points_from_input_cloud(bool save_cloud = true);
    void add_points_from_input_cloud(std::vector<std::set<std::pair<int, int> > >& adjacencies, bool save_cloud = true);
    void top_grouped_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results = 20);
    void top_combined_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results);
    void top_optimized_similarities(std::vector<std::tuple<int, int, double> >& scores, CloudPtrT& query_cloud, size_t nbr_results);
    void top_l2_similarities(std::vector<std::tuple<int, int, double> >& scores, CloudPtrT& query_cloud, size_t nbr_results);
    //void compute_node_difference(std::map<int, int>& sub_freqs, std::map<int, double>& map_scores,
    //                             node* n, std::map<node*, double>& qvec, int current_depth);

    template <class Archive> void save(Archive& archive) const;
    template <class Archive> void load(Archive& archive);

    void save_group_associations(const string& group_file);
    void load_group_associations(const string& group_file);

    grouped_vocabulary_tree() : super(), nbr_points(0), nbr_groups(0) {}
    grouped_vocabulary_tree(const std::string& save_state_path) : super(), nbr_points(0), nbr_groups(0), save_state_path(save_state_path) {}
};

#include "grouped_vocabulary_tree.hpp"

#endif // GROUPED_VOCABULARY_TREE_H
