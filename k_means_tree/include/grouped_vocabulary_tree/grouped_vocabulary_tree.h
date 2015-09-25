#ifndef GROUPED_VOCABULARY_TREE_H
#define GROUPED_VOCABULARY_TREE_H

#include "vocabulary_tree/vocabulary_tree.h"

#include <unordered_map>

/*
 * grouped_vocabulary_tree
 *
 * This class describes a collection of points that occurr together
 *
 */

// we need these two initial indices to denote the indices
// of the group and the index within the group
struct grouped_result : public vocabulary_result {
    int group_index;
    int subgroup_index;
    grouped_result(float score, int group_index, int subgroup_index) : vocabulary_result(0, score), group_index(group_index), subgroup_index(subgroup_index) {}
    grouped_result() : vocabulary_result() {}
};

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

    using result_type = grouped_result; //std::tuple<int, int, double>;
    using group_type = vector<int>;

public: // protected:

    // maps from global index to (group index, index within group)
    std::unordered_map<int, std::pair<int, int> > group_subgroup;
    size_t nbr_points;
    size_t nbr_groups;

protected:

    // TODO: check if any of these are necessary, clean up this mess of a class!
    std::string save_state_path;
    map<node*, int> mapping; // for mapping to unique node IDs that can be used in the next run, might be empty

protected:

    // for caching the vocabulary vectors
    void cache_group_adjacencies(int start_ind, std::vector<std::set<std::pair<int, int> > >& adjacencies);
    void cache_vocabulary_vectors(int start_ind, CloudPtrT& cloud);
    void save_cached_vocabulary_vectors_for_group(std::vector<vocabulary_vector> &vectors, int i);
    void load_cached_vocabulary_vectors_for_group(std::vector<vocabulary_vector> &vectors, std::set<std::pair<int, int> > &adjacencies, int i);

public:

    void query_vocabulary(std::vector<result_type>& results, std::vector<group_type>& groups, CloudPtrT& query_cloud, size_t nbr_query);

    void get_subgroups_for_group(std::set<int>& subgroups, int group_id);
    int get_id_for_group_subgroup(int group_id, int subgroup_id);

    void set_input_cloud(CloudPtrT& new_cloud, std::vector<pair<int, int> >& indices);
    void append_cloud(CloudPtrT& extra_cloud, vector<pair<int, int> >& indices, bool store_points = true);
    void append_cloud(CloudPtrT& extra_cloud, vector<pair<int, int> >& indices, std::vector<std::set<std::pair<int, int> > >& adjacencies, bool store_points = true);
    void add_points_from_input_cloud(bool save_cloud = true);
    void add_points_from_input_cloud(std::vector<std::set<std::pair<int, int> > >& adjacencies, bool save_cloud = true);
    void top_combined_similarities(std::vector<result_type>& scores, CloudPtrT& query_cloud, size_t nbr_results);

    template <class Archive> void save(Archive& archive) const;
    template <class Archive> void load(Archive& archive);

    grouped_vocabulary_tree() : super(), nbr_points(0), nbr_groups(0) {}
    grouped_vocabulary_tree(const std::string& save_state_path) : super(), nbr_points(0), nbr_groups(0), save_state_path(save_state_path) {}
};

#include "grouped_vocabulary_tree.hpp"

#endif // GROUPED_VOCABULARY_TREE_H
