#ifndef GROUPED_VOCABULARY_TREE_H
#define GROUPED_VOCABULARY_TREE_H

#include "vocabulary_tree/vocabulary_tree.h"
#include <unordered_map>

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

public:

    using cloud_idx_score = std::pair<int, double>;

protected:

    std::unordered_map<int, int> index_group;
    size_t nbr_points;
    size_t nbr_groups;

protected:

    double calculate_similarity(int i, int j);

public:

    void set_input_cloud(CloudPtrT& new_cloud, std::vector<pair<int, int> >& indices);
    void append_cloud(CloudPtrT& extra_cloud, vector<pair<int, int> >& indices, bool store_points = true);
    void add_points_from_input_cloud(bool save_cloud = true);
    void top_grouped_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results = 20);

    template <class Archive> void save(Archive& archive) const;
    template <class Archive> void load(Archive& archive);

    grouped_vocabulary_tree() : super(), nbr_points(0), nbr_groups(0) {}
};

#include "grouped_vocabulary_tree.hpp"

#endif // GROUPED_VOCABULARY_TREE_H
