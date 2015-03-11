#ifndef REWEIGHTED_VOCABULARY_TREE
#define REWEIGHTED_VOCABULARY_TREE

#include "vocabulary_tree/vocabulary_tree.h"
#include <stddef.h>

template <typename Point, size_t K>
class reweighted_vocabulary_tree : public vocabulary_tree<Point, K> {
protected:

    using super = vocabulary_tree<Point, K>;
    using typename super::PointT;
    using typename super::CloudT;
    using typename super::CloudPtrT;

public:

    using typename super::cloud_idx_score;
    using typename super::leaf;
    using typename super::node;

protected:

    void compute_new_weights(std::map<int, double> &original_norm_constants, std::map<node*, double>& original_weights,
                             std::map<int, double>& weighted_indices, CloudPtrT& query_cloud);
    void adjust_learned_weights(map<int, double>& original_norm_constants, map<node*, double>& original_weights,
                                map<int, double>& weighted_indices, CloudPtrT& query_cloud);

public:
    void top_similarities_reweighted(std::vector<cloud_idx_score>& scores, std::map<int, double>& weights,
                                     CloudPtrT& query_cloud, size_t nbr_results);
    reweighted_vocabulary_tree() : super() {}
};

#include "reweighted_vocabulary_tree.hpp"

#endif // REWEIGHTED_VOCABULARY_TREE
