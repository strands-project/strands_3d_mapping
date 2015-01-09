#include "vocabulary_tree/vocabulary_tree.h"

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::set_input_cloud(CloudPtrT& new_cloud, std::vector<int>& new_indices)
{
    indices = new_indices;
    // add sorting of indices here if necessary, in this case they should already be sorted
    super::set_input_cloud(new_cloud);
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::add_points_from_input_cloud()
{
    // speed this up if possible!
    std::vector<int> temp = indices;
    std::unique(temp.begin(), temp.end());
    N = temp.size();
    for (int i : temp) {
        db_vector_normalizing_constants.insert(std::make_pair(i, 0));
    }

    super::add_points_from_input_cloud();

    for (leaf* l : super::leaves) {
        l->data = new inverted_file;
        for (int ind : l->inds) {
            int source_id = indices[ind];
            if (l->data->source_id_freqs.count(source_id) == 1) {
                l->data->source_id_freqs.at(source_id) += 1;
            }
            else {
                l->data->source_id_freqs.insert(std::make_pair(source_id, int(1)));
            }
        }
    }

    compute_normalizing_constants();
    /*distance_transform.resize(super::leaves.size()+1);
    distance_transform[0] = 0;
    for (size_t i = 0; i < super::leaves.size(); ++i) {
        distance_transform[i+1] = distance_transform[i] + super::leaves[i]->inds.size();
    }*/
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::source_freqs_for_node(std::map<int, int>& source_id_freqs, node* n) const
{
    for (int i = n->range.first; i < n->range.second; ++i) {
        for (std::pair<const int, int>& v : super::leaves[i]->data->source_id_freqs) {
            // could probably replace all of this by source_id_freqs[v.first] += v.second;
            if (source_id_freqs.count(v.first) == 1) {
                source_id_freqs.at(v.first) += v.second;
            }
            else {
                source_id_freqs.insert(v);
            }
        }
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::normalizing_constants_for_node(std::map<int, int>& normalizing_constants, node* n)
{
    auto pnorm = [](const float v) { return fabs(v); };

    if (n->is_leaf) {
        int leaf_ind = n->range.first;
        normalizing_constants = super::leaves[leaf_ind]->data->source_id_freqs;
    }
    else {
        for (node* c : n->children) {
            // here we need one set of normalizing constants for every child to not mess up scores between subtrees
            std::map<int, int> child_normalizing_constants;
            normalizing_constants_for_node(child_normalizing_constants, c);
            for (std::pair<const int, int>& v : child_normalizing_constants) {
                if (normalizing_constants.count(v.first) == 1) {
                    normalizing_constants.at(v.first) += v.second;
                }
                else {
                    normalizing_constants.insert(v);
                }
            }
        }
    }

    n->weight = float(normalizing_constants.size()) / N;
    for (std::pair<const int, int>& v : normalizing_constants) {
        //v.second.second += pnorm(n->weight*v.second.first);
        db_vector_normalizing_constants.at(v.first) += pnorm(n->weight*v.second);
    }
}

// do it one image at a time
// need to integrate the weights somehow... could have the accumulated weights in each leaf?
template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compute_normalizing_constants()
{
    std::map<int, int> normalizing_constants;
    normalizing_constants_for_node(normalizing_constants, &(super::root));
}

template <typename Point, size_t K>
size_t vocabulary_tree<Point, K>::nbr_sources_for_freqs(std::map<int, int>& source_id_freqs) const
{
    return source_id_freqs.size();
}

/*template <typename Point, size_t K>
size_t vocabulary_tree<Point, K>::points_in_node(node* n)
{
    if (n == NULL) {
        return 0;
    }
    return (distance_transform[n->range.second] - distance_transform[n->range.first]);
}*/

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::top_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    auto pnorm = [](const float v) { return fabs(v); };

    std::map<node*, float> query_id_freqs;
    float qnorm = compute_query_vector(query_id_freqs, query_cloud);
    std::map<int, float> map_scores;

    for (std::pair<node* const, float>& v : query_id_freqs) {
        float qi = v.second;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);
        for (std::pair<const int, int>& u : source_id_freqs) {
            float pi = v.first->weight*float(u.second);
            float residual = pnorm(qi-pi)-pnorm(pi)-pnorm(qi);
            if (map_scores.count(u.first) == 1) {
                map_scores.at(u.first) += residual;
            }
            else {
                float dbnorm = db_vector_normalizing_constants.at(u.first);
                map_scores.insert(std::make_pair(u.first, dbnorm+qnorm+residual));
            }
        }
    }

    // this could probably be optimized a bit also, quite big copy operattion
    scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second < s2.second;
    });

    scores.resize(nbr_results);
}

template <typename Point, size_t K>
float vocabulary_tree<Point, K>::compute_query_vector(std::map<node*, float>& query_id_freqs, CloudPtrT& query_cloud)
{
    auto pnorm = [](const float v) { return fabs(v); };

    for (PointT& p : query_cloud->points) {
        std::vector<node*> path;
        super::get_path_for_point(path, p);
        for (node* n : path) {
            if (query_id_freqs.count(n) == 1) {
                query_id_freqs.at(n) += 1.0f;
            }
            else {
                query_id_freqs.insert(std::make_pair(n, 1.0f));
            }
        }
    }
    float qnorm = 0.0f;
    for (std::pair<node* const, float>& v : query_id_freqs) {
        v.second = v.first->weight*v.second;
        qnorm += pnorm(v.second);
    }

    return qnorm;
}
