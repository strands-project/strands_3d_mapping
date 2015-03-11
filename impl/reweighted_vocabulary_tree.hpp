#include "reweighted_vocabulary_tree/reweighted_vocabulary_tree.h"

using namespace std;

template <typename Key, typename Value1, typename Value2>
set<Key> key_intersection(map<Key, Value1> const& lhs, map<Key, Value2> const& rhs)
{
    typedef typename map<Key, Value1>::const_iterator input_iterator1;
    typedef typename map<Key, Value2>::const_iterator input_iterator2;

    set<Key> result;
    input_iterator1 it1 = lhs.begin();
    input_iterator2 it2 = rhs.begin();
    input_iterator1 end1 = lhs.end();
    input_iterator2 end2 = rhs.end();
    while (it1 != end1 && it2 != end2) {
        if (it1->first == it2->first) {
            result.insert(it1->first);
            ++it1;
            ++it2;
        }
        else {
            if (it1->first < it2->first) {
                ++it1;
            }
            else {
                ++it2;
            }
        }
    }
    return result;
}

template <typename Point, size_t K>
void reweighted_vocabulary_tree<Point, K>::compute_new_weights(map<int, double>& original_norm_constants,
                                                               map<node*, double>& original_weights,
                                                               map<int, double>& weighted_indices,
                                                               CloudPtrT& query_cloud)
{
    map<node*, pair<size_t, double> > new_weights;

    for (PointT p : query_cloud->points) {
        std::vector<node*> path;
        super::get_path_for_point(path, p);
        for (node* n : path) {
            // if root node, skip since it contributes the same to every
            if (n == &(super::root)) {
                continue;
            }
            // if no intersection with weighted_indices, continue
            map<int, int> source_inds;
            super::source_freqs_for_node(source_inds, n);
            set<int> intersection = key_intersection(source_inds, weighted_indices);
            if (intersection.empty()) {
                continue;
            }

            if (new_weights.count(n) == 0) {
                new_weights.insert(make_pair(n, make_pair(0, 0.0)));
            }
            for (int i : intersection) {
                pair<size_t, double>& ref = new_weights.at(n);
                ref.first += 1;
                ref.second += weighted_indices.at(i);
            }
        }
    }

    for (pair<node* const, pair<size_t, double> >& v : new_weights) {
        // compute and store the new weights
        double original_weight = v.first->weight;
        double new_weight = (v.second.second / double(v.second.first)) * original_weight;
        original_weights.insert(make_pair(v.first, original_weight));
        v.first->weight = new_weight;

        // update the normalization computations
        map<int, int> source_inds;
        super::source_freqs_for_node(source_inds, v.first);
        for (pair<const int, int>& u : source_inds) {
            // first, save the original normalization if it isn't already
            if (original_norm_constants.count(u.first) == 0) {
                original_norm_constants.insert(make_pair(u.first, super::db_vector_normalizing_constants.at(u.first)));
            }
            super::db_vector_normalizing_constants.at(u.first) -=
                    super::pexp(original_weight*u.second) - super::pexp(new_weight*u.second);
        }
    }
}

template <typename Point, size_t K>
void reweighted_vocabulary_tree<Point, K>::top_similarities_reweighted(vector<cloud_idx_score>& scores, map<int, double>& weights,
                                                                       CloudPtrT& query_cloud, size_t nbr_results)
{
    // need a data structure to store all of the original weights that we change
    map<node*, double> original_weights;
    map<int, double> original_norm_constants;
    compute_new_weights(original_norm_constants, original_weights, weights, query_cloud);
    //adjust_learned_weights(original_norm_constants, original_weights, weights, query_cloud);
    //super::top_similarities(scores, query_cloud, nbr_results);
    super::top_partial_similarities(scores, query_cloud, nbr_results);
    for (pair<node* const, double> v : original_weights) {
        // restore the node weights
        v.first->weight = v.second;
    }

    for (pair<const int, double>& v : original_norm_constants) {
        // restore the normalization constants
        super::db_vector_normalizing_constants.at(v.first) = v.second;
    }
}

// here, values in original weights are assumed to be in range [-1, 1]
template <typename Point, size_t K>
void reweighted_vocabulary_tree<Point, K>::adjust_learned_weights(map<int, double>& original_norm_constants,
                                                                  map<node*, double>& original_weights,
                                                                  map<int, double>& weighted_indices,
                                                                  CloudPtrT& query_cloud)
{
    map<node*, double> query_vector;
    super::compute_query_vector(query_vector, query_cloud);
    double eps = 0.01;

    map<node*, double> new_weights;

    double sdet = 1.0;
    for (pair<node* const, double>& v : query_vector) {
        // exchange this for matching_min_depth later
        if (v.first == &(super::root)) {
            continue;
        }
        map<int, int> source_id_freqs;
        super::source_freqs_for_node(source_id_freqs, v.first);
        double sii = 0.0;
        for (pair<const int, double>& s : weighted_indices) {
            if (source_id_freqs.count(s.first) == 0) {
                continue;
            }
            double pi = double(source_id_freqs[s.first])*v.first->weight;
            double qi = v.second;
            double dist = super::pexp(pi-qi)-super::pexp(pi)-super::pexp(qi);
            sii += s.second*dist; // sqrt(dist)*sqrt(dist), S smaller if close match
        }
        if (1.0/v.first->weight + sii < eps) {
            sii = eps - 1.0/v.first->weight;
        }
        new_weights[v.first] = sii;
        sdet *= sii;
    }

    sdet = pow(sdet, 1.0/double(query_vector.size())); // this could also be the number of nodes but feels unnatural

    for (pair<node* const, double>& v : new_weights) {
        double original_weight = v.first->weight;
        original_weights[v.first] = original_weight;
        v.first->weight = 1.0/(1.0/v.first->weight + sdet*v.second);

        map<int, int> source_id_freqs;
        super::source_freqs_for_node(source_id_freqs, v.first);
        for (pair<const int, int>& s : source_id_freqs) {
            double pn = double(source_id_freqs[s.first]);
            if (original_norm_constants.count(s.first)) {
                original_norm_constants[s.first] = super::db_vector_normalizing_constants[s.first];
            }
            super::db_vector_normalizing_constants[s.first] -= super::pexp(original_weight*pn) - super::pexp(v.first->weight*pn);
        }
    }
}
