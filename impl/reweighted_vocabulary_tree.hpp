#include "reweighted_vocabulary_tree/reweighted_vocabulary_tree.h"

using namespace std;

template <typename Point, size_t K>
void reweighted_vocabulary_tree<Point, K>::top_similarities_reweighted(vector<cloud_idx_score>& scores, map<int, double>& weights,
                                                                       CloudPtrT& query_cloud, size_t nbr_results)
{
    // need a data structure to store all of the original weights that we change
    map<node*, double> original_weights;
    map<int, double> original_norm_constants;
    super::compute_new_weights(original_norm_constants, original_weights, weights, query_cloud);
    //adjust_learned_weights(original_norm_constants, original_weights, weights, query_cloud);
    //super::top_similarities(scores, query_cloud, nbr_results);
    super::top_similarities(scores, query_cloud, nbr_results);
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
