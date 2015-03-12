#include "vocabulary_tree/vocabulary_tree.h"
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>

#include <Eigen/Core>

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::set_min_match_depth(int depth)
{
    matching_min_depth = depth;
}

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::pexp(const double v) const
{
    return fabs(v);
}

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::proot(const double v) const
{
    return fabs(v);
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::set_input_cloud(CloudPtrT& new_cloud, std::vector<int>& new_indices)
{
    // some features will have nans in them, we need to remove those
    Eigen::Matrix<float, super::rows, 1> p;
    CloudPtrT temp_cloud(new CloudT);
    temp_cloud->reserve(new_cloud->size());
    indices.reserve(new_indices.size());
    for (size_t i = 0; i < new_cloud->size(); ++i) {
        p = eig(new_cloud->at(i));
        if (std::find_if(p.data(), p.data()+super::rows, [] (float f) {
            return std::isnan(f) || std::isinf(f);
        }) == p.data()+super::rows) {
            temp_cloud->push_back(new_cloud->at(i));
            indices.push_back(new_indices[i]);
        }
    }
    super::set_input_cloud(temp_cloud);

    //indices = new_indices;
    // add sorting of indices here if necessary, in this case they should already be sorted
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::append_cloud(CloudPtrT& extra_cloud, vector<int>& extra_indices, bool store_points)
{
    Eigen::Matrix<float, super::rows, 1> p;
    CloudPtrT temp_cloud(new CloudT);
    temp_cloud->reserve(extra_cloud->size());
    indices.reserve(indices.size()+extra_indices.size());
    for (size_t i = 0; i < extra_cloud->size(); ++i) {
        p = eig(extra_cloud->at(i));
        if (std::find_if(p.data(), p.data()+super::rows, [] (float f) {
            return std::isnan(f) || std::isinf(f);
        }) == p.data()+super::rows) {
            temp_cloud->push_back(extra_cloud->at(i));
            indices.push_back(extra_indices[i]);
        }
    }
    super::append_cloud(temp_cloud, store_points);

    for (leaf* l : super::leaves) {
        l->data->source_id_freqs.clear();
        for (int ind : l->inds) {
            int source_id = indices[ind];
            l->data->source_id_freqs[source_id] += 1;
        }
    }

    // maybe put this code directly in compute_normalizing_constants
    db_vector_normalizing_constants.clear();
    std::vector<int> temp = indices;
    std::unique(temp.begin(), temp.end());
    N = temp.size();
    for (int i : temp) {
        db_vector_normalizing_constants.insert(std::make_pair(i, 0));
    }

    // we really only need to compute them for the new indices
    compute_normalizing_constants();
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::add_points_from_input_cloud(bool save_cloud)
{
    // speed this up if possible!
    std::vector<int> temp = indices;
    std::unique(temp.begin(), temp.end());
    N = temp.size();
    for (int i : temp) {
        db_vector_normalizing_constants.insert(std::make_pair(i, 0));
    }

    // DEBUG: Just a fix for sift features leading to overflow
    super::add_points_from_input_cloud();
    /*for (PointT& p : super::cloud->points) {
        eig(p).normalize();
    }*/

    for (leaf* l : super::leaves) {
        l->data = new inverted_file;
        for (int ind : l->inds) {
            int source_id = indices[ind];
            l->data->source_id_freqs[source_id] += 1;
        }
    }

    compute_normalizing_constants();
    /*distance_transform.resize(super::leaves.size()+1);
    distance_transform[0] = 0;
    for (size_t i = 0; i < super::leaves.size(); ++i) {
        distance_transform[i+1] = distance_transform[i] + super::leaves[i]->inds.size();
    }*/

    if (!save_cloud) {
        super::cloud->clear();
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::source_freqs_for_node(std::map<int, int>& source_id_freqs, node* n) const
{
    for (int i = n->range.first; i < n->range.second; ++i) {
        for (std::pair<const int, int>& v : super::leaves[i]->data->source_id_freqs) {
            source_id_freqs[v.first] += v.second;
        }
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::normalizing_constants_for_node(std::map<int, int>& normalizing_constants, node* n, int current_depth)
{
    if (n->is_leaf) {
        int leaf_ind = n->range.first;
        normalizing_constants = super::leaves[leaf_ind]->data->source_id_freqs;
    }
    else {
        for (node* c : n->children) {
            // here we need one set of normalizing constants for every child to not mess up scores between subtrees
            std::map<int, int> child_normalizing_constants;
            normalizing_constants_for_node(child_normalizing_constants, c, current_depth+1);
            for (std::pair<const int, int>& v : child_normalizing_constants) {
                normalizing_constants[v.first] += v.second;
            }
        }
    }

    n->weight = log(N / double(normalizing_constants.size()));

    // this could be further up if we do not want to e.g. calculate weights for upper nodes
    if (current_depth < matching_min_depth) {
        return;
    }

    for (std::pair<const int, int>& v : normalizing_constants) {
        db_vector_normalizing_constants[v.first] += pexp(n->weight*v.second); // hope this is inserting 0
    }
}

// do it one image at a time
// need to integrate the weights somehow... could have the accumulated weights in each leaf?
template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compute_normalizing_constants()
{
    db_vector_normalizing_constants.clear(); // this should be safe...
    std::map<int, int> normalizing_constants;
    normalizing_constants_for_node(normalizing_constants, &(super::root), 0);
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
    std::map<node*, double> query_id_freqs;
    double qnorm = compute_query_vector(query_id_freqs, query_cloud);
    std::map<int, double> map_scores;

    double qk = 1.0f;
    double pk = 1.0f;
    double qkr = 1.0f;
    double pkr = 1.0f;

    if (normalized) {
        qk = qnorm;
        qkr = proot(qk);
    }

    for (std::pair<node* const, double>& v : query_id_freqs) {
        double qi = v.second/qkr;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);

        for (std::pair<const int, int>& u : source_id_freqs) {
            if (normalized) {
                pk = db_vector_normalizing_constants[u.first]; // 1.0f for not normalized
                pkr = proot(pk);
            }
            double pi = v.first->weight*double(u.second)/pkr;
            double residual = pexp(qi-pi)-pexp(pi)-pexp(qi); // = 2*(pexp(std::max(qi-pi, 0.0))-pexp(qi));
            if (map_scores.count(u.first) == 1) {
                map_scores.at(u.first) += residual;
            }
            else {
                double dbnorm = db_vector_normalizing_constants[u.first];
                map_scores.insert(std::make_pair(u.first, dbnorm/pk+qnorm/qk+residual));
            }
        }
    }

    // this could probably be optimized a bit also, quite big copy operattion
    scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second < s2.second; // find min elements!
    });

    scores.resize(nbr_results);
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compare_vocabulary_vectors(std::map<node*, double>& query_id_freqs)
{
    for (std::pair<node* const, double>& v : query_id_freqs) {
        double qi = v.second;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);

        for (std::pair<const int, int>& u : source_id_freqs) {
            double dbnorm = db_vector_normalizing_constants[u.first];
            double pi = v.first->weight*double(u.second);
            double residual = pexp(std::max(qi-pi, 0.0))-pexp(qi);

            cout << "(" << qi << ", " << pi << ", " << dbnorm << ", " << residual << ")" << endl;
        }
    }

    int dummy;
    cin >> dummy;
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::top_combined_similarities(vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    vector<cloud_idx_score> larger_scores;
    top_partial_similarities(larger_scores, query_cloud, 0);
    vector<cloud_idx_score> smaller_scores;
    test_partial_similarities(smaller_scores, query_cloud, 0);

    map<int, double> larger_map((larger_scores.begin()), larger_scores.end());
    map<int, double> smaller_map((smaller_scores.begin()), smaller_scores.end());

    for (pair<const int, double>& s : larger_map) {
        if (smaller_map.count(s.first)) {
            s.second = std::max(s.second, smaller_map[s.first]);
        }
        else {
            s.second = 1.0; // very large
        }
    }

    scores.insert(scores.end(), larger_map.begin(), larger_map.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second < s2.second; // find min elements!
    });
    scores.resize(nbr_results);
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::top_partial_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    std::map<node*, double> query_id_freqs;
    double qnorm = compute_query_vector(query_id_freqs, query_cloud);
    std::map<int, double> map_scores;

    //std::map<int, int> total_id_freqs;
    //source_freqs_for_node(total_id_freqs, &(super::root));
    //compare_vocabulary_vectors(query_id_freqs);

    for (std::pair<node* const, double>& v : query_id_freqs) {
        double qi = v.second;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);

        for (std::pair<const int, int>& u : source_id_freqs) {
            /*if (std::abs(total_id_freqs[u.first]-query_cloud->size()) > 10) {
                continue;
            }*/
            double dbnorm = db_vector_normalizing_constants[u.first];
            double pi = v.first->weight*double(u.second);
            double residual = pexp(std::max(qi-pi, 0.0))-pexp(qi); // ~ (|q_i-p_i|-|q_i|-|p_i|)+|q_i|
            double normalization = qnorm;//dbnorm;//std::min(float(total_id_freqs[u.first]), float(query_cloud->size()));
            if (map_scores.count(u.first) == 1) {
                map_scores.at(u.first) += residual/normalization;
            }
            else {
                map_scores.insert(std::make_pair(u.first, (qnorm+residual)/normalization));
            }
        }
    }

    // this could probably be optimized a bit also, quite big copy operattion
    scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second < s2.second; // find min elements!
    });

    if (nbr_results > 0) {
        scores.resize(nbr_results);
    }

    /*for (cloud_idx_score& s : scores) {
        s.second = proot(s.second);
    }*/
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::test_partial_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    std::map<node*, double> query_id_freqs;
    double qnorm = compute_query_vector(query_id_freqs, query_cloud);
    std::map<int, double> map_scores;
    //std::map<int, int> common_nodes; // DEBUG

    //std::map<int, int> total_id_freqs;
    //source_freqs_for_node(total_id_freqs, &(super::root));
    //compare_vocabulary_vectors(query_id_freqs);

    for (std::pair<node* const, double>& v : query_id_freqs) {
        double qi = v.second;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);

        for (std::pair<const int, int>& u : source_id_freqs) {
            /*if (std::abs(total_id_freqs[u.first]-query_cloud->size()) > 10) {
                continue;
            }*/
            //common_nodes[u.first] += 1; // DEBUG

            double dbnorm = db_vector_normalizing_constants[u.first];
            double pi = v.first->weight*double(u.second);
            double residual = pexp(std::max(pi-qi, 0.0))-pexp(pi); // ~ (|q_i-p_i|-|q_i|-|p_i|)+|q_i|
            double normalization = dbnorm;//dbnorm;//std::min(float(total_id_freqs[u.first]), float(query_cloud->size()));
            if (map_scores.count(u.first) != 0) {
                map_scores.at(u.first) += residual/normalization;
                //map_scores.at(u.first) += std::min(pi, qi);
            }
            else {
                map_scores.insert(std::make_pair(u.first, (dbnorm+residual)/normalization));
                //map_scores.insert(std::make_pair(u.first, std::min(pi, qi)));
            }
        }
    }

    /*for (pair<const int, double>& s : map_scores) { // DEBUG
        s.second /= float(common_nodes[s.first]);
    }*/

    // this could probably be optimized a bit also, quite big copy operattion
    scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second < s2.second; // find min elements!
    });

    if (nbr_results > 0) {
        scores.resize(nbr_results);
    }

    /*for (cloud_idx_score& s : scores) {
        s.second = proot(s.second);
    }*/
}

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::compute_query_vector(std::map<node*, double>& query_id_freqs, CloudPtrT& query_cloud)
{
    for (PointT p : query_cloud->points) {
        std::vector<node*> path;
        super::get_path_for_point(path, p);
        int current_depth = 0;
        for (node* n : path) {
            if (current_depth >= matching_min_depth) {
                query_id_freqs[n] += 1.0f;
            }
            ++current_depth;
        }
    }
    double qnorm = 0.0f;
    for (std::pair<node* const, double>& v : query_id_freqs) {
        v.second = v.first->weight*v.second;
        qnorm += pexp(v.second);
    }

    return qnorm;
}

template <typename Point, size_t K>
template <class Archive>
void vocabulary_tree<Point, K>::save(Archive& archive) const
{
    super::save(archive);
    archive(indices);
    archive(db_vector_normalizing_constants);
    archive(N);
}

template <typename Point, size_t K>
template <class Archive>
void vocabulary_tree<Point, K>::load(Archive& archive)
{
    super::load(archive);
    archive(indices);
    archive(db_vector_normalizing_constants);
    archive(N);
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::pyramid_match_weights_for_node(map<node*, double>& original_weights, node* n, size_t current_depth)
{
    if (current_depth >= matching_min_depth) {
        if (classic_pyramid_match) {
            original_weights.insert(make_pair(n, n->weight));
            //n->weight = pow(1.0/double(super::dim), double(super::depth-current_depth));
            std::map<int, int> source_id_freqs;
            source_freqs_for_node(source_id_freqs, n);
            n->weight = 1.0/double(source_id_freqs.size());
        }
        else {
            std::vector<int> cloud_indices;
            for (size_t i = n->range.first; i < n->range.second; ++i) {
                leaf* l = super::leaves[i];
                cloud_indices.insert(cloud_indices.end(), l->inds.begin(), l->inds.end());
            }

            std::vector<double> distances(cloud_indices.size());
            for (int ind : cloud_indices) {
                distances.push_back((eig(n->centroid)-eig(super::cloud->at(ind))).template lpNorm<1>());
            }

            double m = std::accumulate(distances.begin(), distances.end(), 0.0);
            m /= double(distances.size());

            double var = 0.0;
            for (double d : distances) {
                var += (d - m)*(d - m);
            }

            n->weight = std::min(1.0/sqrt(var), 200000.0); // need to check what is a good value here, check mean of leaves
        }
    }

    if (!n->is_leaf) {
        for (node* c : n->children) {
            pyramid_match_weights_for_node(original_weights, c, current_depth+1);
        }
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compute_pyramid_match_weights(map<node*, double>& original_weights)
{
    pyramid_match_weights_for_node(original_weights, &(super::root), 0);
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::pyramid_match_score_for_node(std::map<int, double>& scores, std::map<int, int>& source_id_freqs,
                                                             node* n, const PointT& p, int current_depth)
{
    if (!n->is_leaf) {
        node* c = super::get_next_node(n, p);
        std::map<int, int> child_id_freqs;
        source_freqs_for_node(child_id_freqs, c);
        for (std::pair<const int, int>& u : child_id_freqs) {
            source_id_freqs.at(u.first) -= u.second; // might result in 0 but not less
        }
        pyramid_match_score_for_node(scores, child_id_freqs, c, p, current_depth+1);
    }

    if (current_depth < matching_min_depth) {
        return;
    }

    for (pair<const int, int>& u : source_id_freqs) {
        // but we have to compare this with the value of q also?!
        scores[u.first] += n->weight*double(u.second);
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::top_pyramid_match_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    // in the end, this should be just a toggle, use one distance measure or the other
    map<node*, double> original_weights; // this could be a vector of pairs instead
    compute_pyramid_match_weights(original_weights);

    std::map<int, double> map_scores;
    std::map<int, int> source_id_freqs;
    source_freqs_for_node(source_id_freqs, &(super::root));
    for (PointT& p : query_cloud->points) {
        pyramid_match_score_for_node(map_scores, source_id_freqs, &(super::root), p, 0);
    }
    /*int query_size = query_cloud->size();
    if (classic_pyramid_match) {
        for (pair<const int, double>& s : map_scores) {
            //s.second *= 1.0/double(std::min(source_id_freqs.at(s.first), query_size));
        }
    }*/

    scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second > s2.second; // find max elements!
    });
    scores.resize(nbr_results);

    for (pair<node* const, double>& w : original_weights) {
        w.first->weight = w.second;
    }
}
