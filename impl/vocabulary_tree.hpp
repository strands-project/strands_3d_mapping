#include "vocabulary_tree/vocabulary_tree.h"
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>

#include <Eigen/Core>

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
            return std::isnan(f);
        }) == p.data()+super::rows) {
            temp_cloud->push_back(new_cloud->at(i));
            indices.push_back(new_indices[i]);
        }
        /*if (std::find_if(std::begin(p.histogram), std::end(p.histogram), [] (float f) {
            return std::isnan(f);
        }) == std::end(p.histogram)) {
            temp_cloud->push_back(p);
            indices.push_back(new_indices[i]);
        }*/
    }
    super::set_input_cloud(temp_cloud);

    //indices = new_indices;
    // add sorting of indices here if necessary, in this case they should already be sorted
    //super::set_input_cloud(new_cloud);
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
            return std::isnan(f);
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
            if (l->data->source_id_freqs.count(source_id) == 1) {
                l->data->source_id_freqs.at(source_id) += 1;
            }
            else {
                l->data->source_id_freqs.insert(std::make_pair(source_id, int(1)));
            }
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
void vocabulary_tree<Point, K>::add_points_from_input_cloud()
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

    n->weight = log(N / double(normalizing_constants.size()));
    for (std::pair<const int, int>& v : normalizing_constants) {
        db_vector_normalizing_constants.at(v.first) += pexp(n->weight*v.second);
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

    /*for (PointT p : query_cloud->points) {
        std::cout << eig(p).transpose() << std::endl;
    }*/

    //std::cout << "Number of features: " << query_cloud->size() << std::endl;
    //std::cout << "Number of intersecting nodes: " << query_id_freqs.size() << std::endl;
    /*for (std::pair<node* const, double>& v : query_id_freqs) {
        if (v.first->is_leaf) {
            std::cout << "leaf: ";
        }
        std::cout << "with leaves: " << (v.first->range.second-v.first->range.first) << ", occurrences: " << v.second << " " << std::endl;
    }*/

    //int nodes_not_present = 0;
    int counter = 0;
    for (std::pair<node* const, double>& v : query_id_freqs) {
        double qi = v.second/qkr;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);
        /*if (source_id_freqs.count(34) == 0) {
            std::cout << "Not present in: " << counter << " with " << source_id_freqs.size() << " leaves" << std::endl;
            ++nodes_not_present;
        }*/
        for (std::pair<const int, int>& u : source_id_freqs) {
            if (normalized) {
                pk = db_vector_normalizing_constants.at(u.first); // 1.0f for not normalized
                pkr = proot(pk);
            }
            double pi = v.first->weight*double(u.second)/pkr;
            double residual = pexp(qi-pi)-pexp(pi)-pexp(qi);
            if (map_scores.count(u.first) == 1) {
                map_scores.at(u.first) += residual;
            }
            else {
                double dbnorm = db_vector_normalizing_constants.at(u.first);
                map_scores.insert(std::make_pair(u.first, dbnorm/pk+qnorm/qk+residual));
            }
        }
        ++counter;
    }

    //std::cout << "34 not present in: " << nodes_not_present << " groups" << std::endl;

    // this could probably be optimized a bit also, quite big copy operattion
    scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second < s2.second;
    });

    scores.resize(nbr_results);

    //std::cout << "q norm: " << qnorm << std::endl;
    //std::cout << "p norm: " << db_vector_normalizing_constants.at(scores[0].first) << std::endl;
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::pyramid_match_score_for_node(std::map<int, double>& scores, std::map<int, int>& source_id_freqs, node* n, const PointT& p)
{
    if (!n->is_leaf) {
        node* c = super::get_next_node(n, p);
        std::map<int, int> child_id_freqs;
        source_freqs_for_node(child_id_freqs, c);
        for (std::pair<const int, int>& u : child_id_freqs) {
            source_id_freqs.at(u.first) -= u.second; // might result in 0 but not less
        }
        pyramid_match_score_for_node(scores, child_id_freqs, c, p);
    }
    for (pair<const int, int>& u : source_id_freqs) {
        if (scores.count(u.first) == 1) {
            scores.at(u.first) += n->weight*double(u.second);
        }
        else {
            scores.insert(std::make_pair(u.first, n->weight*double(u.second)));
        }
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
        pyramid_match_score_for_node(map_scores, source_id_freqs, &(super::root), p);
    }
    int query_size = query_cloud->size();
    if (classic_pyramid_match) {
        for (pair<const int, double>& s : map_scores) {
            s.second *= 1.0/double(std::min(source_id_freqs.at(s.first), query_size));
        }
    }

    scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second < s2.second;
    });
    scores.resize(nbr_results);

    for (pair<node* const, double>& w : original_weights) {
        w.first->weight = w.second;
    }
}

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::compute_query_vector(std::map<node*, double> &query_id_freqs, CloudPtrT& query_cloud)
{
    for (PointT p : query_cloud->points) {
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
void vocabulary_tree<Point, K>::pyramid_match_weights_for_node(map<node *, double>& original_weights, node* n, size_t current_depth)
{
    if (classic_pyramid_match) {
        original_weights.insert(make_pair(n, n->weight));
        n->weight = pow(1.0/double(super::dim), double(super::depth-current_depth));
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

    if (!n->is_leaf) {
        for (node* c : n->children) {
            pyramid_match_weights_for_node(original_weights, c, current_depth+1);
        }
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compute_pyramid_match_weights(map<node*, double>& original_weights)
{
    // there should really be a node iterator for these things
    // remember to call set_input_cloud if loaded from file
    pyramid_match_weights_for_node(original_weights, &(super::root), 0);

    double accum = 0.0;
    for (leaf* l : super::leaves) {
        accum += l->weight;
    }
    accum /= double(super::leaves.size());
    std::cout << "Mean leaf weight: " << accum << std::endl;
    //exit(0);
}
