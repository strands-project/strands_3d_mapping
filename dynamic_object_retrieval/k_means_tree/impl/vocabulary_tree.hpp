#include "vocabulary_tree/vocabulary_tree.h"

#include <Eigen/Core>

#include <chrono> // DEBUG

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::query_vocabulary(std::vector<result_type>& results, CloudPtrT& query_cloud, size_t nbr_results)
{
    top_combined_similarities(results, query_cloud, nbr_results);
    //debug_similarities(results, query_cloud, nbr_results);
}

template <typename Key, typename Value1, typename Value2>
vector<Value2> key_intersection(const map<Key, Value1>& lhs, const vector<pair<Key, Value2> >& rhs)
{
    typedef typename map<Key, Value1>::const_iterator input_iterator1;
    typedef typename vector<pair<Key, Value2> >::const_iterator input_iterator2;

    vector<Value2> result;
    input_iterator1 it1 = lhs.cbegin();
    input_iterator2 it2 = rhs.cbegin();
    input_iterator1 end1 = lhs.cend();
    input_iterator2 end2 = rhs.cend();
    while (it1 != end1 && it2 != end2) {
        if (it1->first == it2->first) {
            result.push_back(it2->second);
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

template <typename Key, typename Value>
bool has_key_intersection(const set<Key>& lhs, const map<Key, Value>& rhs)
{
    typedef typename set<Key>::const_iterator input_iterator1;
    typedef typename map<Key, Value>::const_iterator input_iterator2;

    input_iterator1 it1 = lhs.cbegin();
    input_iterator2 it2 = rhs.cbegin();
    input_iterator1 end1 = lhs.cend();
    input_iterator2 end2 = rhs.cend();
    while (it1 != end1 && it2 != end2) {
        if (*it1 == it2->first) {
            return true;
        }
        if (*it1 < it2->first) {
            ++it1;
        }
        else {
            ++it2;
        }
    }
    return false;
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compute_new_weights(map<int, double>& original_norm_constants,
                                                    map<node*, double>& original_weights,
                                                    vector<pair<int, double> >& weighted_indices,
                                                    CloudPtrT& query_cloud)
{
    map<node*, pair<size_t, double> > new_weights;

    set<node*> already_visited;

    std::sort(weighted_indices.begin(), weighted_indices.end(), [](const pair<int, double>& p1, const pair<int, double>& p2) {
        return p1.first < p2.first;
    });

    for (PointT p : query_cloud->points) {
        std::vector<node*> path;
        super::get_path_for_point(path, p);
        int current_depth = 0;
        for (node* n : path) {
            // if root node, skip since it contributes the same to every
            if (current_depth < matching_min_depth || already_visited.count(n) > 0) {
                ++current_depth;
                continue;
            }
            already_visited.insert(n);

            // if no intersection with weighted_indices, continue
            map<int, int> source_inds;
            source_freqs_for_node(source_inds, n);

            vector<double> intersection = key_intersection(source_inds, weighted_indices);
            if (intersection.empty()) {
                ++current_depth;
                continue;
            }

            if (new_weights.count(n) == 0) {
                new_weights.insert(make_pair(n, make_pair(0, 0.0)));
            }

            /*
            for (int i : intersection) {
                pair<size_t, double>& ref = new_weights.at(n);
                ref.first += 1;
                ref.second += weighted_indices.at(i);
            }
            */
            for (double w : intersection) {
                pair<size_t, double>& ref = new_weights.at(n);
                ref.first += 1;
                ref.second += w;
            }

            ++current_depth;
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
        source_freqs_for_node(source_inds, v.first);
        for (pair<const int, int>& u : source_inds) {
            // first, save the original normalization if it isn't already
            if (original_norm_constants.count(u.first) == 0) {
                original_norm_constants.insert(make_pair(u.first, db_vector_normalizing_constants.at(u.first)));
            }
            db_vector_normalizing_constants.at(u.first) -=
                    pexp(original_weight*u.second) - pexp(new_weight*u.second);
        }
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compute_new_weights(map<int, double>& original_norm_constants,
                                                    map<node*, double>& original_weights,
                                                    vector<pair<set<int>, double> >& weighted_indices,
                                                    CloudPtrT& query_cloud)
{
    map<node*, pair<size_t, double> > new_weights;

    set<node*> already_visited;

    // this is not how it is supposed to work I think? shouldn't it just look at
    // one node once, not multiple as it appears below?
    // I think so, we should definitely changes it as it multiplies the
    // significance of places where multiple features intersect
    for (PointT p : query_cloud->points) {
        std::vector<node*> path;
        super::get_path_for_point(path, p);
        int current_depth = 0;
        for (node* n : path) {
            // if root node, skip since it contributes the same to every
            if (current_depth < matching_min_depth || already_visited.count(n) > 0) {
                ++current_depth;
                continue;
            }
            already_visited.insert(n);

            // if no intersection with weighted_indices, continue
            map<int, int> source_inds;
            source_freqs_for_node(source_inds, n); // we're gonna do this for the children, wouldn't it be better to do recursive?

            for (const pair<set<int>, double>& w : weighted_indices) {
                bool has_intersection = has_key_intersection(w.first, source_inds);
                if (!has_intersection) {
                    continue;
                }

                if (new_weights.count(n) == 0) {
                    new_weights.insert(make_pair(n, make_pair(0, 0.0)));
                }

                pair<size_t, double>& ref = new_weights.at(n);
                ref.first += 1;
                ref.second += w.second;
            }

            ++current_depth;
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
        source_freqs_for_node(source_inds, v.first);
        for (pair<const int, int>& u : source_inds) {
            // first, save the original normalization if it isn't already
            if (original_norm_constants.count(u.first) == 0) {
                original_norm_constants.insert(make_pair(u.first, db_vector_normalizing_constants.at(u.first)));
            }
            db_vector_normalizing_constants.at(u.first) -=
                    pexp(original_weight*u.second) - pexp(new_weight*u.second);
        }
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::restore_old_weights(map<int, double>& original_norm_constants,
                                                    map<node*, double>& original_weights)
{
    for (pair<node* const, double> v : original_weights) {
        // restore the node weights
        v.first->weight = v.second;
    }

    for (pair<const int, double>& v : original_norm_constants) {
        // restore the normalization constants
        db_vector_normalizing_constants.at(v.first) = v.second;
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::set_min_match_depth(int depth)
{
    matching_min_depth = depth;
}

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::pexp(const double v) const
{
    //return v*v;
    return fabs(v);
}

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::proot(const double v) const
{
    //return sqrt(v);
    return fabs(v);
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::unfold_nodes(vector<node*>& path, node* n, const PointT& p, map<node*, double>& active)
{
    if (n->is_leaf) {
        return;
    }
    node* closest = super::get_next_node(n, p);
    if (active.count(closest) == 0) {
        return;
    }
    path.push_back(closest);
    unfold_nodes(path, closest, p, active);
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::get_path_for_point(vector<node*>& path, const PointT& point, map<node*, double>& active)
{
    path.push_back(&(super::root));
    unfold_nodes(path, &(super::root), point, active);
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compute_vocabulary_vector(std::map<node*, double>& query_id_freqs,
                                                          CloudPtrT& query_cloud, map<node*, double>& active)
{
    for (PointT p : query_cloud->points) {
        std::vector<node*> path;
        get_path_for_point(path, p, active);
        int current_depth = 0;
        for (node* n : path) {
            if (current_depth >= matching_min_depth) {
                query_id_freqs[n] += 1.0f;
            }
            ++current_depth;
        }
    }

    for (std::pair<node* const, double>& v : query_id_freqs) {
        v.second = v.first->weight*v.second;
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

// we should make some const here to make clear what happens
template <typename Point, size_t K>
double vocabulary_tree<Point, K>::compute_min_combined_dist(vector<int>& included_indices, CloudPtrT& cloud, vector<vocabulary_vector>& smaller_freqs,
                                                            set<pair<int, int> >& adjacencies, map<node*, int>& mapping, map<int, node*>& inverse_mapping, int hint) // TODO: const
{
    vector<int> subgroup_indices;
    for (const vocabulary_vector& vec : smaller_freqs) {
        //cout << vec.subgroup << endl;
        subgroup_indices.push_back(vec.subgroup);
    }

    // used to return which indices are picked
    vector<int> remaining_indices;
    for (int i = 0; i < smaller_freqs.size(); ++i) {
        remaining_indices.push_back(i);
    }

    vector<double> pnorms(smaller_freqs.size(), 0.0); // compute these from smaller_freqs and current vocab weights
    for (int i = 0; i < smaller_freqs.size(); ++i) {
        for (const pair<int, pair<int, double> >& u : smaller_freqs[i].vec) {
            pnorms[i] += pexp(inverse_mapping[u.first]->weight*double(u.second.first));
        }
    }

    // first compute vectors to describe cloud and smaller_clouds
    map<int, double> cloud_freqs;
    double qnorm = compute_query_index_vector(cloud_freqs, cloud, mapping);
    double vnorm = 0.0;

    map<int, double> source_freqs; // to be filled in

    if (hint != -1) {
        cout << "We got hint: " << hint << endl; // REMOVE
        hint = std::distance(subgroup_indices.begin(), std::find(subgroup_indices.begin(), subgroup_indices.end(), hint));
        if (hint >= smaller_freqs.size()) {
            cout << "Using hint = " << hint << endl;
            cout << "We have total voxels: " << smaller_freqs.size() << endl;
            exit(0);
        }
    }

    double last_dist = std::numeric_limits<double>::infinity(); // large
    // repeat until the smallest vector is
    while (!smaller_freqs.empty()) {
        double mindist = std::numeric_limits<double>::infinity(); // large
        int minind = -1;

        for (size_t i = 0; i < smaller_freqs.size(); ++i) {
            // if added any parts, check if close enough to previous ones
            if (!included_indices.empty()) {
                bool close_enough = false;
                for (int j : included_indices) {
                    //cout << "<<<<<<" << endl;
                    //cout << j << endl;
                    //cout << remaining_indices.at(i) << endl;
                    //cout << subgroup_indices.size() << endl;
                    //cout << ">>>>>>" << endl;
                    if (adjacencies.count(make_pair(subgroup_indices[j], subgroup_indices[remaining_indices.at(i)])) ||
                        adjacencies.count(make_pair(subgroup_indices[remaining_indices.at(i)], subgroup_indices[j]))) {
                        close_enough = true;
                        break;
                    }
                }
                if (!close_enough) {
                    //cout << "Did not find any adjacencies!" << endl;
                    continue;
                }
                else {
                    //cout << "Found adjacencies!" << endl;
                }
            }
            else if (hint != -1) {
                i = hint;
            }

            //double normalization = 1.0/std::max(pnorms[i] + vnorm, qnorm);
            double dist = 0.0;
            double normdiff = 0.0;
            for (const pair<const int, double>& v : cloud_freqs) {
                // compute the distance that we are interested in
                double source_comp = 0.0;
                double cand_comp = 0.0;
                if (source_freqs.count(v.first) != 0) { // this could be maintained in a map as a pair with cloud_freqs
                    source_comp = source_freqs[v.first];
                }
                if (smaller_freqs[i].vec.count(v.first) != 0) {
                    cand_comp = inverse_mapping[v.first]->weight*double(smaller_freqs[i].vec[v.first].first);
                }
                normdiff += pexp(source_comp) + pexp(cand_comp) - pexp(source_comp+cand_comp);
                if (source_comp != 0 || cand_comp != 0) {
                    //dist -= normalization*std::min(v.second, sum_val);
                    dist += std::min(v.second, source_comp + cand_comp);
                }
            }
            dist = 1.0 - dist/std::max(pnorms[i] + vnorm - normdiff, qnorm);
            if (dist < mindist) {
                mindist = dist;
                minind = i;
            }

            if (hint != -1 && included_indices.empty()) {
                break;
            }
        }

        if (mindist > last_dist) {
            break;
        }

        last_dist = mindist;
        //vnorm += pnorms[minind];

        for (pair<const int, pair<int, double> >& v : smaller_freqs[minind].vec) {
            double val = inverse_mapping[v.first]->weight*double(v.second.first);
            if (source_freqs.count(v.first) != 0) {
                vnorm += pexp(source_freqs[v.first]+val) - pexp(source_freqs[v.first]);
            }
            else {
                vnorm += pexp(val);
            }
            source_freqs[v.first] += val;
        }

        // remove the ind that we've used in the score now
        smaller_freqs.erase(smaller_freqs.begin() + minind);
        pnorms.erase(pnorms.begin() + minind);
        included_indices.push_back(remaining_indices.at(minind));
        remaining_indices.erase(remaining_indices.begin() + minind);
    }

    /*
    double recomputed_norm = 0.0;
    for (const pair<int, double>& u : source_freqs) {
        recomputed_norm += pexp(u.second);
    }

    cout << "vnorm: " << vnorm << endl;
    cout << "recomputed: " << recomputed_norm << endl;
    */

    for (int& i : included_indices) {
        i = subgroup_indices[i];
    }

    return last_dist;
}

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::compute_vocabulary_norm(CloudPtrT& cloud)
{
    map<node*, double> cloud_freqs;
    return compute_query_vector(cloud_freqs, cloud);
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

    super::add_points_from_input_cloud();

    for (leaf* l : super::leaves) {
        l->data = new inverted_file;
        for (int ind : l->inds) {
            int source_id = indices[ind];
            l->data->source_id_freqs[source_id] += 1;
        }
    }

    compute_normalizing_constants();

    if (!save_cloud) {
        super::cloud->clear();
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
        //Eigen::Matrix<float, super::rows, super::dim> child_centers;
        //int counter = 0;
        for (node* c : n->children) {
            // here we need one set of normalizing constants for every child to not mess up scores between subtrees
            std::map<int, int> child_normalizing_constants;
            normalizing_constants_for_node(child_normalizing_constants, c, current_depth+1);
            for (std::pair<const int, int>& v : child_normalizing_constants) {
                normalizing_constants[v.first] += v.second;
            }

            //child_centers.col(counter) = eig(c->centroid);
            //++counter;
        }

        /*Eigen::Matrix<float, super::rows, 1> m = child_centers.rowwise().mean();
        child_centers.colwise() -= m;
        Eigen::Matrix<float, 1, super::dim> distances = child_centers.colwise().norm();
        float variance = 1.0/float(super::dim)*distances*distances.transpose();

        for (node* c : n->children) {
            c->weight += log(variance);
        }*/
    }

    if (normalizing_constants.empty()) {
        n->weight = 0.0;
    }
    else {
        n->weight = log(N) - log(double(normalizing_constants.size()));
    }

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
void vocabulary_tree<Point, K>::top_combined_similarities(std::vector<result_type>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    std::map<node*, double> query_id_freqs;
    double qnorm = compute_query_vector(query_id_freqs, query_cloud);
    std::map<int, double> map_scores;

    //int skipped = 0;
    for (const std::pair<node*, double>& v : query_id_freqs) {
        double qi = v.second;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);
        /*if (source_id_freqs.size() < 20) {
            ++skipped;
            continue;
        }*/
        for (const std::pair<int, int>& u : source_id_freqs) {
            map_scores[u.first] += std::min(v.first->weight*double(u.second), qi);
        }
    }

    //cout << "Skipped " << float(skipped)/float(query_id_freqs.size()) << endl;

    for (pair<const int, double>& u : map_scores) {
        double dbnorm = db_vector_normalizing_constants[u.first];
        u.second = 1.0 - u.second/std::max(qnorm, dbnorm);
    }

    // this could probably be optimized a bit also, quite big copy operattion
    //scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    scores.reserve(map_scores.size());
    for (const pair<int, double>& s : map_scores) {
        if (!std::isnan(s.second)) {
            scores.push_back(result_type {s.first, float(s.second)});
        }
    }
    std::sort(scores.begin(), scores.end(), [](const result_type& s1, const result_type& s2) {
        return s1.score < s2.score; // find min elements!
    });

    if (nbr_results > 0 && scores.size() > nbr_results) {
        scores.resize(nbr_results);
    }
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::debug_similarities(std::vector<result_type>& scores, CloudPtrT& query_cloud, size_t nbr_results)
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
            double dbnorm = db_vector_normalizing_constants[u.first];
            if (normalized) {
                pk = dbnorm; // 1.0f for not normalized
                pkr = proot(pk);
            }
            double pi = v.first->weight*double(u.second)/pkr;
            double residual = pexp(qi-pi)-pexp(pi)-pexp(qi); // = 2*(pexp(std::max(qi-pi, 0.0))-pexp(qi));
            if (map_scores.count(u.first) == 1) {
                map_scores.at(u.first) += residual;
            }
            else {
                map_scores.insert(std::make_pair(u.first, dbnorm/pk+qnorm/qk+residual));
            }
        }
    }

    // this could probably be optimized a bit also, quite big copy operattion
    scores.reserve(map_scores.size());
    for (const pair<int, double>& s : map_scores) {
        scores.push_back(result_type {s.first, float(s.second)});
    }
    std::sort(scores.begin(), scores.end(), [](const result_type& s1, const result_type& s2) {
        return s1.score < s2.score; // find min elements!
    });

    if (nbr_results > 0 && scores.size() > nbr_results) {
        scores.resize(nbr_results);
    }
}

// all of these might not be necessary but could be useful
template <typename Point, size_t K>
double vocabulary_tree<Point, K>::compute_query_vector(std::map<node*, double>& query_id_freqs, CloudPtrT& query_cloud)
{
    Eigen::Matrix<float, super::rows, 1> pe;

    for (PointT p : query_cloud->points) {
        pe = eig(p);
        if (std::find_if(pe.data(), pe.data()+super::rows, [] (float f) {
            return std::isnan(f) || std::isinf(f);
        }) != pe.data()+super::rows) {
            continue;
        }
        std::vector<node*> path;
        super::get_path_for_point(path, p);
        int current_depth = 0;
        for (node* n : path) {
            if (current_depth >= matching_min_depth) {
                query_id_freqs[n] += 1.0;
            }
            ++current_depth;
        }
    }
    double qnorm = 0.0;
    for (std::pair<node* const, double>& v : query_id_freqs) {
        v.second = v.first->weight*v.second;
        qnorm += pexp(v.second);
    }

    return qnorm;
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compute_query_vector(map<node*, int>& query_id_freqs, CloudPtrT& query_cloud)
{
    Eigen::Matrix<float, super::rows, 1> pe;

    for (PointT p : query_cloud->points) {
        pe = eig(p);
        if (std::find_if(pe.data(), pe.data()+super::rows, [] (float f) {
            return std::isnan(f) || std::isinf(f);
        }) != pe.data()+super::rows) {
            continue;
        }
        std::vector<node*> path;
        super::get_path_for_point(path, p);
        int current_depth = 0;
        for (node* n : path) {
            if (current_depth >= matching_min_depth) {
                query_id_freqs[n] += 1;
            }
            ++current_depth;
        }
    }
}

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::compute_query_vector(map<node*, pair<double, int> >& query_id_freqs, CloudPtrT& query_cloud)
{
    for (PointT p : query_cloud->points) {
        std::vector<node*> path;
        super::get_path_for_point(path, p);
        int current_depth = 0;
        for (node* n : path) {
            if (current_depth >= matching_min_depth) {
                pair<double, int>& value = query_id_freqs[n];
                value.first += 1.0f;
                value.second = current_depth;
            }
            ++current_depth;
        }
    }
    double qnorm = 0.0f;
    for (std::pair<node* const, pair<double, int> >& v : query_id_freqs) {
        v.second.second = v.first->weight*v.second.second;
        qnorm += pexp(v.second.second);
    }

    return qnorm;
}

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::compute_query_index_vector(map<int, double>& query_index_freqs, CloudPtrT& query_cloud, map<node*, int>& mapping)
{
    map<node*, double> query_node_freqs;
    double qnorm = compute_query_vector(query_node_freqs, query_cloud);
    for (pair<node* const, double>& u : query_node_freqs) {
        query_index_freqs[mapping[u.first]] = u.second;
    }
    return qnorm;
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::compute_query_index_vector(map<int, int>& query_index_freqs, CloudPtrT& query_cloud, map<node*, int>& mapping)
{
    map<node*, int> query_node_freqs;
    compute_query_vector(query_node_freqs, query_cloud);
    for (const pair<node*, int>& u : query_node_freqs) {
        query_index_freqs[mapping[u.first]] = u.second;
    }
}

// this version computes the unnormalized and normalized histograms, basically the both base version at the same time
template <typename Point, size_t K>
vocabulary_vector vocabulary_tree<Point, K>::compute_query_index_vector(CloudPtrT& query_cloud, map<node*, int>& mapping)
{
    // the mapping is gonna have to be internal to the class, no real idea in having it outside
    vocabulary_vector vec;
    map<node*, int> query_node_freqs;
    compute_query_vector(query_node_freqs, query_cloud);
    vec.norm = 0.0;
    for (const pair<node*, int>& u : query_node_freqs) {
        double element = u.first->weight*double(u.second);
        vec.vec[mapping[u.first]] = make_pair(u.second, element);
        vec.norm += pexp(element);
    }

    return vec;
}

/*
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
    cout << "Finished loading vocabulary_tree" << endl;
}
*/

template <typename Point, size_t K>
double vocabulary_tree<Point, K>::compute_distance(const std::map<node*, double>& vec1, double norm1,
                                                   const std::map<node*, double>& vec2, double norm2) const
{
    double score = 0.0;
    for (const pair<node* const, double>& n : vec1) {
        if (vec2.count(n.first) == 0) {
            continue;
        }
        // actually we don't need to check the depth of the node since that is already handled by compute_query_vector: GREAT!
        score += std::min(n.second, vec2.at(n.first));
    }
    score = 1.0 - score/std::max(norm1, norm2);
    return score;
}
