#include "vocabulary_tree/vocabulary_tree.h"
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>

#include <Eigen/Core>

#include <chrono> // DEBUG

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
double vocabulary_tree<Point, K>::compute_min_combined_dist(CloudPtrT& cloud, vector<CloudPtrT>& smaller_clouds, vector<double>& pnorms,
                                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& centers) // TODO: const
{
    chrono::time_point<std::chrono::system_clock> start1, end1;
    start1 = chrono::system_clock::now();

    pcl::PointCloud<pcl::PointXYZRGB> remaining_centers;
    remaining_centers += *centers;
    pcl::PointCloud<pcl::PointXYZRGB> included_centers;

    // first compute vectors to describe cloud and smaller_clouds
    map<node*, double> cloud_freqs;
    double qnorm = compute_query_vector(cloud_freqs, cloud);
    double vnorm = 0.0;

    map<node*, double> source_freqs; // to be filled in

    vector<map<node*, double> > smaller_freqs(smaller_clouds.size());
    //vector<double> pnorms(smaller_clouds.size());
    for (size_t i = 0; i < smaller_clouds.size(); ++i) {
        //pnorms[i] = compute_query_vector(smaller_freqs[i], smaller_clouds[i]);
        compute_vocabulary_vector(smaller_freqs[i], smaller_clouds[i], cloud_freqs);
    }

    end1 = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds1 = end1-start1;
    cout << "First part took " << elapsed_seconds1.count() << " seconds" << endl;

    chrono::time_point<std::chrono::system_clock> start2, end2;
    start2 = chrono::system_clock::now();

    double last_dist = std::numeric_limits<double>::infinity(); // large
    // repeat until the smallest vector is
    while (!smaller_freqs.empty()) {
        double mindist = std::numeric_limits<double>::infinity(); // large
        int minind = -1;
        double mincompdist = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < smaller_freqs.size(); ++i) {
            // if added any parts, check if close enough to previous ones
            if (!included_centers.empty()) {
                bool close_enough = false;
                for (pcl::PointXYZRGB& p : included_centers) {
                    if ((eig(p) - eig(remaining_centers.at(i))).norm() < 0.2f) {
                        close_enough = true;
                        break;
                    }
                }
                if (!close_enough) {
                    continue;
                }
            }

            //map<node*, double>& m = smaller_freqs[i];
            //double dist = std::max(pnorms[i] + vnorm, qnorm);
            double normalization = 1.0/std::max(pnorms[i] + vnorm, qnorm);
            double dist = 1.0;
            double compdist = pnorms[i] + vnorm;
            for (pair<node* const, double>& v : cloud_freqs) {
                // compute the distance that we are interested in
                double sum_val = 0.0;
                if (source_freqs.count(v.first) != 0) { // this could be maintained in a map as a pair with cloud_freqs
                    sum_val += source_freqs[v.first];
                }
                if (smaller_freqs[i].count(v.first) != 0) {
                    sum_val += smaller_freqs[i][v.first];
                }
                if (sum_val != 0) {
                    //dist -= std::min(v.second, sum_val);
                    dist -= normalization*std::min(v.second, sum_val);
                }

                // compute the distance that we use to compare
                if (sum_val != 0) {
                    compdist += std::max(sum_val - v.second, 0.0) - sum_val; // = max(-v.second, -sum_val) = -min(v.second, sum_val)
                }
            }
            compdist /= (pnorms[i] + vnorm);
            if (dist < mindist) {
                mincompdist = compdist;
                mindist = dist;
                minind = i;
            }
        }

        if (mindist > last_dist) {
            break;
        }

        last_dist = mindist;
        vnorm += pnorms[minind];

        for (pair<node* const, double>& v : smaller_freqs[minind]) {
            source_freqs[v.first] += v.second;
        }

        // remove the ind that we've used in the score now
        smaller_freqs.erase(smaller_freqs.begin() + minind);
        pnorms.erase(pnorms.begin() + minind);
        included_centers.push_back(remaining_centers.at(minind));
        remaining_centers.erase(remaining_centers.begin() + minind);
    }

    end2 = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds2 = end2-start2;
    cout << "Second part took " << elapsed_seconds2.count() << " seconds" << endl;

    cout << "Breaking with " << smaller_freqs.size() << " vectors out of " << smaller_clouds.size() << " left to go." << endl;

    return last_dist;
}

// keep in mind that smaller_freqs will be modified here
template <typename Point, size_t K>
double vocabulary_tree<Point, K>::compute_min_combined_dist(CloudPtrT& cloud, vector<map<int, double> >& smaller_freqs, vector<double>& pnorms,
                                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& centers, map<node*, int>& mapping, int hint) // TODO: const
{
    pcl::PointCloud<pcl::PointXYZRGB> remaining_centers;
    remaining_centers += *centers;
    pcl::PointCloud<pcl::PointXYZRGB> included_centers;

    // first compute vectors to describe cloud and smaller_clouds
    map<int, double> cloud_freqs;
    double qnorm = compute_query_index_vector(cloud_freqs, cloud, mapping);
    double vnorm = 0.0;

    map<int, double> source_freqs; // to be filled in

    if (hint != -1) {
        if (hint >= centers->size()) {
            cout << "Using hint = " << hint << endl;
            cout << "We have total voxels: " << centers->size() << endl;
            //exit(0);
        }
    }

    double last_dist = std::numeric_limits<double>::infinity(); // large
    // repeat until the smallest vector is
    while (!smaller_freqs.empty()) {
        double mindist = std::numeric_limits<double>::infinity(); // large
        int minind = -1;

        for (size_t i = 0; i < smaller_freqs.size(); ++i) {
            // if added any parts, check if close enough to previous ones
            if (!included_centers.empty()) {
                bool close_enough = false;
                for (pcl::PointXYZRGB& p : included_centers) {
                    if ((eig(p) - eig(remaining_centers.at(i))).norm() < 0.2f) {
                        close_enough = true;
                        break;
                    }
                }
                if (!close_enough) {
                    continue;
                }
            }
            else if (hint != -1) {
                i = hint;
            }

            double normalization = 1.0/std::max(pnorms[i] + vnorm, qnorm);
            double dist = 1.0;
            for (pair<const int, double>& v : cloud_freqs) {
                // compute the distance that we are interested in
                double sum_val = 0.0;
                if (source_freqs.count(v.first) != 0) { // this could be maintained in a map as a pair with cloud_freqs
                    sum_val += source_freqs[v.first];
                }
                if (smaller_freqs[i].count(v.first) != 0) {
                    sum_val += smaller_freqs[i][v.first];
                }
                if (sum_val != 0) {
                    dist -= normalization*std::min(v.second, sum_val);
                }
            }
            if (dist < mindist) {
                mindist = dist;
                minind = i;
            }

            if (hint != -1 && included_centers.empty()) {
                break;
            }
        }

        if (mindist > last_dist) {
            break;
        }

        last_dist = mindist;
        vnorm += pnorms[minind];

        for (pair<const int, double>& v : smaller_freqs[minind]) {
            source_freqs[v.first] += v.second;
        }

        // remove the ind that we've used in the score now
        smaller_freqs.erase(smaller_freqs.begin() + minind);
        pnorms.erase(pnorms.begin() + minind);
        included_centers.push_back(remaining_centers.at(minind));
        remaining_centers.erase(remaining_centers.begin() + minind);
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

/*template <typename Point, size_t K>
void vocabulary_tree<Point, K>::top_combined_similarities(vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    vector<cloud_idx_score> larger_scores;
    top_larger_similarities(larger_scores, query_cloud, 0);
    vector<cloud_idx_score> smaller_scores;
    top_smaller_similarities(smaller_scores, query_cloud, 0);

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
}*/

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::top_combined_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    std::map<node*, double> query_id_freqs;
    double qnorm = compute_query_vector(query_id_freqs, query_cloud);
    std::map<int, double> map_scores;

    for (std::pair<node* const, double>& v : query_id_freqs) {
        double qi = v.second;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);

        for (std::pair<const int, int>& u : source_id_freqs) {
            double dbnorm = db_vector_normalizing_constants[u.first];
            double pi = v.first->weight*double(u.second);
            double normalization = std::max(qnorm, dbnorm);
            double residual = std::min(pi, qi)/normalization;
            if (map_scores.count(u.first) != 0) {
                map_scores.at(u.first) -= residual;
            }
            else {
                map_scores.insert(std::make_pair(u.first, 1.0-residual));
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
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::top_larger_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    std::map<node*, double> query_id_freqs;
    double qnorm = compute_query_vector(query_id_freqs, query_cloud);
    std::map<int, double> map_scores;

    for (std::pair<node* const, double>& v : query_id_freqs) {
        double qi = v.second;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);

        for (std::pair<const int, int>& u : source_id_freqs) {
            double pi = v.first->weight*double(u.second);
            double residual = pexp(std::max(qi-pi, 0.0))-pexp(qi);
            double normalization = qnorm;
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
}

template <typename Point, size_t K>
void vocabulary_tree<Point, K>::top_smaller_similarities(std::vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    std::map<node*, double> query_id_freqs;
    double qnorm = compute_query_vector(query_id_freqs, query_cloud);
    std::map<int, double> map_scores;

    for (std::pair<node* const, double>& v : query_id_freqs) {
        double qi = v.second;
        std::map<int, int> source_id_freqs;
        source_freqs_for_node(source_id_freqs, v.first);

        for (std::pair<const int, int>& u : source_id_freqs) {

            double dbnorm = db_vector_normalizing_constants[u.first];
            double pi = v.first->weight*double(u.second);
            double residual = pexp(std::max(pi-qi, 0.0))-pexp(pi); // ~ (|q_i-p_i|-|q_i|-|p_i|)+|q_i|
            double normalization = dbnorm;
            if (map_scores.count(u.first) != 0) {
                map_scores.at(u.first) += residual/normalization;
            }
            else {
                map_scores.insert(std::make_pair(u.first, (dbnorm+residual)/normalization));
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
double vocabulary_tree<Point, K>::compute_query_vector(std::map<node*, pair<double, int> >& query_id_freqs, CloudPtrT& query_cloud)
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
size_t vocabulary_tree<Point, K>::deep_count_sets()
{
    unordered_set<int> indices_ids;
    for (leaf* l : super::leaves) {
        for (const pair<int, int>& i : l->data->source_id_freqs) {
            indices_ids.insert(i.first);
        }
    }
    cout << "cloud size: " << indices.size() << endl;
    return indices_ids.size();
}
