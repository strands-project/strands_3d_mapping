#include "grouped_vocabulary_tree/grouped_vocabulary_tree.h"

#include <cereal/types/unordered_map.hpp>

using namespace std;

// the first index is the segment, the second one is the oversegment
template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::set_input_cloud(CloudPtrT& new_cloud, vector<pair<int, int> >& indices)
{
    // to begin with, we might have to remove nan points
    Eigen::Matrix<float, super::rows, 1> p;
    CloudPtrT temp_cloud(new CloudT);
    temp_cloud->reserve(new_cloud->size());
    vector<pair<int, int> > temp_indices;
    temp_indices.reserve(indices.size());

    for (size_t i = 0; i < new_cloud->size(); ++i) {
        p = eig(new_cloud->at(i));
        if (std::find_if(p.data(), p.data()+super::rows, [] (float f) {
            return std::isnan(f) || std::isinf(f);
        }) == p.data()+super::rows) {
            temp_cloud->push_back(new_cloud->at(i));
            temp_indices.push_back(indices[i]);
        }
    }

    cout << "Temp cloud size: " << temp_cloud->size() << endl;
    cout << "Indices size: " << temp_indices.size() << endl;

    //std::sort(temp_indices.begin(), temp_indices.end(), [](const pair<int, int>& p1, const pair<int, int>& p2) {
    //    return p1.first < p2.first && p1.second < p2.second;
    //});

    vector<int> new_indices(temp_indices.size());

    pair<int, int> previous_p = make_pair(-1, -1);
    int index_group_ind = -1;
    int counter = 0;
    for (pair<int, int>& p : temp_indices) {
        // everything with this index pair should have the same label, assume ordered
        if (p != previous_p) {
            ++index_group_ind;
            index_group[index_group_ind] = p.first;
            previous_p = p;
        }
        new_indices[counter] = index_group_ind;
        ++counter;
    }

    nbr_points = counter;
    nbr_groups = index_group_ind + 1;

    cout << "New indices size: " << temp_indices.size() << endl;

    super::set_input_cloud(temp_cloud, new_indices);
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::append_cloud(CloudPtrT& extra_cloud, vector<pair<int, int> >& indices, bool store_points)
{
    // to begin with, we might have to remove nan points
    Eigen::Matrix<float, super::rows, 1> p;
    CloudPtrT temp_cloud(new CloudT);
    temp_cloud->reserve(extra_cloud->size());
    vector<pair<int, int> > temp_indices;
    temp_indices.reserve(indices.size());

    for (size_t i = 0; i < extra_cloud->size(); ++i) {
        p = eig(extra_cloud->at(i));
        if (std::find_if(p.data(), p.data()+super::rows, [] (float f) {
            return std::isnan(f) || std::isinf(f);
        }) == p.data()+super::rows) {
            temp_cloud->push_back(extra_cloud->at(i));
            temp_indices.push_back(indices[i]);
        }
    }

    // assume these are all new groups
    vector<int> new_indices(temp_indices.size());

    pair<int, int> previous_p = make_pair(-1, -1);
    int index_group_ind = nbr_groups - 1;
    int counter = 0;
    for (pair<int, int>& p : temp_indices) {
        // everything with this index pair should have the same label, assume ordered
        if (p != previous_p) {
            ++index_group_ind;
            index_group[index_group_ind] = p.first;
            previous_p = p;
        }
        new_indices[counter] = index_group_ind;
        ++counter;
    }

    nbr_points += counter;
    nbr_groups = index_group_ind + 1;

    super::append_cloud(temp_cloud, new_indices, store_points);
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::add_points_from_input_cloud(bool save_cloud)
{
    super::add_points_from_input_cloud(save_cloud);
}

template <typename Point, size_t K>
double grouped_vocabulary_tree<Point, K>::calculate_similarity(int i, int j)
{
    CloudPtrT cloudi(new CloudT);
    CloudPtrT cloudj(new CloudT);

    // first I guess search for all the indices where i and j are
    for (leaf* l : super::leaves) {
        if (l->data->source_id_freqs.count(i) != 0) {
            size_t points = l->data->source_id_freqs[i];
            for (size_t k = 0; k < points; ++k) {
                cloudi->push_back(l->centroid);
            }
        }
        if (l->data->source_id_freqs.count(j) != 0) {
            size_t points = l->data->source_id_freqs[j];
            for (size_t k = 0; k < points; ++k) {
                cloudj->push_back(l->centroid);
            }
        }
    }

    std::map<node*, double> query_id_freqsi;
    double qnormi = super::compute_query_vector(query_id_freqsi, cloudi);

    std::map<node*, double> query_id_freqsj;
    double qnormj = super::compute_query_vector(query_id_freqsj, cloudj);

    //double similarity = qnormi + qnormj;
    double similarity = 0.0;

    for (std::pair<node* const, double>& v : query_id_freqsi) {
        if (query_id_freqsj.count(v.first) == 0) {
            continue;
        }
        double qi = v.second;
        double pi = query_id_freqsj[v.first];
        //double residual = pexp(pi-qi)-pexp(pi)-pexp(qi);

        //similarity += residual;
        similarity += std::min(pi, qi);
    }

    return similarity;
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
void grouped_vocabulary_tree<Point, K>::merge_maps(map<int, int>& map1, const map<int, int>& map2)
{
    using iter = map<int, int>::iterator;
    using citer = map<int, int>::const_iterator;

    iter it1 = map1.begin();
    citer it2 = map2.cbegin();
    iter end1 = map1.end();
    citer end2 = map2.cend();
    while (it2 != end2) {
        bool done1 = it1 == end1;
        if (!done1 && it1->first == it2->first) { // element already in 1, just increase count
            it1->second += it2->second;
            ++it1;
            ++it2;
        }
        else {
            if (done1 || it1->first > it2->first) {
                // from c++11 docs:
                // The function optimizes its insertion time if position points to the element
                // that will follow the inserted element (or to the end, if it would be the last).
                map1.insert(it1, *it2);
                ++it2;
            }
            else {
                ++it1; // if map1 is large while map2 is small this will not give better than const time insertion
            }
        }
    }
}

/*
 * The idea here is that we only look at the nodes that are present at some depth,
 * set that are not present anywhere deeper than this, we will just ignore
 * this means that for the first few levels, we only expand if qvec also has a path there
 * When given a bit more thought this will actually not do since the paths that separate
 * above the level cut will not influence which other paths get expanded
 *
 */

/*template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::compute_node_difference(map<int, int>& sub_freqs, map<int, double>& map_scores,
                                                                node* n, map<node*, double>& qvec, int current_depth)
{
    if (n->is_leaf) {
        leaf* l = static_cast<leaf*>(n);
        sub_freqs = l->data->source_id_freqs;
    }
    else {
        for (node* c : n->children) {
            if (qvec.count(c) == 0) {
                continue;
            }
            map<int, int> child_freqs;
            compute_node_difference(child_freqs, map_scores, c, qvec, current_depth+1);
            if (current_depth >= super::matching_min_depth) {
                for (const pair<int, int>& u : child_freqs) {
                    sub_freqs[u.first] += u.second;
                }
            }
        }
    }

    double weight = n->weight;
    double qi = qvec[n];
    for (const pair<int, int>& u : sub_freqs) {
        map_scores[u.first] += std::min(weight*double(u.second), qi);
    }
}*/

/*template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::compute_node_vocabulary_vector(Eigen::SparseVector<int>& node_vector, node* n)
{
    for (int i = n->range.first; i < n->range.second; ++i) {
        node_vector += leaf_vocabulary_vectors[i];
    }
}*/

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::compute_node_vocabulary_vector(unordered_map<int, double>& node_vector, node* n)
{
    if (false) {//leaf_ranges.count(n) > 0) {
        leaf_range node_range = leaf_ranges[n];
        for (int i = node_range.first; i < node_range.second; ++i) {
            for (const pair<int, int>& u : intermediate_node_vocabulary_vectors[i]) {
                node_vector[u.first] += n->weight*double(u.second);
            }
        }
    }
    else {
        for (int i = n->range.first; i < n->range.second; ++i) {
            for (const pair<int, int>& u : leaf_vocabulary_vectors[i]) {
                node_vector[u.first] += n->weight*double(u.second);
            }
        }
    }
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::top_optimized_similarities(vector<tuple<int, int, double> >& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    std::map<node*, double> query_id_freqs;
    double qnorm = super::compute_query_vector(query_id_freqs, query_cloud);

    unordered_map<int, double> vocabulary_difference(max_index);

    double epsilon = 7;
    int skipped = 0;
    for (pair<node* const, double>& v : query_id_freqs) {

        if (v.first->weight < epsilon) {
            ++skipped;
            continue;
        }

        unordered_map<int, double> node_vocabulary_vector; // includes weights
        compute_node_vocabulary_vector(node_vocabulary_vector, v.first);

        for (const pair<int, double>& u : node_vocabulary_vector) {
            //present_at_lower_levels.insert(u.first);
            vocabulary_difference[u.first] += std::min(u.second, v.second);
        }
    }

    cout << "Skipped " << float(skipped)/float(query_id_freqs.size()) << " nodes" << endl;

    for (pair<const int, double>& u : vocabulary_difference) {
        u.second = 1.0 - u.second/std::max(qnorm, super::db_vector_normalizing_constants[u.first]);
    }

    unordered_map<int, pair<int, double> > map_scores;
    for (const pair<int, double>& s : vocabulary_difference) {
        pair<int, int> grouped_ind = group_subgroup[s.first];
        if (map_scores.count(grouped_ind.first) > 0) {
            pair<int, double>& value = map_scores[grouped_ind.first];
            if (s.second < value.second) {
                value = make_pair(grouped_ind.second, s.second);
            }
        }
        else {
            map_scores.insert(make_pair(grouped_ind.first, make_pair(grouped_ind.second, s.second)));
        }
    }

    // this could probably be optimized a bit also, quite big copy operattion
    //scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    scores.reserve(map_scores.size());
    for (const pair<int, pair<int, double> >& u : map_scores) {
        scores.push_back(make_tuple(u.first, u.second.first, u.second.second));
    }
    std::sort(scores.begin(), scores.end(), [](const tuple<int, int, double>& s1, const tuple<int, int, double>& s2) {
        return get<2>(s1) < get<2>(s2); // find min elements!
    });

    if (nbr_results > 0) {
        scores.resize(nbr_results);
    }

    /*for (tuple<int, int, double>& t : scores) {
        //cout << "min value index: " << endl;
        //cout << get<1>(t) << endl;
        //cout << "min group index: " << endl;
        //cout << min_group_index[get<0>(t)] << endl;
        get<1>(t) = group_subgroup[get<0>(t)].second;//-= min_group_index[get<0>(t)]; // we don't know which supervoxel it belongs to anymore, just the image
        // let's see if we can change this
    }*/
}

/*template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::compute_leaf_vocabulary_vectors()
{
    cout << "Resizing the vocabulary vector" << endl;

    leaf_vocabulary_vectors.resize(super::leaves.size());

    cout << "Trying to find the max" << endl;

    max_index = 0;
    for (leaf* l : super::leaves) {
        if (l->inds.empty()) {
            continue;
        }
        auto it = std::max_element(l->inds.begin(), l->inds.end());
        if (*it > max_index) {
            max_index = *it;
        }
    }
    max_index += 1;

    cout << "Got through finding the max" << endl;

    for (size_t i = 0; i < super::leaves.size(); ++i) {
        leaf_vocabulary_vectors[i].resize(max_index);
        leaf_vocabulary_vectors[i].reserve(super::leaves[i]->data->source_id_freqs.size());
        for (const pair<int, int>& u : super::leaves[i]->data->source_id_freqs) {
            leaf_vocabulary_vectors[i].insert(u.first) = u.second;
        }
    }

    cout << "Finished computing leaf vocabulary vectors..." << endl;
}*/

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::compute_min_group_indices()
{
    for (const pair<int, int>& i : index_group) {
        if (min_group_index.count(i.second) > 0) {
            int& value = min_group_index[i.second];
            value = std::min(value, i.first);
        }
        else {
            min_group_index.insert(make_pair(i.second, i.first));
        }
    }
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::compute_leaf_vocabulary_vectors()
{
    leaf_vocabulary_vectors.resize(super::leaves.size());
    for (size_t i = 0; i < super::leaves.size(); ++i) {
        for (const pair<int, int>& u : super::leaves[i]->data->source_id_freqs) {
            leaf_vocabulary_vectors[i].insert(u);
        }
    }
}

template<typename Point, size_t K>
typename grouped_vocabulary_tree<Point, K>::leaf_range grouped_vocabulary_tree<Point, K>::compute_vector_for_nodes_at_depth(node* n, int depth, int current_depth)
{
    leaf_range rtn = leaf_range(max_index, 0);
    if (current_depth < depth) {
        for (node* c : n->children) {
            leaf_range child_range = compute_vector_for_nodes_at_depth(c, depth, current_depth+1);
            rtn.first = std::min(rtn.first, child_range.first);
            rtn.second = std::max(rtn.second, child_range.second);
        }
    }
    else {
        rtn.first = intermediate_node_vocabulary_vectors.size();
        rtn.second = rtn.first + 1;
        intermediate_node_vocabulary_vectors.push_back(std::unordered_map<int, int>());
        for (int i = n->range.first; i < n->range.second; ++i) {
            for (const pair<int, int>& u : super::leaves[i]->data->source_id_freqs) {
                intermediate_node_vocabulary_vectors.back()[u.first] += u.second;
            }
        }
    }
    leaf_ranges.insert(make_pair(n, rtn));
    return rtn;
}

template<typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::compute_intermediate_node_vocabulary_vectors()
{
    compute_vector_for_nodes_at_depth(&(super::root), 3, 0);
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::top_combined_similarities(vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    vector<cloud_idx_score> smaller_scores;
    super::top_combined_similarities(smaller_scores, query_cloud, 0);
    map<int, double> map_scores;
    for (cloud_idx_score& s : smaller_scores) {
        if (map_scores.size() >= nbr_results) {
            break;
        }
        int grouped_ind = index_group[s.first];
        if (map_scores.count(grouped_ind) > 0) {
            double& value = map_scores[grouped_ind];
            value = std::min(value, s.second);
        }
        else {
            map_scores.insert(cloud_idx_score(grouped_ind, s.second));
        }
    }
    scores.insert(scores.end(), map_scores.begin(), map_scores.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second < s2.second; // find min elements!
    });
    if (nbr_results > 0) {
        scores.resize(nbr_results);
    }
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::top_grouped_similarities(vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    vector<cloud_idx_score> sub_scores;

    size_t nbr_compare_groups = size_t(0.01f*float(nbr_groups)); // take the 5% top percentile

    super::top_smaller_similarities(sub_scores, query_cloud, nbr_compare_groups);

    map<int, double> group_scores;

    map<int, vector<int> > scoring_groups;

    for (cloud_idx_score s : sub_scores) {
        group_scores[index_group[s.first]] += s.second;
        scoring_groups[index_group[s.first]].push_back(s.first);
        /*if (group_scores.count(index_group[s.first]) == 1) {
            group_scores[index_group[s.first]] = std::min(s.second, group_scores[index_group[s.first]]);
        }
        else {
            group_scores[index_group[s.first]] = s.second;
        }*/

    }

    for (pair<const int, vector<int> >& s : scoring_groups) {
        for (size_t i = 0; i < s.second.size(); ++i) {
            for (size_t j = 0; j < i; ++j) {
                group_scores[s.first] -= calculate_similarity(s.second[i], s.second[j]);
            }
        }
    }

    /*for (pair<const int, double>& s : group_scores) {
        s.second = 1.0/s.second;
    }*/

    scores.insert(scores.end(), group_scores.begin(), group_scores.end());
    std::sort(scores.begin(), scores.end(), [](const cloud_idx_score& s1, const cloud_idx_score& s2) {
        return s1.second > s2.second; // find max elements!
    });

    scores.resize(nbr_results);
}

template <typename Point, size_t K>
template <class Archive>
void grouped_vocabulary_tree<Point, K>::load(Archive& archive)
{
    super::load(archive);
    archive(index_group, nbr_points, nbr_groups);
}

template <typename Point, size_t K>
template <class Archive>
void grouped_vocabulary_tree<Point, K>::save(Archive& archive) const
{
    super::save(archive);
    archive(index_group, nbr_points, nbr_groups);
}
