#include "grouped_vocabulary_tree/grouped_vocabulary_tree.h"

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>

#include <boost/filesystem.hpp>

using namespace std;

template <typename T>
vector<T> apply_permutation_vector(const vector<T>& vec, const vector<int>& p)
{
    vector<T> sorted_vec(p.size());
    std::transform(p.begin(), p.end(), sorted_vec.begin(), [&](int i){ return vec[i]; });
    return sorted_vec;
}

template <typename T, typename Compare>
vector<int> sort_permutation_vector(const vector<T>& vec, Compare compare)
{
    vector<int> p(vec.size());
    std::iota(p.begin(), p.end(), 0);
    std::sort(p.begin(), p.end(), [&](int i, int j){ return compare(vec[i], vec[j]); });
    return p;
}

// we need to add some more argument here that we can request more data from
// actually what we need are cached vocabulary vectors and norms, maybe
// we should store that information with the vocabulary tree, in a folder structure
// this has the advantage that it is independent of what we are representing
// we also need to store the adjacency of subsegments within the sweeps
template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::query_vocabulary(std::vector<result_type>& results, CloudPtrT& query_cloud, size_t nbr_query)
{
    // need a way to get
    // 1. mapping - can get this directly but would need to cache
    // 2. vocabulary_norms, vocabulary_vectors and vocabulary_index_vectors, all for one sweep!
    // 3. the clouds - for what do we actually need them? aa, yes, to get the adjacency
    // 4. folder path for saving and loading - part of initialization

    // the result types should probably be structs to make clear which part is which part is which
    std::vector<result_type> scores;
    top_optimized_similarities(scores, query_cloud, 50*nbr_query); // make initial number of subsegments configurable

    if (mapping.empty()) {
        super::get_node_mapping(mapping);
    }

    // TODO: fix this part
    // these two must be computed at runtime, probably the first time this function is called
    map<int, node*> inverse_mapping;
    for (const pair<node*, int>& u : mapping) {
        inverse_mapping.insert(make_pair(u.second, u.first));
    }

    std::vector<result_type> updated_scores;
    std::vector<vector<int> > updated_indices;
    //vector<index_score> total_scores;
    for (size_t i = 0; i < scores.size(); ++i) {
        vector<vocabulary_vector> vectors;
        set<pair<int, int> > adjacencies;

        load_cached_vocabulary_vectors_for_group(vectors, adjacencies, get<0>(scores[i]));

        vector<int> selected_indices;
        // get<1>(scores[i])) is actually the index within the group!
        double score = super::compute_min_combined_dist(selected_indices, query_cloud, vectors, adjacencies, mapping, inverse_mapping, get<1>(scores[i]));
        updated_scores.push_back(result_type(get<0>(scores[i]), get<1>(scores[i]), score));
        updated_indices.push_back(selected_indices);
    }

    auto p = sort_permutation_vector(updated_scores, [](const result_type& s1, const result_type& s2) {
        return get<2>(s1) < get<2>(s2); // find min elements!
    });

    // the new scores after growing and re-ordering
    results = apply_permutation_vector(updated_scores, p);
    // the subsegment indices within the sweep, not used atm (but we should be able to retrieve this somehow!!)
    vector<vector<int> > oversegment_indices = apply_permutation_vector(updated_indices, p);

    updated_scores.resize(nbr_query);
    oversegment_indices.resize(nbr_query);
}


template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::cache_group_adjacencies(int start_ind, vector<set<pair<int, int> > >& adjacencies)
{
    boost::filesystem::path cache_path = boost::filesystem::path(save_state_path) / "vocabulary_vectors";
    boost::filesystem::create_directory(cache_path);

    for (int i = 0; i < adjacencies.size(); ++i) {
        stringstream ss("group");
        ss << setfill('0') << setw(6) << start_ind + i;
        boost::filesystem::path group_path = cache_path / ss.str();
        boost::filesystem::create_directory(group_path);

        boost::filesystem::path adjacencies_path = group_path / "adjacencies.cereal";
        ofstream outa(adjacencies_path.string());
        {
            cereal::BinaryOutputArchive archive_o(outa);
            archive_o(adjacencies[i]);
        }
        outa.close();
    }
}

// takes a vector which for each groups contains the adjacencies of the elements, stores in gvt location
// we need something more here, like at which index the cloud starts
template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::cache_vocabulary_vectors(int start_ind, CloudPtrT& cloud)
{
    // fuck, we need boost to create folders in the adjacencies
    boost::filesystem::path cache_path = boost::filesystem::path(save_state_path) / "vocabulary_vectors";
    boost::filesystem::create_directory(cache_path);

    int first_group = group_subgroup[start_ind].first;
    int current_group = first_group;
    int current_subgroup = 0;
    vector<vocabulary_vector> current_vectors;
    CloudPtrT current_cloud(new CloudT);

    // iterate through all the groups and create the vocabulary vectors and norms
    for (int i = 0; i < cloud->size(); ++i) {
        pair<int, int> group = group_subgroup[start_ind + i]; // this will have to be index in cleaned up cloud
        if (group.first != current_group) {
            save_cached_vocabulary_vectors_for_group(current_vectors, current_group);
            current_group = group.first;
            current_vectors.clear();
        }

        if (group.second != current_subgroup) {
            if (!current_cloud->empty()) {
                vocabulary_vector vec = super::compute_query_index_vector(current_cloud, mapping);
                current_vectors.push_back(vec);
            }
            current_subgroup = group.second;
            current_cloud->clear();
        }
        current_cloud->push_back(cloud->at(i));
        // now, get all of the elements in the group
        // wait 2 s, what happens if we split up one group in the middle?????? we need to make sure that does never happen
        // if group_subgroup was a vector this would be more natural

    }

    // actually, maybe we should do this at the same time as we add the cloud
    // should we demand that the adjacencies are handed in at the same time????
    // yes let's assume that for now
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::save_cached_vocabulary_vectors_for_group(vector<vocabulary_vector>& vectors, int i)
{
    boost::filesystem::path cache_path = boost::filesystem::path(save_state_path) / "vocabulary_vectors";

    stringstream ss("group");
    ss << setfill('0') << setw(6) << i;
    boost::filesystem::path group_path = cache_path / ss.str();
    boost::filesystem::create_directory(group_path);

    boost::filesystem::path vectors_path = group_path / "vectors.cereal";
    ofstream outv(vectors_path.string());
    {
        cereal::BinaryOutputArchive archive_o(outv);
        archive_o(vectors);
    }
    outv.close();
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::load_cached_vocabulary_vectors_for_group(vector<vocabulary_vector>& vectors,
                                                                                 set<pair<int, int> >& adjacencies, int i)
{
    boost::filesystem::path cache_path = boost::filesystem::path(save_state_path) / "vocabulary_vectors";

    stringstream ss("group");
    ss << setfill('0') << setw(6) << i;
    boost::filesystem::path group_path = cache_path / ss.str();

    boost::filesystem::path vectors_path = group_path / "vectors.cereal";
    ifstream inv(vectors_path.string());
    {
        cereal::BinaryInputArchive archive_i(inv);
        archive_i(vectors);
    }
    inv.close();

    boost::filesystem::path adjacencies_path = group_path / "adjacencies.cereal";
    ifstream ina(adjacencies_path.string());
    {
        cereal::BinaryInputArchive archive_i(ina);
        archive_i(adjacencies);
    }
    ina.close();
}

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
    for (const pair<int, int>& p : temp_indices) {
        // everything with this index pair should have the same label, assume ordered
        if (p != previous_p) {
            ++index_group_ind;
            index_group[index_group_ind] = p.first; // these two really should be vectors instead
            group_subgroup[index_group_ind] = p;
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
    for (const pair<int, int>& p : temp_indices) {
        // everything with this index pair should have the same label, assume ordered
        if (p != previous_p) {
            ++index_group_ind;
            index_group[index_group_ind] = p.first;
            group_subgroup[index_group_ind] = p;
            previous_p = p;
        }
        new_indices[counter] = index_group_ind;
        ++counter;
    }

    nbr_points += counter;
    nbr_groups = index_group_ind + 1;

    super::append_cloud(temp_cloud, new_indices, store_points);
    cache_vocabulary_vectors(0, extra_cloud);
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::add_points_from_input_cloud(bool save_cloud)
{
    super::add_points_from_input_cloud(true);
    cache_vocabulary_vectors(0, super::cloud);
    if (!save_cloud) {
        super::cloud->clear();
    }
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

// is this used anywhere? otherwise leaf_vocabulary_vector is useless
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
void grouped_vocabulary_tree<Point, K>::recursive_create_vocabulary_vector_from_ind(map<node*, double>& vvector, int i, node* n, int current_depth)
{
    if (current_depth >= super::matching_min_depth) {
        map<int, int> source_id_freqs;
        super::source_freqs_for_node(source_id_freqs, n);
        if (source_id_freqs.count(i) > 0) {
            vvector[n] = n->weight*double(source_id_freqs[i]);
        }
    }

    if (n->is_leaf) {
        return;
    }

    for (node* c : n->children) {
        recursive_create_vocabulary_vector_from_ind(vvector, i, c, current_depth + 1);
    }
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::create_vocabulary_vector_from_ind(map<node*, double>& vvector, int i)
{
    // simply go through all of the nodes, get the source freqs and populate vvector if present
    recursive_create_vocabulary_vector_from_ind(vvector, i, &(super::root), 0);
}

template <typename Point, size_t K>
int grouped_vocabulary_tree<Point, K>::get_id_for_group_subgroup(int group_id, int subgroup_id)
{
    pair<int, int> query(group_id, subgroup_id);
    int ind = -1;
    for (const pair<int, pair<int, int> >& p : group_subgroup) {
        if (p.second == query) {
            ind = p.first;
            break;
        }
    }
    if (ind == -1) {
        cout << "Could not find id corresponding to group/subgroup..." << endl;
        exit(0);
    }
    return ind;
}

template <typename Point, size_t K>
double grouped_vocabulary_tree<Point, K>::get_norm_for_group_subgroup(int group_id, int subgroup_id)
{
    int ind = get_id_for_group_subgroup(group_id, subgroup_id);
    return super::db_vector_normalizing_constants[ind];
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::compute_vocabulary_vector_for_group_with_subgroup(map<node*, double>& vvector, int group_id, int subgroup_id)
{
    int ind = get_id_for_group_subgroup(group_id, subgroup_id);
    create_vocabulary_vector_from_ind(vvector, ind);
}

template<typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::get_subgroups_for_group(set<int>& subgroups, int group_id)
{
    for (const pair<int, pair<int, int> >& p : group_subgroup) {
        if (p.second.first == group_id) {
            subgroups.insert(p.second.second);
        }
    }
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::top_optimized_similarities(vector<tuple<int, int, double> >& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    std::map<node*, double> query_id_freqs;
    double qnorm = super::compute_query_vector(query_id_freqs, query_cloud);

    unordered_map<int, double> vocabulary_difference(max_index);

    //double epsilon = 7;
    int skipped = 0;
    for (const pair<node*, double>& v : query_id_freqs) {

        /*if (v.first->weight < epsilon) {
            ++skipped;
            continue;
        }*/

        unordered_map<int, double> node_vocabulary_vector; // includes weights
        compute_node_vocabulary_vector(node_vocabulary_vector, v.first);

        for (const pair<int, double>& u : node_vocabulary_vector) {
            //present_at_lower_levels.insert(u.first);
            vocabulary_difference[u.first] += std::min(u.second, v.second);
        }
    }

    cout << "Skipped " << float(skipped)/float(query_id_freqs.size()) << " nodes" << endl;

    for (pair<const int, double>& u : vocabulary_difference) {
        //cout << super::db_vector_normalizing_constants[u.first] << endl; // this is always 0
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
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::top_l2_similarities(vector<tuple<int, int, double> >& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    std::map<node*, double> query_id_freqs;
    double qnorm = super::compute_query_vector(query_id_freqs, query_cloud);

    unordered_map<int, double> vocabulary_difference(max_index);
    unordered_map<int, double> group_vocabulary_difference;

    for (const pair<node*, double>& v : query_id_freqs) {

        unordered_map<int, double> node_vocabulary_vector; // includes weights
        compute_node_vocabulary_vector(node_vocabulary_vector, v.first);

        unordered_map<int, double> node_group_vocabulary_vector;

        for (const pair<int, double>& u : node_vocabulary_vector) {
            vocabulary_difference[u.first] += std::min(u.second, v.second);
            node_group_vocabulary_vector[group_subgroup[u.first].first] += u.second;
        }

        for (const pair<int, double>& u : node_vocabulary_vector) {
            group_vocabulary_difference[u.first] += std::min(u.second, v.second);
        }
    }

    for (pair<const int, double>& u : group_vocabulary_difference) {
        u.second = qnorm - u.second;
    }

    for (pair<const int, double>& u : vocabulary_difference) {
        double normalization = super::db_vector_normalizing_constants[u.first];
        u.second = std::max(normalization - u.second, group_vocabulary_difference[group_subgroup[u.first].first]);
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
void grouped_vocabulary_tree<Point, K>::group_normalizing_constants_for_node(std::map<int, int>& normalizing_constants, node* n, int current_depth)
{
    if (n->is_leaf) {
        int leaf_ind = n->range.first;
        for (const pair<int, int>& u : super::leaves[leaf_ind]->data->source_id_freqs) {
            normalizing_constants[group_subgroup[u.first].first] += u.second;
        }
        //normalizing_constants = super::leaves[leaf_ind]->data->source_id_freqs;
    }
    else {
        for (node* c : n->children) {
            // here we need one set of normalizing constants for every child to not mess up scores between subtrees
            std::map<int, int> child_normalizing_constants;
            group_normalizing_constants_for_node(child_normalizing_constants, c, current_depth+1);
            for (const pair<int, int>& v : child_normalizing_constants) {
                normalizing_constants[v.first] += v.second;
            }
        }
    }

    n->weight = log(super::N / double(normalizing_constants.size()));

    // this could be further up if we do not want to e.g. calculate weights for upper nodes
    if (current_depth < super::matching_min_depth) {
        return;
    }

    map<int, int> source_id_freqs;
    super::source_freqs_for_node(source_id_freqs, n);
    for (const pair<int, int>& v : source_id_freqs) {
        super::db_vector_normalizing_constants[v.first] += super::pexp(n->weight*v.second); // hope this is inserting 0
    }
}

// do it one image at a time
// need to integrate the weights somehow... could have the accumulated weights in each leaf?
template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::compute_group_normalizing_constants()
{
    // first, update super::N, i.e. the number of scans
    set<int> groups;
    for (const pair<int, pair<int, int> >& u : group_subgroup) {
        groups.insert(u.second.first);
    }
    super::N = groups.size();

    super::db_vector_normalizing_constants.clear();
    std::map<int, int> normalizing_constants;
    group_normalizing_constants_for_node(normalizing_constants, &(super::root), 0);
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

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::save_group_associations(const string& group_file)
{
    ofstream out(group_file, std::ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(group_subgroup);
    }
}

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::load_group_associations(const string& group_file)
{
    ifstream in(group_file, std::ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(group_subgroup);
    }
}
