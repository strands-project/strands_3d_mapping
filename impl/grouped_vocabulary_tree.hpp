#include "grouped_vocabulary_tree/grouped_vocabulary_tree.h"

#include <cereal/types/unordered_map.hpp>

using namespace std;

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

    vector<int> new_indices(indices.size());

    pair<int, int> previous_p = make_pair(-1, -1);
    int index_group_ind = -1;
    int counter = 0;
    for (pair<int, int>& p : temp_indices) {
        // everything with this index pair should have the same label, assume ordered
        if (p != previous_p) {
            ++index_group_ind;
            index_group[index_group_ind] = p.first;
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
    vector<int> new_indices(indices.size());

    pair<int, int> previous_p = make_pair(-1, -1);
    int index_group_ind = nbr_groups - 1;
    int counter = 0;
    for (pair<int, int>& p : temp_indices) {
        // everything with this index pair should have the same label, assume ordered
        if (p != previous_p) {
            ++index_group_ind;
            index_group[index_group_ind] = p.first;
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

/*template <typename Point, size_t K>
double grouped_vocabulary_tree<Point, K>::calculate_similarity(vector<int>& group)
{
    vector<CloudPtrT> clouds;
    for (size_t i = 0; i < group.size(); ++i) {
        clouds.push_back(CloudPtrT(new CloudT));
    }

    // first I guess search for all the indices where i and j are
    for (leaf* l : super::leaves) {
        if (l->data->source_id_freqs.count(i) != 0) {
            size_t points = l->data->source_id_freqs[i];
            for (size_t k = 0; k < points; ++k) {
                cloudi->push_back(l->centroid);
            }
        }
    }

    vector<map<node*, double> > query_id_freqs;
    for (CloudTPtr& c : clouds) {
        query_id_freqs.push_back(map<node*, double>());
        super::compute_query_vector(query_id_freqs.back(), c);
    }

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
}*/

template <typename Point, size_t K>
void grouped_vocabulary_tree<Point, K>::top_grouped_similarities(vector<cloud_idx_score>& scores, CloudPtrT& query_cloud, size_t nbr_results)
{
    vector<cloud_idx_score> sub_scores;

    size_t nbr_compare_groups = size_t(0.01f*float(nbr_groups)); // take the 5% top percentile

    super::test_partial_similarities(sub_scores, query_cloud, nbr_compare_groups);

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
