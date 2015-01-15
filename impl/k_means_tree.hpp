#include "k_means_tree/k_means_tree.h"

#include <pcl/filters/extract_indices.h>

#include <random>
#include <algorithm>

using namespace std;

template <typename Point>
typename map_proxy<Point>::map_type eig(Point& v)
{
    return typename map_proxy<Point>::map_type(v.histogram);
}

template <typename Point>
typename map_proxy<Point>::const_map_type eig(const Point& v)
{
    return typename map_proxy<Point>::const_map_type(v.histogram);
}

template <>
typename map_proxy<pcl::PointXYZ>::map_type eig(pcl::PointXYZ& v)
{
    return v.getVector3fMap();
}

template <>
typename map_proxy<pcl::PointXYZ>::const_map_type eig(const pcl::PointXYZ& v)
{
    return v.getVector3fMap();
}

template <>
typename map_proxy<pcl::PointXYZRGB>::map_type eig(pcl::PointXYZRGB& v)
{
    return v.getVector3fMap();
}

template <>
typename map_proxy<pcl::PointXYZRGB>::const_map_type eig(const pcl::PointXYZRGB& v)
{
    return v.getVector3fMap();
}

template <typename Point, size_t K, typename Data>
float k_means_tree<Point, K, Data>::norm_func(const PointT& p1, const PointT& p2) const
{
    return -eig(p1).dot(eig(p2));
}

template <typename Point, size_t K, typename Data>
void k_means_tree<Point, K, Data>::add_points_from_input_cloud()
{
    vector<int> inds(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        inds[i] = i;
    }
    root.range = assign_nodes(cloud, root.children, 0, inds);
}

template <typename Point, size_t K, typename Data>
vector<size_t> k_means_tree<Point, K, Data>::sample_without_replacement(size_t upper) const
{
    random_device device;
    mt19937 generator(device());

    // Generate a vector with all possible numbers and shuffle it.
    vector<size_t> result;
    for (size_t i = 0; i < upper; ++i) {
        result.push_back(i);
    }
    shuffle(result.begin(), result.end(), generator);

    // Truncate to the requested size.
    result.resize(dim);

    return result;
}

template <typename Point, size_t K, typename Data>
vector<size_t> k_means_tree<Point, K, Data>::sample_with_replacement(size_t upper) const
{
    random_device device;
    mt19937 generator(device());
    uniform_int_distribution<> dis(0, upper-1);

    vector<size_t> result;
    for (size_t i = 0; i < dim; ++i) {
        result.push_back(dis(generator));
        //result.push_back(rand()%upper);
    }

    return result;
}

/*
template <typename Point, size_t K>
typename k_means_tree<Point, K>::map_type k_means_tree<Point, K>::eig(PointT& v) const
{
    return map_type(v.histogram);
}

template <typename Point, size_t K>
typename k_means_tree<Point, K>::const_map_type k_means_tree<Point, K>::eig(const PointT& v) const
{
    return const_map_type(v.histogram);
}
*/

template <typename Point, size_t K, typename Data>
typename k_means_tree<Point, K, Data>::leaf_range k_means_tree<Point, K, Data>::assign_nodes(CloudPtrT& subcloud, node** nodes, size_t current_depth, const vector<int>& subinds)
{
    std::cout << "Now doing level " << current_depth << std::endl;
    std::cout << subcloud->size() << std::endl;

    // do k-means of the points, iteratively call this again?
    PointT centroids[dim];

    // first, pick centroids at random
    vector<size_t> inds = sample_with_replacement(subcloud->size());
    for (size_t i = 0; i < dim; ++i) {
        centroids[i] = subcloud->points[inds[i]];
    }

    std::vector<int> clusters[dim];
    float cluster_distances[dim];
    size_t min_iter = std::max(50, int(subcloud->size()/1000));
    size_t counter = 0;
    while (true) {
        // compute closest centroids
        for (std::vector<int>& c : clusters) {
            c.clear();
        }
        int ind = 0;
        for (const PointT& p : subcloud->points) {
            for (size_t i = 0; i < dim; ++i) {
                cluster_distances[i] = norm_func(centroids[i], p);
            }
            /*auto closest = min_element(centroids, centroids+dim, [&p](const PointT& q1, const PointT& q2) {
                return (eig(p) - eig(q1)).norm() < (eig(p) - eig(q2)).norm();
            });*/
            auto closest = min_element(cluster_distances, cluster_distances+dim);
            clusters[std::distance(cluster_distances, closest)].push_back(ind);
            ++ind;
        }

        if (counter >= min_iter) {
            break;
        }

        // compute new centroids
        for (size_t i = 0; i < dim; ++i) {
            Eigen::Matrix<double, rows, 1> acc;
            acc.setZero();
            size_t nbr = 0;
            for (size_t ind : clusters[i]) {
                acc += eig(subcloud->at(ind)).template cast<double>();
                ++nbr;
            }
            if (nbr == 0) {
                vector<size_t> temp = sample_with_replacement(subcloud->size());
                centroids[i] = subcloud->at(temp.back());
            }
            else {
                acc *= 1.0/double(nbr);
                eig(centroids[i]) = acc.template cast<float>();
            }
        }

        ++counter;
    }

    leaf_range range(cloud->size(), 0);
    for (size_t i = 0; i < dim; ++i) {
        //std::cout << i << " size: " << clusters[i].size() << std::endl;
        if (current_depth == depth || clusters[i].size() <= 1) {
            leaf* l = new leaf;
            l->inds.resize(clusters[i].size());
            for (size_t j = 0; j < clusters[i].size(); ++j) {
                l->inds[j] = subinds[clusters[i][j]];
            }
            l->centroid = centroids[i];
            /*if (clusters[i].empty()) {
                eig(l->centroid).setZeros();
            }*/
            l->range.first = leaves.size();
            l->range.second = leaves.size()+1;
            leaves.push_back(l);
            nodes[i] = l;
            range.first = std::min(range.first, l->range.first);
            range.second = std::max(range.second, l->range.second);
            continue;
        }
        node* n = new node;
        n->centroid = centroids[i];
        CloudPtrT childcloud(new CloudT);
        childcloud->resize(clusters[i].size());
        vector<int> childinds(clusters[i].size());
        for (size_t j = 0; j < clusters[i].size(); ++j) {
            childcloud->at(j) = subcloud->at(clusters[i][j]);
            childinds[j] = subinds[clusters[i][j]];
        }
        leaf_range rangei = assign_nodes(childcloud, n->children, current_depth+1, childinds);
        n->range = rangei;
        nodes[i] = n;
        range.first = std::min(range.first, rangei.first);
        range.second = std::max(range.second, rangei.second);

        /*pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        inliers->indices = clusters[i];
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(subcloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter (*childcloud);*/
    }

    return range;
}

template <typename Point, size_t K, typename Data>
typename k_means_tree<Point, K, Data>::leaf* k_means_tree<Point, K, Data>::get_leaf_for_point(const PointT& point)
{
    vector<node*> temp;
    unfold_nodes(temp, &root, point);
    leaf* l = static_cast<leaf*>(temp.back());
    return l;
}

template <typename Point, size_t K, typename Data>
void k_means_tree<Point, K, Data>::get_path_for_point(vector<node*>& path, const PointT& point)
{
    path.push_back(&root); // this is needed for the distance in vocabulary_tree
    unfold_nodes(path, &root, point);
}

template <typename Point, size_t K, typename Data>
typename k_means_tree<Point, K, Data>::node* k_means_tree<Point, K, Data>::get_next_node(node* n, const PointT& p)
{
    node** closest = std::min_element(n->children, n->children+dim, [this, &p](const node* q1, const node* q2) {
        return norm_func(p, q1->centroid) < norm_func(p, q2->centroid);
        //return (eig(p) - eig(q1->centroid)).norm() < (eig(p) - eig(q2->centroid)).norm();
    });
    return *closest;
}

template <typename Point, size_t K, typename Data>
void k_means_tree<Point, K, Data>::unfold_nodes(vector<node*>& path, node* n, const PointT& p)
{
    if (n->is_leaf) {
        return;
    }
    /*node** closest = std::min_element(n->children, n->children+dim, [&p](const node* q1, const node* q2) {
        return (eig(p) - eig(q1->centroid)).norm() < (eig(p) - eig(q2->centroid)).norm();
    });*/
    node* closest = get_next_node(n, p);
    path.push_back(closest);
    unfold_nodes(path, closest, p);
}

template <typename Point, size_t K, typename Data>
void k_means_tree<Point, K, Data>::flatten_nodes(CloudPtrT& nodecloud, node* n)
{
    if (n->is_leaf) {
        leaf* l = static_cast<leaf*>(n);
        for (size_t ind : l->inds) {
            nodecloud->push_back(cloud->at(ind));
        }
        return;
    }
    for (node* c : n->children) {
        flatten_nodes(nodecloud, c);
    }
}

template <typename Point, size_t K, typename Data>
void k_means_tree<Point, K, Data>::flatten_nodes_optimized(CloudPtrT& nodecloud, node* n)
{
    for (int i = n->range.first; i < n->range.second; ++i) {
        for (int ind : leaves[i]->inds) {
            nodecloud->push_back(cloud->at(ind));
        }
    }
}

template <typename Point, size_t K, typename Data>
void k_means_tree<Point, K, Data>::get_cloud_for_point_at_level(CloudPtrT& nodecloud, const PointT& p, size_t level)
{
    vector<node*> path;
    unfold_nodes(path, &root, p);
    if (level >= path.size()) {
        return;
    }
    node* n = path[level];
    flatten_nodes(nodecloud, n);
}

template <typename Point, size_t K, typename Data>
void k_means_tree<Point, K, Data>::get_cloud_for_point_at_level_optimized(CloudPtrT& nodecloud, const PointT& p, size_t level)
{
    vector<node*> path;
    unfold_nodes(path, &root, p);
    if (level >= path.size()) {
        return;
    }
    node* n = path[level];
    flatten_nodes_optimized(nodecloud, n);
}

template <typename Point, size_t K, typename Data>
size_t k_means_tree<Point, K, Data>::points_in_node(node* n)
{
    if (n == NULL) {
        return 0;
    }
    size_t nbr_points = 0;
    for (int i = n->range.first; i < n->range.second; ++i) {
        i += leaves[i]->inds.size(); // could use a distance transform for this instead
    }
    return nbr_points;
}

template <typename Point, size_t K, typename Data>
void k_means_tree<Point, K, Data>::append_leaves(node* n)
{
    if (n->is_leaf) {
        leaves.push_back(static_cast<leaf*>(n));
        return;
    }
    for (node* c : n->children) {
        append_leaves(c);
    }
}

template <typename Point, size_t K, typename Data>
template <class Archive>
void k_means_tree<Point, K, Data>::save(Archive& archive) const
{
    archive(depth);
    archive(root);
}

template <typename Point, size_t K, typename Data>
template <class Archive>
void k_means_tree<Point, K, Data>::load(Archive& archive)
{
    archive(depth);
    archive(root);
    append_leaves(&root);
}
