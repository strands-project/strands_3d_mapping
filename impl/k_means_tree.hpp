#include "k_means_tree/k_means_tree.h"

#include <pcl/filters/extract_indices.h>

#include <random>
#include <algorithm>

using namespace std;

template <typename Point, size_t K>
void k_means_tree<Point, K>::add_points_from_input_cloud()
{
    vector<int> inds(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        inds[i] = i;
    }
    assign_nodes(cloud, root.children, 0, inds);
}

template <typename Point, size_t K>
vector<size_t> k_means_tree<Point, K>::sample_without_replacement(size_t upper) const
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

template <typename Point, size_t K>
vector<size_t> k_means_tree<Point, K>::sample_with_replacement(size_t upper) const
{
    random_device device;
    mt19937 generator(device());
    uniform_int_distribution<> dis(0, upper-1);

    vector<size_t> result;
    for (size_t i = 0; i < dim; ++i) {
        result.push_back(dis(generator));
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

template <typename Point, size_t K>
void k_means_tree<Point, K>::assign_nodes(CloudPtrT& subcloud, node** nodes, size_t current_depth, const vector<int>& subinds)
{
    std::cout << "Now doing level " << current_depth << std::endl;
    std::cout << subcloud->size() << std::endl;

    // do k-means of the points, iteratively call this again?
    Point centroids[dim];

    // first, pick centroids at random
    vector<size_t> inds = sample_with_replacement(subcloud->size());
    for (size_t i = 0; i < dim; ++i) {
        centroids[i] = subcloud->points[inds[i]];
    }

    std::vector<int> clusters[dim];
    size_t min_iter = 10;
    size_t counter = 0;
    while (counter < min_iter) {
        // compute closest centroids
        for (std::vector<int>& c : clusters) {
            c.clear();
        }
        int ind = 0;
        for (const Point& p : subcloud->points) {
            auto closest = min_element(centroids, centroids+dim, [&p](const Point& q1, const Point& q2) {
                return (eig(p) - eig(q1)).norm() < (eig(p) - eig(q2)).norm();
            });
            clusters[std::distance(centroids, closest)].push_back(ind);
            ++ind;
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
                vector<size_t> temp = sample_with_replacement(subcloud->size()); // this is a bit overkill
                centroids[i] = subcloud->at(temp.back());
            }
            else {
                acc *= 1.0/double(nbr);
                eig(centroids[i]) = acc.template cast<float>();
            }
        }

        ++counter;
    }

    for (size_t i = 0; i < dim; ++i) {
        //std::cout << i << " size: " << clusters[i].size() << std::endl;
        if (current_depth == depth || clusters[i].empty()) {
            leaf* l = new leaf;
            l->inds.resize(clusters[i].size());
            for (size_t j = 0; j < clusters[i].size(); ++j) {
                l->inds[j] = subinds[clusters[i][j]];
            }
            l->centroid = centroids[i];
            nodes[i] = l;
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
        assign_nodes(childcloud, n->children, current_depth+1, childinds);
        nodes[i] = n;

        /*pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        inliers->indices = clusters[i];
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(subcloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter (*childcloud);*/
    }
}

template <typename Point, size_t K>
typename k_means_tree<Point, K>::node* k_means_tree<Point, K>::get_leaf_for_point(const PointT& point)
{
    vector<node*> temp;
    unfold_nodes(temp, &root, point);
    return temp.back();
}

template <typename Point, size_t K>
void k_means_tree<Point, K>::get_path_for_point(vector<node*>& path, const PointT& point)
{
    unfold_nodes(path, &root, point);
}

template <typename Point, size_t K>
void k_means_tree<Point, K>::unfold_nodes(vector<node*>& path, node* n, const PointT& p)
{
    if (n->is_leaf) {
        return;
    }
    node** closest = std::min_element(n->children, n->children+dim, [&p](const node* q1, const node* q2) {
        return (eig(p) - eig(q1->centroid)).norm() < (eig(p) - eig(q2->centroid)).norm();
    });
    path.push_back(*closest);
    unfold_nodes(path, *closest, p);
}

template <typename Point, size_t K>
void k_means_tree<Point, K>::flatten_nodes(CloudPtrT& nodecloud, node* n, const PointT& p)
{
    if (n->is_leaf) {
        leaf* l = static_cast<leaf*>(n);
        for (size_t ind : l->inds) {
            nodecloud->push_back(cloud->at(ind));
        }
        return;
    }
    for (node* c : n->children) {
        flatten_nodes(nodecloud, c, p);
    }
}

template <typename Point, size_t K>
void k_means_tree<Point, K>::get_cloud_for_point_at_level(CloudPtrT& nodecloud, const PointT& p, size_t level)
{
    vector<node*> path;
    unfold_nodes(path, &root, p);
    if (level >= path.size()) {
        return;
    }
    node* n = path[level];
    flatten_nodes(nodecloud, n, p);
}
