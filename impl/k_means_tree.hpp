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

// it would actually be possible to have the definition here if they were just inlined
template <>
typename map_proxy<pcl::PointXYZ>::map_type eig(pcl::PointXYZ& v);

template <>
typename map_proxy<pcl::PointXYZ>::const_map_type eig(const pcl::PointXYZ& v);

template <>
typename map_proxy<pcl::PointXYZRGB>::map_type eig(pcl::PointXYZRGB& v);

template <>
typename map_proxy<pcl::PointXYZRGB>::const_map_type eig(const pcl::PointXYZRGB& v);

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
bool k_means_tree<Point, K, Data>::compare_centroids(const Eigen::Matrix<float, rows, dim>& centroids,
                                                     const Eigen::Matrix<float, rows, dim>& last_centroids) const
{
    return centroids.isApprox(last_centroids, 1e-30f);
}

template <typename Point, size_t K, typename Data>
typename k_means_tree<Point, K, Data>::leaf_range k_means_tree<Point, K, Data>::assign_nodes(CloudPtrT& subcloud, node** nodes, size_t current_depth, const vector<int>& subinds)
{
    std::cout << "Now doing level " << current_depth << std::endl;
    std::cout << subcloud->size() << std::endl;

    // do k-means of the points, iteratively call this again?
    //PointT centroids[dim];
    Eigen::Matrix<float, rows, dim> centroids;
    Eigen::Matrix<float, rows, dim> last_centroids;
    Eigen::Matrix<float, 1, dim> distances;

    // first, pick centroids at random
    vector<size_t> inds = sample_with_replacement(subcloud->size());
    for (size_t i = 0; i < dim; ++i) {
        //centroids[i] = subcloud->points[inds[i]];
        centroids.col(i) = eig(subcloud->points[inds[i]]);
    }
    last_centroids.setZero();

    // if there are no more than 1000 points, continue as normal,
    // otherwise decrease to about a 1000 points then double with every iteration
    int skip = std::max(1 << int(log2(double(subcloud->size())/1000.0)), 1);

    std::vector<int> clusters[dim];
    //float cluster_distances[dim];
    size_t min_iter = std::max(50, int(subcloud->size()/100));
    size_t counter = 0;
    PointT p;
    while (true) {
        // compute closest centroids
        for (std::vector<int>& c : clusters) {
            c.clear();
        }
        //int ind = 0;
        //for (const PointT& p : subcloud->points) {
        for (int ind = 0; ind < subcloud->size(); ind += skip) {
            p = subcloud->at(ind);
            /*for (size_t i = 0; i < dim; ++i) {
                cluster_distances[i] = norm_func(centroids[i], p);
            }
            auto closest = min_element(cluster_distances, cluster_distances+dim);
            clusters[std::distance(cluster_distances, closest)].push_back(ind);
            */
            int closest;
            // Wrap these two calls with some nice inlining
            distances = eig(p).transpose()*centroids;
            distances.maxCoeff(&closest);
            clusters[closest].push_back(ind);
            //++ind;
        }

        if (skip == 1 && (counter >= min_iter || compare_centroids(centroids, last_centroids))) {
            break;
        }

        last_centroids = centroids;
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
                //centroids[i] = subcloud->at(temp.back());
                centroids.col(i) = eig(subcloud->at(temp.back()));
            }
            else {
                acc *= 1.0/double(nbr);
                //eig(centroids[i]) = acc.template cast<float>();
                centroids.col(i) = acc.template cast<float>();
            }
        }

        skip = std::max(skip/2, 1);
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
            //l->centroid = centroids[i];
            eig(l->centroid) = centroids.col(i);

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
        //n->centroid = centroids[i];
        eig(n->centroid) = centroids.col(i);
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

#if 0
template <typename Point, size_t K, typename Data>
typename k_means_tree<Point, K, Data>::leaf_range k_means_tree<Point, K, Data>::assign_nodes(CloudPtrT& subcloud, node** nodes, size_t current_depth, const vector<int>& subinds)
{
    std::cout << "Now doing level " << current_depth << std::endl;
    std::cout << subcloud->size() << std::endl;

    // do k-means of the points, iteratively call this again?
    //PointT centroids[dim];
    Eigen::Matrix<float, rows, Eigen::Dynamic> points;
    Eigen::Matrix<float, dim, rows> centroids;
    Eigen::Matrix<float, dim, Eigen::Dynamic> distances;
    Eigen::Matrix<int, 1, Eigen::Dynamic> closest;
    points.resize(rows, subcloud->size());
    distances.resize(dim, subcloud->size());
    closest.resize(subcloud->size());

    // first, pick centroids at random
    vector<size_t> inds = sample_with_replacement(subcloud->size());
    for (size_t i = 0; i < dim; ++i) {
        centroids.row(i) = eig(subcloud->points[inds[i]]);
    }

    size_t counter = 0;
    for (const PointT& p : subcloud->points) {
        points.col(counter) = eig(p);
        ++counter;
    }

    //float cluster_distances[dim];
    size_t min_iter = std::max(50, int(subcloud->size()/100));
    counter = 0;
    while (true) {
        // compute closest centroids
        distances = centroids*points;
        for (size_t i = 0; i < distances.cols(); ++i) {
            distances.col(i).maxCoeff(&closest(i));
        }

        if (counter >= min_iter) {
            break;
        }

        // compute new centroids
        centroids.setZero();
        size_t normalizations[dim] = {};
        for (size_t i = 0; i < closest.cols(); ++i) {
            centroids.row(closest(i)) += points.row(i);
            normalizations[closest(i)] += 1;
        }
        for (size_t i = 0; i < dim; ++i) {
            if (normalizations[i] == 0) {
                vector<size_t> temp = sample_with_replacement(subcloud->size());
                centroids.row(i) = points.row(temp.back());
            }
            else {
                centroids.row(i) *= 1.0f/float(normalizations[i]);
            }
        }

        ++counter;
    }

    std::vector<int> clusters[dim];
    for (int i = 0; i < closest.cols(); ++i) {
        clusters[closest(i)].push_back(i);
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
            eig(l->centroid) = centroids.row(i);
            l->range.first = leaves.size();
            l->range.second = leaves.size()+1;
            leaves.push_back(l);
            nodes[i] = l;
            range.first = std::min(range.first, l->range.first);
            range.second = std::max(range.second, l->range.second);
            continue;
        }
        node* n = new node;
        //n->centroid = centroids[i];
        eig(n->centroid) = centroids.row(i);
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
#endif

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
