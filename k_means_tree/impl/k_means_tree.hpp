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

template <typename Point, size_t K, typename Data, int Lp>
float k_means_tree<Point, K, Data, Lp>::norm_func(const PointT& p1, const PointT& p2) const
{
    //return (eig(p1)-eig(p2)).squaredNorm();//norm();
    return (eig(p1)-eig(p2)).array().abs().sum();
    //return (eig(p1)-eig(p2)).template lpNorm<desc_norm>();
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::add_points_from_input_cloud()
{
    vector<int> inds(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        inds[i] = i;
    }
    root.range = assign_nodes(cloud, root.children, 0, inds);
    inserted_points = cloud->size();
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::append_cloud(CloudPtrT& extra_cloud, bool store_points)
{
    vector<int> inds(extra_cloud->size());
    for (size_t i = 0; i < extra_cloud->size(); ++i) {
        inds[i] = inserted_points + i;
    }
    if (store_points) {
        cloud->insert(cloud->end(), extra_cloud->begin(), extra_cloud->end());
    }
    assign_extra(extra_cloud, &root, inds);
    inserted_points += extra_cloud->size();
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::assign_extra(CloudPtrT& subcloud, node* n, const vector<int>& subinds)
{
    //std::cout << subcloud->size() << std::endl;

    Eigen::Matrix<float, rows, dim> centroids;
    Eigen::Matrix<float, 1, dim> distances;
    for (size_t i = 0; i < dim; ++i) {
        centroids.col(i) = eig(n->children[i]->centroid);
    }

    std::vector<int> clusters[dim];

    PointT p;
    for (int ind = 0; ind < subcloud->size(); ++ind) {
        p = subcloud->at(ind);
        int closest;
        //distances = eig(p).transpose()*centroids;
        distances = (centroids.colwise()-eig(p)).array().abs().colwise().sum();
        //distances = (centroids.colwise()-eig(p)).colwise().squaredNorm();
        distances.minCoeff(&closest);
        clusters[closest].push_back(ind);
    }

    for (size_t i = 0; i < dim; ++i) {
        node* c = n->children[i];
        if (c->is_leaf) {
            leaf* l = static_cast<leaf*>(c);
            l->inds.reserve(l->inds.size() + clusters[i].size());
            for (size_t j = 0; j < clusters[i].size(); ++j) {
                l->inds.push_back(subinds[clusters[i][j]]);
            }
            continue;
        }
        CloudPtrT childcloud(new CloudT);
        childcloud->resize(clusters[i].size());
        vector<int> childinds(clusters[i].size());
        for (size_t j = 0; j < clusters[i].size(); ++j) {
            childcloud->at(j) = subcloud->at(clusters[i][j]);
            childinds[j] = subinds[clusters[i][j]];
        }
        assign_extra(childcloud, c, childinds);
    }
}

template <typename Point, size_t K, typename Data, int Lp>
vector<size_t> k_means_tree<Point, K, Data, Lp>::sample_without_replacement(size_t upper) const
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

template <typename Point, size_t K, typename Data, int Lp>
vector<size_t> k_means_tree<Point, K, Data, Lp>::sample_with_replacement(size_t upper) const
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

template <typename Point, size_t K, typename Data, int Lp>
bool k_means_tree<Point, K, Data, Lp>::compare_centroids(const Eigen::Matrix<float, rows, dim>& centroids,
                                                     const Eigen::Matrix<float, rows, dim>& last_centroids) const
{
    return centroids.isApprox(last_centroids, 1e-30f);
}

template <typename Point, size_t K, typename Data, int Lp>
typename k_means_tree<Point, K, Data, Lp>::leaf_range k_means_tree<Point, K, Data, Lp>::assign_nodes(CloudPtrT& subcloud, node** nodes, size_t current_depth, const vector<int>& subinds)
{
    //std::cout << "Now doing level " << current_depth << std::endl;
    //std::cout << subcloud->size() << std::endl;

    // do k-means of the points, iteratively call this again?
    Eigen::Matrix<float, rows, dim> centroids;
    Eigen::Matrix<float, rows, dim> last_centroids;
    Eigen::Matrix<float, 1, dim> distances;

    // first, pick centroids at random
    vector<size_t> inds = sample_with_replacement(subcloud->size());
    for (size_t i = 0; i < dim; ++i) {
        centroids.col(i) = eig(subcloud->points[inds[i]]);
    }
    last_centroids.setZero();

    // if there are no more than 1000 points, continue as normal,
    // otherwise decrease to about a 1000 points then double with every iteration
    int skip = std::max(1 << int(log2(double(subcloud->size())/1000.0)), 1);

    std::vector<int> clusters[dim];
    size_t min_iter = std::max(50, int(subcloud->size()/100)); // 50 100
    size_t counter = 0;
    PointT p;
    while (true) {
        // compute closest centroids
        for (std::vector<int>& c : clusters) {
            c.clear();
        }
        int _subcloud_size = subcloud->size();
        for (int ind = 0; ind < _subcloud_size; ind += skip) {
            p = subcloud->at(ind);
            int closest;
            // Wrap these two calls with some nice inlining
            //distances = eig(p).transpose()*centroids;
            distances = (centroids.colwise()-eig(p)).array().abs().colwise().sum();
            //distances = (centroids.colwise()-eig(p)).colwise().squaredNorm();
            distances.minCoeff(&closest);
            clusters[closest].push_back(ind);
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
                centroids.col(i) = eig(subcloud->at(temp.back()));
            }
            else {
                acc *= 1.0/double(nbr);
                centroids.col(i) = acc.template cast<float>();
            }
        }

        skip = std::max(skip/2, 1);
        ++counter;
    }

    leaf_range range(cloud->size(), 0);
    for (size_t i = 0; i < dim; ++i) {
        if (current_depth == depth || clusters[i].size() <= 1) {
            leaf* l = new leaf;
            l->inds.resize(clusters[i].size());
            for (size_t j = 0; j < clusters[i].size(); ++j) {
                l->inds[j] = subinds[clusters[i][j]];
            }
            eig(l->centroid) = centroids.col(i);

            l->range.first = leaves.size();
            l->range.second = leaves.size()+1;
            leaves.push_back(l);
            nodes[i] = l;
            range.first = std::min(range.first, l->range.first);
            range.second = std::max(range.second, l->range.second);
            continue;
        }
        node* n = new node;
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
    }

    return range;
}

template <typename Point, size_t K, typename Data, int Lp>
typename k_means_tree<Point, K, Data, Lp>::leaf* k_means_tree<Point, K, Data, Lp>::get_leaf_for_point(const PointT& point)
{
    vector<node*> temp;
    unfold_nodes(temp, &root, point);
    leaf* l = static_cast<leaf*>(temp.back());
    return l;
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::get_path_for_point(vector<node*>& path, const PointT& point)
{
    path.push_back(&root); // this is needed for the distance in vocabulary_tree
    unfold_nodes(path, &root, point);
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::get_path_for_point(vector<pair<node*, int> >& depth_path, const PointT& point)
{
    depth_path.push_back(make_pair(&root, 0)); // this is needed for the distance in vocabulary_tree
    unfold_nodes(depth_path, &root, point, 1);
}

template <typename Point, size_t K, typename Data, int Lp>
typename k_means_tree<Point, K, Data, Lp>::node* k_means_tree<Point, K, Data, Lp>::get_next_node(node* n, const PointT& p)
{
    double distances[dim];
    for (size_t i = 0; i < dim; ++i) {
        distances[i] = norm_func(p, n->children[i]->centroid);
    }
    return n->children[std::distance(std::begin(distances), std::min_element(distances, distances+dim))];
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::unfold_nodes(vector<node*>& path, node* n, const PointT& p)
{
    if (n->is_leaf) {
        return;
    }
    node* closest = get_next_node(n, p);
    path.push_back(closest);
    unfold_nodes(path, closest, p);
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::unfold_nodes(vector<pair<node*, int> >& depth_path, node* n, const PointT& p, int current_depth)
{
    if (n->is_leaf) {
        return;
    }
    node* closest = get_next_node(n, p);
    depth_path.push_back(make_pair(closest, current_depth));
    unfold_nodes(depth_path, closest, p, current_depth+1);
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::flatten_nodes(CloudPtrT& nodecloud, node* n)
{
    for (int i = n->range.first; i < n->range.second; ++i) {
        for (int ind : leaves[i]->inds) {
            nodecloud->push_back(cloud->at(ind));
        }
    }
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::get_cloud_for_point_at_level(CloudPtrT& nodecloud, const PointT& p, size_t level)
{
    vector<node*> path;
    unfold_nodes(path, &root, p);
    if (level >= path.size()) {
        return;
    }
    node* n = path[level];
    flatten_nodes(nodecloud, n);
}

template <typename Point, size_t K, typename Data, int Lp>
size_t k_means_tree<Point, K, Data, Lp>::points_in_node(node* n)
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

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::append_leaves(node* n)
{
    if (n->is_leaf) {
        leaves.push_back(static_cast<leaf*>(n));
        return;
    }
    for (node* c : n->children) {
        append_leaves(c);
    }
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::assign_mapping_recursive(node* n, map<node*, int>& mapping, int& counter)
{
    mapping[n] = counter;
    ++counter;
    if (n->is_leaf) {
        return;
    }
    for (node* c : n->children) {
        assign_mapping_recursive(c, mapping, counter);
    }
}

template <typename Point, size_t K, typename Data, int Lp>
void k_means_tree<Point, K, Data, Lp>::get_node_mapping(map<node*, int>& mapping)
{
    int counter = 0;
    assign_mapping_recursive(&root, mapping, counter);
}

template <typename Point, size_t K, typename Data, int Lp>
template <class Archive>
void k_means_tree<Point, K, Data, Lp>::save(Archive& archive) const
{
    archive(depth);
    archive(inserted_points);
    archive(root);
}

template <typename Point, size_t K, typename Data, int Lp>
template <class Archive>
void k_means_tree<Point, K, Data, Lp>::load(Archive& archive)
{
    cout << "Reading depth" << endl;
    archive(depth);
    cout << "Reading points" << endl;
    archive(inserted_points); // this will not work for older file types
    cout << "Reading tree from root" << endl;
    archive(root.is_leaf);
    archive(root);
    cout << "Setting up the leaves vector" << endl;
    append_leaves(&root);
    cout << "Finished loading k_means_tree" << endl;
}
