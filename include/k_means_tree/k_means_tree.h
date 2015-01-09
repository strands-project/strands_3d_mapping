#ifndef K_MEANS_TREE_H
#define K_MEANS_TREE_H

#include <pcl/point_types.h>
#include <stddef.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>

template <typename Point>
struct map_proxy {
    static const size_t rows = std::extent<decltype(Point::histogram)>::value;
    using map_type = Eigen::Map<Eigen::Matrix<float, rows, 1> >;
    using const_map_type = Eigen::Map<const Eigen::Matrix<float, rows, 1> >;
};

template <>
struct map_proxy<pcl::PointXYZ>
{
    static const size_t rows = 3;
    using map_type = Eigen::Map<Eigen::Matrix<float, rows, 1> >;
    using const_map_type = Eigen::Map<const Eigen::Matrix<float, rows, 1> >;
};

template <>
struct map_proxy<pcl::PointXYZRGB>
{
    static const size_t rows = 3;
    using map_type = Eigen::Map<Eigen::Matrix<float, rows, 1> >;
    using const_map_type = Eigen::Map<const Eigen::Matrix<float, rows, 1> >;
};

template <>
struct map_proxy<const pcl::PointXYZ>
{
    static const size_t rows = 3;
    using map_type = Eigen::Map<Eigen::Matrix<float, rows, 1> >;
    using const_map_type = Eigen::Map<const Eigen::Matrix<float, rows, 1> >;
};

template <>
struct map_proxy<const pcl::PointXYZRGB>
{
    static const size_t rows = 3;
    using map_type = Eigen::Map<Eigen::Matrix<float, rows, 1> >;
    using const_map_type = Eigen::Map<const Eigen::Matrix<float, rows, 1> >;
};

template <typename Point, size_t K, typename Data = void>
class k_means_tree {
protected:

    using PointT = Point;
    using CloudT = pcl::PointCloud<PointT>;
    using CloudPtrT = typename CloudT::Ptr;
    static const size_t dim = K;
    static const size_t rows = map_proxy<PointT>::rows;
    using data_type = Data;
    using leaf_range = std::pair<int, int>;

public:

    struct node {
        typedef node* ptr_type;
        ptr_type children[dim];
        PointT centroid;
        bool is_leaf;
        leaf_range range;
        float weight;
        node() : is_leaf(false)
        {
            for (ptr_type& n : children) {
               n = NULL;
            }
        }
        virtual ~node()
        {
            for (ptr_type& n : children) {
                delete n;
            }
        }
    };

    struct leaf : public node {
        std::vector<int> inds;
        data_type* data;
        leaf() : node(), data(NULL) { node::is_leaf = true; }
        ~leaf()
        {
            delete data;
        }
    };

protected:

    CloudPtrT cloud;
    node root;
    size_t depth;
    std::vector<leaf*> leaves;

protected:

    //std::vector<size_t> sample_without_replacement(size_t upper) const;
    //std::vector<size_t> sample_with_replacement(size_t upper) const;
    leaf_range assign_nodes(CloudPtrT& subcloud, node** nodes, size_t current_depth, const std::vector<int>& subinds);
    void unfold_nodes(std::vector<node*>& path, node* nodes, const PointT& p);
    void flatten_nodes(CloudPtrT& nodecloud, node* n);
    void flatten_nodes_optimized(CloudPtrT& nodecloud, node* n);
    node* get_next_node(node* n, const PointT& p);

public:

    std::vector<size_t> sample_without_replacement(size_t upper) const;
    std::vector<size_t> sample_with_replacement(size_t upper) const;

    void set_input_cloud(CloudPtrT& new_cloud)
    {
        cloud = CloudPtrT(new CloudT);
        std::vector<int> dummy;
        pcl::removeNaNFromPointCloud(*new_cloud, *cloud, dummy);
        //cloud = new_cloud;
    }

    size_t size() const { return cloud->size(); }
    CloudPtrT get_cloud() { return cloud; }

    void add_points_from_input_cloud();
    leaf* get_leaf_for_point(const PointT& point);
    void get_path_for_point(std::vector<node*>& path, const PointT &point);
    void get_cloud_for_point_at_level(CloudPtrT& nodecloud, const PointT& p, size_t level);
    void get_cloud_for_point_at_level_optimized(CloudPtrT& nodecloud, const PointT& p, size_t level);
    size_t points_in_node(node* n);

    k_means_tree(size_t depth = 5) : depth(depth) {}
    virtual ~k_means_tree() { leaves.clear(); }

};

#include "k_means_tree.hpp"

#endif // K_MEANS_TREE_H
