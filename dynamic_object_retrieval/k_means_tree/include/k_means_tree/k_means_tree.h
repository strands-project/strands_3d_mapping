#ifndef K_MEANS_TREE_H
#define K_MEANS_TREE_H

#include <pcl/point_types.h>
#include <stddef.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>

#include <cereal/types/array.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

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

template <typename Point, size_t K, typename Data = void, int Lp=1>
class k_means_tree {
protected:

    using PointT = Point;
    using CloudT = pcl::PointCloud<PointT>;
    using CloudPtrT = typename CloudT::Ptr;
    static const size_t dim = K;
    static const size_t rows = map_proxy<PointT>::rows;
    static const int desc_norm = Lp;
    using data_type = Data;
    using leaf_range = std::pair<int, int>;

public:

    struct node {
        typedef node* ptr_type;
        ptr_type children[dim];
        PointT centroid;
        bool is_leaf;
        leaf_range range;
        double weight;
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
        template <class Archive>
        void save(Archive& archive) const
        {
            archive(is_leaf);
            archive(range);
            archive(weight);
            archive(centroid.histogram);
            if (is_leaf) {
                const leaf* l = static_cast<const leaf*>(this);
                archive(l->inds);
                archive(*(l->data));
            }
            else {
                for (ptr_type const n : children) {
                    archive(*n); // assuming this can't be zero
                }
            }
        }

        template <class Archive>
        void load(Archive& archive)
        {
            archive(range);
            archive(weight);
            archive(centroid.histogram);
            if (is_leaf) {
                leaf* l = static_cast<leaf*>(this);
                l->data = new data_type;
                archive(l->inds);
                archive(*(l->data));
            }
            else {
                for (ptr_type& n : children) {
                    bool child_is_leaf;
                    archive(child_is_leaf);
                    if (child_is_leaf) {
                        leaf* l = new leaf;
                        n = l;
                    }
                    else {
                        n = new node;
                    }
                    archive(*n); // assuming this can't be zero
                }
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
    size_t inserted_points;

protected:

    leaf_range assign_nodes(CloudPtrT& subcloud, node** nodes, size_t current_depth, const std::vector<int>& subinds);
    void unfold_nodes(std::vector<node*>& path, node* nodes, const PointT& p);
    void unfold_nodes(std::vector<std::pair<node*, int> >& depth_path, node* n, const PointT& p, int current_depth);
    void flatten_nodes(CloudPtrT& nodecloud, node* n);
    node* get_next_node(node* n, const PointT& p);
    float norm_func(const PointT& p1, const PointT& p2) const;
    void append_leaves(node* n);
    bool compare_centroids(const Eigen::Matrix<float, rows, dim>& centroids,
                           const Eigen::Matrix<float, rows, dim>& last_centroids) const;
    void assign_extra(CloudPtrT& subcloud, node *n, const std::vector<int>& subinds);
    void assign_mapping_recursive(node* n, std::map<node*, int>& mapping, int& counter);

public:

    std::vector<size_t> sample_without_replacement(size_t upper) const;
    std::vector<size_t> sample_with_replacement(size_t upper) const;

    void set_input_cloud(CloudPtrT& new_cloud)
    {
        cloud = new_cloud;
    }
    void append_cloud(CloudPtrT& extra_cloud, bool store_points = true);

    size_t size() const { return inserted_points; }

    void clear()
    {
        for (leaf* l : leaves) {
            l->inds.clear();
        }
        inserted_points = 0;
    }

    CloudPtrT get_cloud() { return cloud; }

    void add_points_from_input_cloud();
    leaf* get_leaf_for_point(const PointT& point);
    void get_path_for_point(std::vector<node*>& path, const PointT &point);
    void get_path_for_point(std::vector<std::pair<node*, int> >& depth_path, const PointT& point);
    void get_cloud_for_point_at_level(CloudPtrT& nodecloud, const PointT& p, size_t level);
    size_t points_in_node(node* n);
    void get_node_mapping(std::map<node*, int>& mapping);
    double get_mean_leaf_points();
    /*
    template <class Archive> void save(Archive& archive) const;
    template <class Archive> void load(Archive& archive);
    */
    template <class Archive>
    void save(Archive& archive) const
    {
        archive(depth);
        archive(inserted_points);
        archive(root);
    }

    template <class Archive>
    void load(Archive& archive)
    {
        std::cout << "Reading depth" << std::endl;
        archive(depth);
        std::cout << "Reading points" << std::endl;
        archive(inserted_points); // this will not work for older file types
        std::cout << "Reading tree from root" << std::endl;
        archive(root.is_leaf);
        archive(root);
        std::cout << "Setting up the leaves vector" << std::endl;
        append_leaves(&root);
        std::cout << "Finished loading k_means_tree" << std::endl;
    }

    k_means_tree(size_t depth = 5) : depth(depth), inserted_points(0) {}
    virtual ~k_means_tree() { leaves.clear(); }

};

#ifndef VT_PRECOMPILE
#include "k_means_tree.hpp"
#endif

#endif // K_MEANS_TREE_H
