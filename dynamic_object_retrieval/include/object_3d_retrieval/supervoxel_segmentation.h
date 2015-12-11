#ifndef SUPERVOXEL_SEGMENTATION_H
#define SUPERVOXEL_SEGMENTATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <boost/graph/incremental_components.hpp>
#include <boost/graph/graphviz.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/tuple.hpp>

template <typename Graph>
struct serialized_graph {
    std::vector<std::tuple<size_t, size_t, float> > edges;
    void from_graph(Graph& g)
    {
        using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
        using edge_iterator = typename boost::graph_traits<Graph>::edge_iterator;

        typename boost::property_map<Graph, boost::edge_weight_t>::type edge_id = boost::get(boost::edge_weight, g);
        typename boost::property_map<Graph, boost::vertex_name_t>::type vertex_name = boost::get(boost::vertex_name, g);

        edge_iterator edge_it, edge_end;
        for (std::tie(edge_it, edge_end) = boost::edges(g); edge_it != edge_end; ++edge_it) {
            Vertex u = source(*edge_it, g);
            Vertex v = target(*edge_it, g);
            edges.push_back(std::make_tuple(boost::get(vertex_name, u), boost::get(vertex_name, v), boost::get(edge_id, *edge_it)));
        }
    }
    void to_graph(Graph& g)
    {
        using Edge = typename boost::graph_traits<Graph>::edge_descriptor;
        using edge_weight_property = boost::property<boost::edge_weight_t, float>;

        typename boost::property_map<Graph, boost::vertex_name_t>::type vertex_name = boost::get(boost::vertex_name, g);

        bool flag;
        Edge edge;
        for (auto e : edges) {
            edge_weight_property ew = std::get<2>(e);
            std::tie(edge, flag) = boost::add_edge(std::get<0>(e), std::get<1>(e), ew, g);
            boost::get(vertex_name, std::get<0>(e)) = std::get<0>(e);
            boost::get(vertex_name, std::get<1>(e)) = std::get<1>(e);
        }
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(edges);
    }
};

class supervoxel_segmentation {
public:
    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    using NormalCloudT = pcl::PointCloud<pcl::Normal>;
    using VoxelT = pcl::Supervoxel<PointT>;
    using supervoxel_map = std::map<uint32_t, VoxelT::Ptr>;

    typedef boost::property<boost::edge_weight_t, float> edge_weight_property;
    typedef boost::property<boost::vertex_name_t, size_t> vertex_name_property;
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, vertex_name_property, edge_weight_property>;
    using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
    using VertexIndex = boost::graph_traits<Graph>::vertices_size_type;
    using Edge = boost::graph_traits<Graph>::edge_descriptor;
    using Components = boost::component_index<VertexIndex>;
    //using VertexId = boost::property_map<Graph, VertexIndex>::type; // not sure about this, same thing as VertexIndex?

    float voxel_resolution;
    float cut_threshold;
    float planar_weight;
    bool do_filter;

    // returns a supervoxel clusters with corresponding adjacency graph
    std::tuple<Graph*, std::vector<CloudT::Ptr>, std::vector<CloudT::Ptr>, std::map<size_t, size_t> >
    compute_convex_oversegmentation(CloudT::Ptr& cloud_in, bool visualize = false);
    std::vector<CloudT::Ptr> compute_rgb_clouds(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud_l,
                                                CloudT::Ptr& cloud_in, supervoxel_map& supervoxel_clusters, int max_label);
    Graph* create_supervoxel_graph(std::vector<CloudT::Ptr>& segments, std::vector<CloudT::Ptr>& rgb_segments, CloudT::Ptr& cloud_in);
    void compute_voxel_clouds(std::vector<CloudT::Ptr>& segment_voxels, std::map<uint32_t, size_t>& voxel_inds,
                              supervoxel_map& supervoxels, float voxel_resolution, CloudT::Ptr& original);
    void subsample_cloud(CloudT::Ptr& cloud_out, CloudT::Ptr& cloud_in);
    void preprocess_cloud(CloudT::Ptr& cloud_out, CloudT::Ptr& cloud_in);
    float boundary_convexness(VoxelT::Ptr& first_supervoxel, VoxelT::Ptr& second_supervoxel);
    void connected_components(std::vector<Graph*>& graphs_out, Graph& graph_in);
    void recursive_split(std::vector<Graph*>& graphs_out, Graph& graph_in);
    std::vector<Graph*> color_model_split(std::vector<Graph*>& graphs, std::vector<CloudT::Ptr>& voxel_clouds);
    void graph_cut(std::vector<Graph *>& graphs_out, Graph& graph_in);
    //void visualize_boost_graph(Graph& graph_in);
    float mean_graph_weight(Graph& graph_in);
    size_t graph_size(Graph& graph_in);
    size_t graph_edges_size(Graph& graph_in);
    void visualize_segments(std::vector<CloudT::Ptr>& clouds_out, bool subsample = false);
    void visualize_segments(std::vector<Graph*> graphs, std::vector<CloudT::Ptr>& voxel_clouds);
    void post_merge_convex_segments(std::vector<CloudT::Ptr>& merged_segments, std::map<size_t, size_t>& indices,
                                    std::vector<CloudT::Ptr>& full_segments, Graph& graph_in);
    //void create_full_segment_clouds(std::vector<CloudT::Ptr>& full_segments, std::vector<CloudT::Ptr>& supervoxel_segments,
    //                                std::map<size_t, size_t>& indices, std::vector<CloudT::Ptr>& segments,
    //                                CloudT::Ptr& cloud, std::vector<Graph*>& graphs);
    std::pair<std::vector<CloudT::Ptr>, std::map<size_t, size_t> > merge_connected_clouds(std::vector<CloudT::Ptr>& rgb_segments,
                                                                                          std::vector<Graph*> graphs);
    void save_graph(Graph& g, const std::string& filename) const;
    void load_graph(Graph& g, const std::string& filename) const;

    template <typename GraphT>
    void visualize_boost_graph(GraphT& graph_in)
    {
        std::string filename = "tmp.dot";
        std::string imagefile = "tmp.png";
        std::ofstream file;
        file.open(filename);
        boost::write_graphviz(file, graph_in);
        file.close();
        std::string command = "dot -Tpng " + filename + " > " + imagefile;
        system(command.c_str());
        //command = "gvfs-open " + imagefile;
        //system(command.c_str());

        cv::Mat image;
        image = cv::imread(imagefile, CV_LOAD_IMAGE_COLOR);   // Read the file

        if(!image.data) {
            std::cout <<  "Could not open or find the image" << std::endl;
            exit(0);
        }

        cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
        cv::imshow("Display window", image);                   // Show our image inside it.

        cv::waitKey(0);
    }

    supervoxel_segmentation(float voxel_resolution = 0.02f, float cut_threshold = 0.2f, float planar_weight = 0.3f, bool do_filter = true) :
        voxel_resolution(voxel_resolution), cut_threshold(cut_threshold), planar_weight(planar_weight), do_filter(do_filter) {}
};

#endif // SUPERVOXEL_SEGMENTATION_H
