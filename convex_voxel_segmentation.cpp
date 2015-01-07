#include "convex_voxel_segmentation.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <utility>
#include <set>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <vtkPolyLine.h>

#include <boost/config.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/typeof/typeof.hpp>

#include <boost/graph/graph_utility.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>

typedef boost::adjacency_list <boost::vecS, boost::vecS, boost::undirectedS> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertices_size_type VertexIndex;
typedef boost::component_index<VertexIndex> Components;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::property<boost::edge_weight_t, float> edge_weight_property;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, edge_weight_property> weighted_graph;

convex_voxel_segmentation::convex_voxel_segmentation(bool display_segmentation) : display_segmentation(display_segmentation), voxel_resolution(0.008f)
{
}

void convex_voxel_segmentation::connected_components(std::vector<std::set<size_t> >& groups, const std::set<edge_pair>& edges) const
{
    std::set<edge_pair>::iterator it = std::max_element(edges.begin(), edges.end(), [](const edge_pair& p1, const edge_pair& p2) {
       return std::max(p1.first, p1.second) < std::max(p2.first, p2.second);
    });
    size_t m = std::max(it->first, it->second);
    //const int VERTEX_COUNT = 6;
    Graph graph(m+1);

    std::vector<VertexIndex> rank(num_vertices(graph));
    std::vector<Vertex> parent(num_vertices(graph));

    typedef VertexIndex* Rank;
    typedef Vertex* Parent;

    boost::disjoint_sets<Rank, Parent> ds(&rank[0], &parent[0]);

    initialize_incremental_components(graph, ds);
    incremental_components(graph, ds);

    boost::graph_traits<Graph>::edge_descriptor edge;
    bool flag;

    for (const edge_pair& p : edges) {
        boost::tie(edge, flag) = boost::add_edge(p.first, p.second, graph);
        ds.union_set(p.first, p.second);
    }

    /*std::cout << "An undirected graph:" << std::endl;
    boost::print_graph(graph, boost::get(boost::vertex_index, graph));
    std::cout << std::endl;

    BOOST_FOREACH(Vertex current_vertex, vertices(graph)) {
        std::cout << "representative[" << current_vertex << "] = " <<
                     ds.find_set(current_vertex) << std::endl;
    }

    std::cout << std::endl;*/

    // NOTE: Because we're using vecS for the graph type, we're
    // effectively using identity_property_map for a vertex index map.
    // If we were to use listS instead, the index map would need to be
    // explicitly passed to the component_index constructor.
    Components components(parent.begin(), parent.end());

    groups.resize(components.size());
    // Iterate through the component indices
    BOOST_FOREACH(VertexIndex current_index, components) {
        //std::cout << "component " << current_index << " contains: ";

        // Iterate through the child vertex indices for [current_index]
        BOOST_FOREACH(VertexIndex child_index, components[current_index]) {
            //std::cout << child_index << " ";
            groups[current_index].insert(child_index);
        }

        //std::cout << std::endl;
    }

}

float convex_voxel_segmentation::graph_cut(std::vector<std::set<size_t> >& groups, const std::set<edge_pair>& edges, const std::map<edge_pair, float>& weights) const
{
    std::set<edge_pair>::iterator it = std::max_element(edges.begin(), edges.end(), [](const edge_pair& p1, const edge_pair& p2) {
       return std::max(p1.first, p1.second) < std::max(p2.first, p2.second);
    });
    size_t m = std::max(it->first, it->second);

    weighted_graph g(m);
    Edge edge;
    bool flag;
    for (const edge_pair& p : edges) {
        edge_weight_property e = weights.at(p);
        boost::tie(edge, flag) = boost::add_edge(p.first, p.second, e, g);
    }

    // define a property map, `parities`, that will store a boolean value for each vertex.
    // Vertices that have the same parity after `stoer_wagner_min_cut` runs are on the same side of the min-cut.
    BOOST_AUTO(parities, boost::make_one_bit_color_map(num_vertices(g), boost::get(boost::vertex_index, g)));

    // run the Stoer-Wagner algorithm to obtain the min-cut weight. `parities` is also filled in.
    float w = boost::stoer_wagner_min_cut(g, boost::get(boost::edge_weight, g), boost::parity_map(parities));

    std::cout << "The min-cut weight of G is " << w << ".\n" << std::endl;
    //assert(w == 7);

    groups.resize(2);
    //std::cout << "One set of vertices consists of:" << std::endl;
    size_t i;
    for (i = 0; i < num_vertices(g); ++i) {
        if (boost::get(parities, i)) {
            //std::cout << i << endl;
            groups[0].insert(i);
        }
    }
    //std::cout << endl;

    //std::cout << "The other set of vertices consists of:" << std::endl;
    for (i = 0; i < num_vertices(g); ++i) {
        if (!boost::get(parities, i)) {
            //std::cout << i << std::endl;
            groups[1].insert(i);
        }
    }
    //std::cout << std::endl;

    return w;
}

void convex_voxel_segmentation::addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
                                       pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer,
                                       std::vector<bool>& local_pairs) const
{
    //Iterate through all adjacent points, and add a center point to adjacent point pair
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
    size_t counter = 0;
    for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
    {
        if (!local_pairs[counter]) { // uncomment to visualize connections as well
            ++counter;
            continue;
        }
        std::string linename = supervoxel_name + std::to_string(counter);

        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
        vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
        vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

        vtkUnsignedCharArray *colorT = vtkUnsignedCharArray::New();
        colorT->SetName("Colors");
        colorT->SetNumberOfComponents(3); //4 components cuz of RGBA
        unsigned char red[3] = {255, 0, 0};
        unsigned char white[3] = {255, 255, 255};

        points->InsertNextPoint (supervoxel_center.data);
        points->InsertNextPoint (adjacent_itr->data);

        // Create a polydata to store everything in
        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
        // Add the points to the dataset
        polyData->SetPoints (points);
        //std::cout << "Point size: " << points->GetNumberOfPoints () << std::endl;
        //std::cout << "Remove size: " << local_pairs.size() << std::endl;
        polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
        for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++) {
            polyLine->GetPointIds ()->SetId (i,i);
            if (local_pairs[counter]) {
                colorT->InsertNextTupleValue(red); //color for point1
            }
            else {
                colorT->InsertNextTupleValue(white); //color for point0
            }
        }
        cells->InsertNextCell (polyLine);
        // Add the lines to the dataset
        polyData->SetLines (cells);
        polyData->GetCellData()->SetScalars(colorT);
        viewer->addModelFromPolyData (polyData,linename);
        ++counter;
    }
}

void convex_voxel_segmentation::visualize_segmentation(std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >& supervoxel_clusters,
                            std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                            PointCloudT::Ptr& voxel_centroid_cloud,
                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colored_voxel_cloud,
                            PointNCloudT::Ptr sv_normal_cloud,
                            std::set<edge_pair>& remove_pairs) const
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

    viewer->addPointCloud (colored_voxel_cloud, "colored voxels");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "colored voxels");

    //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
    //viewer->addPointCloudNormals<PointNT> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

    pcl::console::print_highlight ("Getting supervoxel adjacency\n");

    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
    for ( ; label_itr != supervoxel_adjacency.end (); )
    {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        std::vector<bool> local_pairs;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
        {
            pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
            adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
            if (remove_pairs.find(edge_pair(label_itr->first, adjacent_itr->second)) != remove_pairs.end() ||
                    remove_pairs.find(edge_pair(adjacent_itr->second, label_itr->first)) != remove_pairs.end()) {
                local_pairs.push_back(true);
            }
            else {
                local_pairs.push_back(false);
            }
        }
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
        addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer, local_pairs);
        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
}

void convex_voxel_segmentation::segment_pointcloud(PointCloudT::Ptr& cloud,
                        std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >& supervoxel_clusters,
                        std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                        PointCloudT::Ptr& voxel_centroid_cloud,
                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colored_voxel_cloud,
                        PointNCloudT::Ptr& sv_normal_cloud,
                        pcl::PointCloud<pcl::PointXYZL>::Ptr& labels) const
{
    bool use_transform = false;//! pcl::console::find_switch (argc, argv, "--NT");

    float seed_resolution = 0.1f;
    float color_importance = 0.6f;
    float spatial_importance = 0.4f;
    float normal_importance = 1.0f;

    //////////////////////////////  //////////////////////////////
    ////// This is how to use supervoxels
    //////////////////////////////  //////////////////////////////

    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution, use_transform);
    super.setInputCloud (cloud);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);

    pcl::console::print_highlight ("Extracting supervoxels!\n");
    super.extract (supervoxel_clusters);
    pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
    super.refineSupervoxels(3, supervoxel_clusters);

    super.getSupervoxelAdjacency(supervoxel_adjacency);
    voxel_centroid_cloud = super.getVoxelCentroidCloud();
    colored_voxel_cloud = super.getColoredVoxelCloud ();
    sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
    labels = super.getLabeledVoxelCloud();

    //pcl::PointCloud<pcl::PointXYZL>::Ptr other_labels = super.getLabeledCloud();
    //std::cout << "Other label cloud size: " << other_labels->size() << std::endl;
}

std::pair<bool, float> convex_voxel_segmentation::concave_relationship(pcl::Supervoxel<PointT>::Ptr& first_supervoxel,
                                            pcl::Supervoxel<PointT>::Ptr& second_supervoxel) const
{
    PointCloudT::Ptr first_voxels = first_supervoxel->voxels_;
    pcl::PointCloud<pcl::Normal>::Ptr first_normals = first_supervoxel->normals_;
    PointCloudT::Ptr second_voxels = second_supervoxel->voxels_;
    pcl::PointCloud<pcl::Normal>::Ptr second_normals = second_supervoxel->normals_;

    Eigen::Vector3f n1 = first_supervoxel->normal_.getNormalVector3fMap();
    Eigen::Vector3f n2 = second_supervoxel->normal_.getNormalVector3fMap();

    float flat_penalty = -1.0f; // -0.3f
    if (acos(fabs(n1.dot(n2))) < M_PI / 8.0) {
        return std::pair<bool, float>(false, flat_penalty);
    }

    float dist_threshold = 0.03;

    float mindist = std::numeric_limits<float>::infinity();
    float count = 0.0f;
    float mean_prod = 0.0f;
    for (size_t i = 0; i < first_voxels->size(); ++i) {
        PointT point = first_voxels->at(i);
        Eigen::Vector3f normali = first_normals->at(i).getNormalVector3fMap();
        Eigen::Vector3f pe = point.getVector3fMap();
        // find closest point in the other voxel
        PointCloudT::iterator it = std::min_element(second_voxels->begin(), second_voxels->end(),
                                                                [&](const PointT& p1, const PointT& p2)
        {
            return (pe - p1.getVector3fMap()).norm() < (pe - p2.getVector3fMap()).norm();
        });

        float dist = (pe - it->getVector3fMap()).norm();
        if (dist > dist_threshold) {
            continue;
        }
        if (dist < mindist) {
            mindist = dist;
        }
        count += 1.0f;
        size_t ind = std::distance(second_voxels->begin(), it);
        Eigen::Vector3f diff = pe - it->getVector3fMap();
        Eigen::Vector3f normalj = second_normals->at(ind).getNormalVector3fMap();
        if (acos(fabs(normali.dot(normalj))) < M_PI/8.0) {
            mean_prod += flat_penalty;
            continue;
        }
        //std::cout << "Diff norm: " << diff.norm() << std::endl;
        diff.normalize();
        float concaveness = diff.dot(normalj - normali);
        if (concaveness < 0) {
            mean_prod += concaveness;
        }
        else {
            mean_prod += concaveness;
        }
        //std::cout << "WHAT: " << diff.dot(normal.getNormalVector3fMap() - second_normals->at(ind).getNormalVector3fMap()) << std::endl;
    }
    mean_prod /= count;
    //std::cout << "mean_prod: " << mean_prod << std::endl;
    //std::cout << "count: " << count << std::endl;
    bool is_concave = false;
    if (mindist > 0.3*dist_threshold || mean_prod > 0.1 || count < 10) { // more likely to be convex than concave
        is_concave = true;
    }
    return std::pair<bool, float>(is_concave, mean_prod);
}

void convex_voxel_segmentation::construct_new_edge_set(std::set<edge_pair>& new_edges, const std::vector<std::set<size_t> >& groups, const std::set<edge_pair>& old_edges) const
{
    for (edge_pair p : old_edges) {
        for (const std::set<size_t>& s : groups) {
            if (s.count(p.first) != 0 && s.count(p.second) != 0) {
                new_edges.insert(p);
                break;
            }
        }
    }
}

void convex_voxel_segmentation::global_segmentation(std::vector<std::set<size_t> >& groups,
                         const std::set<edge_pair>& edges,
                         const std::map<edge_pair, float>& weights,
                         const std::map<size_t, size_t>& vertex_points) const {
    //std::vector<int> vertice_label(vertice_points.size());
    size_t threshold = 1000;

    std::vector<std::set<size_t> > connected_groups;
    connected_components(connected_groups, edges);

    // filter out groups that don't have enough voxels in them
    std::vector<size_t> connected_points;
    connected_points.resize(connected_groups.size());

    std::vector<std::set<edge_pair> > connected_edges(connected_groups.size());
    for (size_t i = 0; i < connected_groups.size(); ++i) {
        connected_points[i] = 0;
        //std::set<size_t> remove_inds;
        for (std::set<size_t>::iterator it = connected_groups[i].begin(); it != connected_groups[i].end();) {
            if (vertex_points.count(*it) == 0) { // this happens because nbr edges > nbr vertices
                connected_groups[i].erase(it++);
                continue;
            }
            connected_points[i] += vertex_points.at(*it);
            ++it;
        }
        if (connected_points[i] == 0) { // remove this group
            /*connected_groups.erase(connected_groups[i].begin() + i);
            connected_points.erase(connected_points[i].begin() + i);
            connected_edges.erase(connected_edges[i].begin() + i);
            --i;*/
            continue;
        }
        //std::cout << "Vertex points: " << connected_points[i] << std::endl;
        //std::cout << "Number of voxels: " << connected_groups[i].size() << std::endl;
        if (connected_points[i] > threshold) {
            for (edge_pair p : edges) {
                if (std::find(connected_groups[i].begin(), connected_groups[i].end(), p.first) != connected_groups[i].end() ||
                    std::find(connected_groups[i].begin(), connected_groups[i].end(), p.second) != connected_groups[i].end()) {
                    connected_edges[i].insert(p);
                }
            }
        }
    }

    // the first goup is the dumping group
    groups.push_back(std::set<size_t>());
    // build up the new groups
    for (size_t i = 0; i < connected_groups.size(); ++i) {
        if (connected_points[i] <= threshold) {
            // no object
            groups[0].insert(connected_groups[i].begin(), connected_groups[i].end());
            continue;
        }
        if (connected_edges[i].size() < 5) {
            groups.push_back(connected_groups[i]);
            continue;
        }
        std::vector<std::set<size_t> > new_groups;
        float total_w = 0.0f;
        for (edge_pair p : connected_edges[i]) {
            total_w += fabs(weights.at(p));
        }
        total_w /= float(connected_edges[i].size());
        float w = graph_cut(new_groups, connected_edges[i], weights);
        // check the energy of the weights along the graph cut to see if it's valid
        if (w/total_w > -30.0f) { // -20.0
            groups.push_back(connected_groups[i]);
            continue;
        }

        // the important thing here is that we only want the ones that were in the group
        // so remove all the groups that do not contain the original group members
        std::vector<std::set<size_t> > cut_groups;//(new_groups.size());
        for (size_t j = 0; j < new_groups.size(); ++j) {
            std::set<size_t> intersection;
            std::set_intersection(new_groups[j].begin(), new_groups[j].end(), connected_groups[i].begin(), connected_groups[i].end(), std::inserter(intersection, intersection.begin()));
            if (!intersection.empty()) {
                cut_groups.push_back(intersection);
            }
        }
        std::set<edge_pair> new_edges;
        construct_new_edge_set(new_edges, cut_groups, edges);

        std::vector<std::set<size_t> > final_groups;
        connected_components(final_groups, new_edges);

        for (size_t j = 0; j < final_groups.size(); ++j) {
            std::set<size_t> intersection;
            std::set_intersection(final_groups[j].begin(), final_groups[j].end(), connected_groups[i].begin(),
                                  connected_groups[i].end(), std::inserter(intersection, intersection.begin()));
            size_t intersection_points = 0;
            for (size_t s : intersection) {
                intersection_points += vertex_points.at(s);
            }
            if (intersection_points < threshold/10) {
                groups[0].insert(intersection.begin(), intersection.end());
            }
            else if (!intersection.empty()) {
                groups.push_back(intersection);
            }
        }

        //groups.insert(groups.end(), final_groups.begin(), final_groups.end());
    }

    // do iterative graph cuts on all the other groups
}

void convex_voxel_segmentation::visualize(const std::vector<std::set<size_t> >& groups,
                                          pcl::PointCloud<pcl::PointXYZL>::Ptr& labels,
                                          std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >& supervoxel_clusters,
                                          std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                                          PointCloudT::Ptr& voxel_centroid_cloud,
                                          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colored_voxel_cloud,
                                          PointNCloudT::Ptr sv_normal_cloud,
                                          std::set<edge_pair>& remove_pairs) const
{
    int colormap[][3] = {{166,206,227},
    {31,120,180},
    {178,223,138},
    {51,160,44},
    {251,154,153},
    {227,26,28},
    {253,191,111},
    {255,127,0},
    {202,178,214},
    {106,61,154},
    {255,255,153},
    {177,89,40},
    {141,211,199},
    {255,255,179},
    {190,186,218},
    {251,128,114},
    {128,177,211},
    {253,180,98},
    {179,222,105},
    {252,205,229},
    {217,217,217},
    {188,128,189},
    {204,235,197},
    {255,237,111}};
    size_t counter = 0;
    for (const std::set<size_t>& g : groups) {
        unsigned char first_r, first_g, first_b, first_a;
        first_r = first_g = first_b = 0;
        for (size_t m : g) {
            for (size_t i = 0; i < labels->size(); ++i) {
                if (labels->at(i).label == m) {
                    colored_voxel_cloud->at(i).r = colormap[counter%24][0];
                    colored_voxel_cloud->at(i).g = colormap[counter%24][1];
                    colored_voxel_cloud->at(i).b = colormap[counter%24][2];
                    if (counter == 0) {
                        colored_voxel_cloud->at(i).r = 0;
                        colored_voxel_cloud->at(i).g = 0;
                        colored_voxel_cloud->at(i).b = 0;
                    }
                }
            }
        }
        ++counter;
    }
    visualize_segmentation(supervoxel_clusters, supervoxel_adjacency, voxel_centroid_cloud, colored_voxel_cloud, sv_normal_cloud, remove_pairs);
}

void convex_voxel_segmentation::local_convexity_segmentation(std::vector<PointCloudT::Ptr>& result, std::vector<NormalCloudT::Ptr>& segment_normals, PointCloudT::Ptr& cloud) const
{
    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    PointCloudT::Ptr voxel_centroid_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_voxel_cloud;
    PointNCloudT::Ptr sv_normal_cloud;
    pcl::PointCloud<pcl::PointXYZL>::Ptr labels;

    segment_pointcloud(cloud, supervoxel_clusters, supervoxel_adjacency, voxel_centroid_cloud, colored_voxel_cloud, sv_normal_cloud, labels);
    std::map<size_t, size_t> vertex_points;
    for (const auto& kv : supervoxel_clusters) {
        vertex_points[kv.first] = kv.second->voxels_->size();
    }

    using map_iterator = std::multimap<uint32_t,uint32_t>::iterator;
    map_iterator label_itr = supervoxel_adjacency.begin ();
    std::set<edge_pair> remove_pairs;
    std::set<edge_pair> edge_pairs;
    std::map<edge_pair, float> weights;
    for ( ; label_itr != supervoxel_adjacency.end (); )
    {
        map_iterator adjacent_itr = supervoxel_adjacency.equal_range (label_itr->first).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (label_itr->first).second; ++adjacent_itr)
        {
            pcl::Supervoxel<PointT>::Ptr first_supervoxel = supervoxel_clusters.at (label_itr->first);
            pcl::Supervoxel<PointT>::Ptr second_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
            edge_pairs.insert(edge_pair(label_itr->first, adjacent_itr->second));
            edge_pairs.insert(edge_pair(adjacent_itr->second, label_itr->first));

            bool is_concave;
            float weight;
            boost::tie(is_concave, weight) = concave_relationship(first_supervoxel, second_supervoxel);
            weights[edge_pair(label_itr->first, adjacent_itr->second)] = -weight;
            weights[edge_pair(adjacent_itr->second, label_itr->first)] = -weight;
            if (is_concave) {
                remove_pairs.insert(edge_pair(label_itr->first, adjacent_itr->second));
                remove_pairs.insert(edge_pair(adjacent_itr->second, label_itr->first));
            }
        }
        label_itr = supervoxel_adjacency.upper_bound (label_itr->first);
    }

    std::vector<std::set<size_t> > groups;
    global_segmentation(groups, edge_pairs, weights, vertex_points);
    std::cout << "Number of groups: " << groups.size() << std::endl;

    result.resize(groups.size() - 1);
    segment_normals.resize(groups.size() - 1);
    size_t counter = 0;
    for (const std::set<size_t>& g : groups) {
        if (counter == 0) {
            ++counter;
            continue;
        }
        result[counter - 1] = PointCloudT::Ptr(new PointCloudT);
        segment_normals[counter - 1] = NormalCloudT::Ptr(new NormalCloudT);
        for (size_t m : g) {
            *result[counter - 1]  += *supervoxel_clusters.at(m)->voxels_;
            *segment_normals[counter - 1]  += *supervoxel_clusters.at(m)->normals_;
        }
        ++counter;
    }

    if (display_segmentation) {
        visualize(groups, labels, supervoxel_clusters, supervoxel_adjacency, voxel_centroid_cloud, colored_voxel_cloud, sv_normal_cloud, remove_pairs);
    }
}

void convex_voxel_segmentation::segment_objects(std::vector<PointCloudT::Ptr>& result, std::vector<NormalCloudT::Ptr>& segment_normals,
                                                std::vector<PointCloudT::Ptr>& full_result, PointCloudT::Ptr& original) const
{
    if (original->isOrganized()) {
        cout << "Is organized" << endl;
    }
    else {
        cout << "Is disorganized" << endl;
    }
    PointCloudT::Ptr cloud_constrained(new PointCloudT);
    // Create the filtering object
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(original);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 3.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_constrained);

    // outlier removal
    pcl::RadiusOutlierRemoval<PointT> outrem;
    // build the filter
    outrem.setInputCloud(cloud_constrained);
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius(30);
    // apply filter
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    outrem.filter (*cloud_filtered);

    local_convexity_segmentation(result, segment_normals, cloud_filtered);

    pcl::octree::OctreePointCloudSearch<PointT> octree(voxel_resolution);
    octree.setInputCloud(original);
    octree.addPointsFromInputCloud();

    for (PointCloudT::Ptr& cloud : result) {
        full_result.push_back(PointCloudT::Ptr(new PointCloudT));
        for (PointT& p : cloud->points) {
            pcl::PointIndices::Ptr point_idx_data(new pcl::PointIndices());
            if(!octree.voxelSearch(p, point_idx_data->indices)) {
                continue;
            }
            PointCloudT cloud_p;
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(original);
            extract.setIndices(point_idx_data);
            extract.setNegative(false);
            extract.filter(cloud_p);
            *(full_result.back()) += cloud_p;
        }
    }
}
