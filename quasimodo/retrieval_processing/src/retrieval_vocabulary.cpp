#include <object_3d_retrieval/supervoxel_segmentation.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/surfel_type.h>
#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include "dynamic_object_retrieval/extract_surfel_features.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <dynamic_object_retrieval/definitions.h>

#include <pcl_ros/point_cloud.h>
#include <quasimodo_msgs/index_cloud.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime
#include <sstream> // stringstream
#include <iomanip> // put_time

/**
 * The most reasonable thing to keep track of which metaroom folders that we have traversed
 * would be to create a new json file in the vocabulary folder with a list of all the sweeps
 * that have been added. It should be saved every time a new sweep is reported added.
 * Using this list we can train the vocabulary once sufficiently many sweeps have been added.
 * It is important that the order is kept wrt the metaroom_xml_parser but in theory that
 * should return all of the sweeps in order. The current approach is more risky in that
 * sense. Another possibility is to simply feed the data path as a parameter to this node.
 * That should definitely be doable since the vocabulary path is most often just an extension
 * of the data path. So that would probably be worth trying first.
 */

using namespace std;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using Graph = supervoxel_segmentation::Graph;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;
using VocT = grouped_vocabulary_tree<HistT, 8>;
using IndexT = VocT::index_type;
using AdjacencyT = vector<set<pair<int, int> > >;

// we should retain the vocabulary here if trained
// and check at startup if it is available,
// does that mean we need to provide the querying
// interface as well? maybe
ros::Publisher pub;
ros::ServiceServer service;
int min_training_sweeps;
size_t nbr_added_sweeps;
boost::filesystem::path vocabulary_path;
boost::filesystem::path data_path;
VocT* vt;

set<pair<int, int> > compute_group_adjacencies(const boost::filesystem::path& segment_path)
{
    set<pair<int, int> > adjacencies;

    boost::filesystem::path graph_path = segment_path / "graph.cereal";
    supervoxel_segmentation ss;
    supervoxel_segmentation::Graph g;
    cout << "Loading graph " << graph_path.string() << endl;
    ss.load_graph(g, graph_path.string());

    typename boost::property_map<supervoxel_segmentation::Graph, boost::vertex_name_t>::type vertex_name = boost::get(boost::vertex_name, g);

    // now iterate over all of the adjacencies in the original graph and add them to the set data structure
    using edge_iterator = boost::graph_traits<supervoxel_segmentation::Graph>::edge_iterator;
    edge_iterator edge_it, edge_end;
    for (tie(edge_it, edge_end) = boost::edges(g); edge_it != edge_end; ++edge_it) {
        supervoxel_segmentation::Vertex u = source(*edge_it, g);
        supervoxel_segmentation::Vertex v = target(*edge_it, g);
        supervoxel_segmentation::vertex_name_property from = boost::get(vertex_name, u);
        supervoxel_segmentation::vertex_name_property to = boost::get(vertex_name, v);
        adjacencies.insert(make_pair(from.m_value, to.m_value));
    }

    return adjacencies;
}

/*
void add_to_vocabulary(HistCloudT::Ptr& features, vector<IndexT>& indices, AdjacencyT& adjacencies)
{

}
*/

// we need to split this into 2 parts if we are to keep the same structure
// as before, one for training and one for adding?
void train_vocabulary(const boost::filesystem::path& data_path)
{
    dynamic_object_retrieval::convex_feature_cloud_map segment_features(data_path);
    dynamic_object_retrieval::convex_keypoint_cloud_map segment_keypoints(data_path);
    dynamic_object_retrieval::convex_sweep_index_map sweep_indices(data_path);
    dynamic_object_retrieval::convex_segment_map segment_paths(data_path);
    dynamic_object_retrieval::islast_convex_segment_map last_segments(data_path);

    // this is really the last thing we need to add.

    dynamic_object_retrieval::vocabulary_summary summary;
    //summary.load(vocabulary_path);
    summary.min_segment_features = 30;
    summary.max_training_features = 400000; // 150000;
    summary.max_append_features = 1000000; // 1000000;
    summary.vocabulary_type = "incremental";
    summary.subsegment_type = "convex_segment";
    summary.noise_data_path = data_path.string();
    summary.annotated_data_path = data_path.string();

    size_t min_segment_features = summary.min_segment_features;
    //size_t max_training_features = summary.max_training_features;
    size_t max_append_features = summary.max_append_features;

    vt = new VocT(vocabulary_path.string());
    vt->set_min_match_depth(3);

    HistCloudT::Ptr features(new HistCloudT);
    //CloudT::Ptr centroids(new CloudT);
    AdjacencyT adjacencies;
    vector<IndexT> indices;

    dynamic_object_retrieval::segment_uris uris;

    size_t counter = 0; // index among all segments
    size_t sweep_i; // index of sweep
    size_t last_sweep = 0; // last index of sweep
    size_t sweep_counter = 0; // index within sweep
    bool training = true;
    boost::filesystem::path segment_path;
    // add an iterator with the segment nbr??? maybe not
    // but! add an iterator with the sweep nbr!
    for (auto tup : dynamic_object_retrieval::zip(segment_features, segment_keypoints, sweep_indices, segment_paths, last_segments)) {

        HistCloudT::Ptr features_i;
        CloudT::Ptr keypoints_i;
        boost::filesystem::path last_segment = segment_path;
        bool islast;
        tie(features_i, keypoints_i, sweep_i, segment_path, islast) = tup;

        //cout << "Sweep: " << sweep_i << endl;

        // train on a subset of the provided features
        if (sweep_i != last_sweep) {
            adjacencies.push_back(compute_group_adjacencies(last_segment.parent_path()));

            if (training && last_sweep >= min_training_sweeps - 1) {// features->size() > max_training_features) {
                vt->set_input_cloud(features, indices);
                vt->add_points_from_input_cloud(adjacencies, false);
                features->clear();
                indices.clear();
                adjacencies.clear();
                training = false;
            }

            if (!training && features->size() > max_append_features) {
                cout << "Appending " << features->size() << " points in " << adjacencies.size() << " groups" << endl;
                cout << adjacencies.size() << endl;
                cout << features->size() << endl;
                vt->append_cloud(features, indices, adjacencies, false);
                features->clear();
                indices.clear();
                adjacencies.clear();
            }

            last_sweep = sweep_i;
            sweep_counter = 0;
        }

        // here we insert in the uris
        uris.uris.push_back(string("file://") + segment_path.string());

        if (features_i->size() < min_segment_features) {
            ++counter;
            ++sweep_counter;
            if (sweep_i >= min_training_sweeps && islast) {
                break;
            }
            continue;
        }

        //Eigen::Vector4f point;
        //pcl::compute3DCentroid(*keypoints_i, point);
        //centroids->push_back(PointT());
        //centroids->back().getVector4fMap() = point;
        features->insert(features->end(), features_i->begin(), features_i->end());

        // index of sweep, global index of segment, sweep index of segment
        IndexT index(sweep_i, counter, sweep_counter);
        for (size_t i = 0; i < features_i->size(); ++i) {
            indices.push_back(index);
        }

        ++counter;
        ++sweep_counter;
        if (sweep_i >= min_training_sweeps && islast) {
            break;
        }
    }

    // append the rest
    cout << "Appending " << features->size() << " points in " << adjacencies.size() << " groups" << endl;

    if (features->size() > 0) {
        adjacencies.push_back(compute_group_adjacencies(segment_path.parent_path()));
        vt->append_cloud(features, indices, adjacencies, false);
    }

    summary.nbr_noise_segments = counter; // correct
    summary.nbr_noise_sweeps = sweep_i + 1; // sweep_counter not correct, this is the number of of segments in a sweep
    summary.nbr_annotated_segments = 0;
    summary.nbr_annotated_sweeps = 0;

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    char buffer [80];
    std::strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", std::localtime(&in_time_t));
    summary.last_updated = string(buffer);
    summary.save(vocabulary_path);
    uris.save(vocabulary_path);
    dynamic_object_retrieval::save_vocabulary(*vt, vocabulary_path);
}

// hopefully these sweeps will always be after the previously collected ones
// maybe even iterate through all of this to assert that there are no more?
pair<size_t, size_t> get_offsets_in_data(const boost::filesystem::path& sweep_path)
{
    // iterate through all of the segment paths, don't load anything
    //dynamic_object_retrieval::convex_feature_cloud_map segment_features(data_path);
    //dynamic_object_retrieval::convex_keypoint_cloud_map segment_keypoints(data_path);
    cout << "Computing vocabulary offsets..." << endl;
    dynamic_object_retrieval::convex_sweep_index_map sweep_indices(data_path);
    dynamic_object_retrieval::convex_segment_map segment_paths(data_path);

    size_t counter = 0;
    size_t sweep_i;
    size_t last_sweep = 0;
    boost::filesystem::path segment_path;
    // ok, I will have to use something else here
    // most importantly, I need to keep track of all the previously added
    // number of sweeps and segments
    // the question is how to do that if the node is restarted...
    // maybe add a couple of more fields in the vocabulary_summary?
    for (auto tup : dynamic_object_retrieval::zip(sweep_indices, segment_paths)) {

        boost::tie(sweep_i, segment_path) = tup;

        // skips 0, that's ok
        if (sweep_i != last_sweep) {
            // this should actually work, right?
            //cout << sweep_path.string() << endl;
            //cout << segment_path.parent_path().parent_path().string() << endl;
            if (sweep_path == segment_path.parent_path().parent_path()) {
                cout << "Matching!" << endl;
                return make_pair(sweep_i, counter);
            }
            else {
                //cout << "Not matching!" << endl;
            }
        }

        ++counter;
    }

    cout << "Could not find any corresponding indices!" << endl;
    exit(0);
    return make_pair(0, 0);
}

bool vocabulary_service(quasimodo_msgs::index_cloud::Request& req, quasimodo_msgs::index_cloud::Response& resp)
{
    // let's just create a "ghost" indexing for now, to see if it actually works by comparing to the actual
    // segments_summary.json, maybe even have a separate file for the segment uri:s

    // maybe just skip this before we have actually trained the vocabulary, would make things a lot easier, yes, good idea

    if (vt == NULL) {
        cout << "Vocabulary not trained yet, can not add any object search observations yet, perform more sweeps first..." << endl;
        return false;
    }

    HistCloudT::Ptr features(new HistCloudT);
    pcl::fromROSMsg(req.cloud, *features);
    vector<IndexT> indices;
    // the question is what offsets we should assign these, I think in reality it doesn't matter, possibly the sweep offset might not get set automatically
    // let's just try 0, 0 for now
    //IndexT index(sweep_offset + sweep_i, offset, 0); // we only have one segment within these observations -> sweep_index = 0
    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);
    IndexT index(100000, summary.nbr_noise_segments, 0); // this is a bit weird, did not think we relied on these, we only have one segment within these observations -> sweep_index = 0
    for (size_t i = 0; i < features->size(); ++i) {
        indices.push_back(index);
    }
    AdjacencyT adjacencies;

    if (features->size() > 0) {
        vt->append_cloud(features, indices, adjacencies, false);
    }

    {
        dynamic_object_retrieval::segment_uris uris;
        if (boost::filesystem::exists(vocabulary_path / "segment_uris.json")) {
            cout << "Segment uris file already exists, loading..." << endl;
            uris.load(vocabulary_path);
        }
        else {
            // load all the segments summaries and push them as uris
            dynamic_object_retrieval::data_summary segments_summary;
            segments_summary.load(vocabulary_path.parent_path());
            for (const string& segment_path : segments_summary.index_convex_segment_paths) {
                uris.uris.push_back(string("file://") + segment_path);
            }
        }
        uris.uris.push_back(string("mongodb://") + "world_state" + "/" + "quasimodo" + "/" + req.object_id);
        uris.save(vocabulary_path);
    }

    // This part is new!

    resp.id = summary.nbr_noise_segments;
    summary.nbr_noise_segments += 1;
    summary.nbr_noise_sweeps++;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    char buffer [80];
    std::strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", std::localtime(&in_time_t));
    //std::stringstream ss;
    //ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    //summary.last_updated = ss.str();
    summary.last_updated = string(buffer);
    summary.save(vocabulary_path);
    dynamic_object_retrieval::save_vocabulary(*vt, vocabulary_path);

    return true;
}

void vocabulary_callback(const std_msgs::String::ConstPtr& msg)
{
    cout << "Received callback with path " << msg->data << endl;
    boost::filesystem::path sweep_xml(msg->data);
    boost::filesystem::path segments_path = sweep_xml.parent_path() / "convex_segments";

//    boost::filesystem::path data_path = sweep_xml.parent_path() // room directory
//                                                 .parent_path() // patrol run
//                                                 .parent_path() // date
//                                                 .parent_path() // data root
//                                                 .parent_path(); // actual data root in this data set

    if (vt == NULL) {
        vector<string> room_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string());
        size_t nbr_segmented_sweeps = 0;
        for (const string& xml : room_xmls) {
            if (!boost::filesystem::exists(boost::filesystem::path(xml).parent_path() / "convex_segments")) {
                break;
            }
            ++nbr_segmented_sweeps;
            if (boost::filesystem::path(xml) == sweep_xml) {
                break;
            }
        }
        if (nbr_segmented_sweeps >= min_training_sweeps + 1) {
            train_vocabulary(data_path);
        }
        std_msgs::String done_msg;
        done_msg.data = msg->data;
        pub.publish(done_msg);
        return;
    }

    cout << "After training: " << endl;

    size_t offset; // same thing here, index among segments, can't take this from vocab, instead parse everything?
    size_t sweep_offset;
    // how do we get this to not require anything
    tie(sweep_offset, offset) = get_offsets_in_data(sweep_xml.parent_path());

    size_t counter = 0; // this is important to change as its the index within all segments
    size_t sweep_i; // index among sweeps, how do we get this?

    HistCloudT::Ptr features(new HistCloudT);
    AdjacencyT adjacencies;
    vector<IndexT> indices;

    dynamic_object_retrieval::sweep_convex_segment_cloud_map clouds(sweep_xml.parent_path());
    dynamic_object_retrieval::sweep_convex_feature_cloud_map segment_features(sweep_xml.parent_path());
    dynamic_object_retrieval::sweep_convex_keypoint_cloud_map segment_keypoints(sweep_xml.parent_path());
    dynamic_object_retrieval::sweep_convex_segment_map segment_paths(sweep_xml.parent_path());
    // maybe also iterate over the segment paths?
    vector<string> segment_strings;
    for (auto tup : dynamic_object_retrieval::zip(segment_features, segment_keypoints, clouds, segment_paths)) {
        HistCloudT::Ptr features_i;
        CloudT::Ptr keypoints_i;
        CloudT::Ptr segment;
        boost::filesystem::path segment_path;
        tie(features_i, keypoints_i, segment, segment_path) = tup;
        segment_strings.push_back(segment_path.string());

        features->insert(features->end(), features_i->begin(), features_i->end());

        IndexT index(sweep_offset + sweep_i, offset + counter, counter);
        for (size_t i = 0; i < features_i->size(); ++i) {
            indices.push_back(index);
        }

        ++counter;
    }

    if (features->size() > 0) {
        adjacencies.push_back(compute_group_adjacencies(segments_path));
        vt->append_cloud(features, indices, adjacencies, false);
    }

    {
        dynamic_object_retrieval::segment_uris uris;
        uris.load(vocabulary_path);
        uris.uris.reserve(uris.uris.size() + segment_strings.size());
        for (const string& segment_path : segment_strings) {
            uris.uris.push_back(string("file://") + segment_path);
        }
        uris.save(vocabulary_path);
    }

    // This part is new!
    dynamic_object_retrieval::vocabulary_summary summary;
    summary.load(vocabulary_path);
    summary.nbr_noise_segments += counter;
    summary.nbr_noise_sweeps++;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    char buffer [80];
    std::strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", std::localtime(&in_time_t));
    summary.last_updated = string(buffer);
    summary.save(vocabulary_path);
    dynamic_object_retrieval::save_vocabulary(*vt, vocabulary_path);
    // End of new part!

    std_msgs::String done_msg;
    done_msg.data = msg->data;
    pub.publish(done_msg);
}

void bypass_callback(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String done_msg;
    done_msg.data = msg->data;
    pub.publish(done_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "retrieval_vocabulary");
    ros::NodeHandle n;

    ros::NodeHandle pn("~");
    pn.param<int>("min_training_sweeps", min_training_sweeps, 20);
    string temp_path;
    pn.param<string>("vocabulary_path", temp_path, "~/.semanticMap/vocabulary");
    vocabulary_path = boost::filesystem::path(temp_path);
    pn.param<string>("data_path", temp_path, "~/.semanticMap");
    data_path = boost::filesystem::path(temp_path);

    if (!boost::filesystem::exists(vocabulary_path.parent_path())) {
        cout << "Vocabulary parent path (typically data_path) does not exist, exiting..." << endl;
        return 0;
    }

    boost::filesystem::create_directory(vocabulary_path);
    bool bypass;
    pn.param<bool>("bypass", bypass, 0);

    // add something to check if the vocabulary is already trained
    if (boost::filesystem::exists(vocabulary_path / "grouped_vocabulary.cereal")) {
        vt = new VocT(vocabulary_path.string());
        vt->set_min_match_depth(3);
        dynamic_object_retrieval::load_vocabulary(*vt, vocabulary_path);
        vt->set_cache_path(vocabulary_path.string());
    }
    else {
        vt = NULL;
    }

    pub = n.advertise<std_msgs::String>("/vocabulary_done", 1);
    service = n.advertiseService("/retrieval_vocabulary_service", vocabulary_service);

    ros::Subscriber sub;
    if (bypass) {
        sub = n.subscribe("/features_done", 1, bypass_callback);
    }
    else {
        sub = n.subscribe("/features_done", 1, vocabulary_callback);
    }

    ros::spin();

    return 0;
}

