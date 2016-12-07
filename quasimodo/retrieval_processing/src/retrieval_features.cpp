#include <object_3d_retrieval/supervoxel_segmentation.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/surfel_type.h>
#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include "dynamic_object_retrieval/extract_surfel_features.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/pfhrgb.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <dynamic_object_retrieval/definitions.h>

#include <quasimodo_msgs/transform_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

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

ros::Publisher pub;
ros::ServiceServer service;

/*
void test_compute_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud,
                           SurfelCloudT::Ptr& surfel_cloud, bool do_visualize = false, bool is_query = false)
{
    NormalCloudT::Ptr normals = dynamic_object_retrieval::compute_surfel_normals(surfel_cloud, cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    double saliency_threshold = 0.975;
    double density_threshold_volume = 0.0405;

    //  ISS3D parameters
    double iss_salient_radius_;
    double iss_non_max_radius_;
    double iss_normal_radius_;
    double iss_border_radius_;
    double iss_gamma_21_ (saliency_threshold); // 0.975 orig
    double iss_gamma_32_ (saliency_threshold); // 0.975 orig
    double iss_min_neighbors_ (5);
    int iss_threads_ (3);

    pcl::IndicesPtr indices(new std::vector<int>);

    double volume = dynamic_object_retrieval::compute_cloud_volume_features(cloud);

    if (is_query || volume < density_threshold_volume) {

        // Fill in the model cloud
        double model_resolution = 0.005;

        // Compute model_resolution
        iss_salient_radius_ = 6 * model_resolution;
        iss_non_max_radius_ = 4 * model_resolution;
        iss_normal_radius_ = 4 * model_resolution;
        iss_border_radius_ = 0.5 * model_resolution; // 1

        //
        // Compute keypoints
        //
        pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
        iss_detector.setSearchMethod(tree);
        iss_detector.setSalientRadius(iss_salient_radius_);
        iss_detector.setNonMaxRadius(iss_non_max_radius_);

        iss_detector.setNormalRadius(iss_normal_radius_); // comment these two if not to use border removal
        iss_detector.setBorderRadius(iss_border_radius_); // comment these two if not to use border removal

        iss_detector.setThreshold21(iss_gamma_21_);
        iss_detector.setThreshold32(iss_gamma_32_);
        iss_detector.setMinNeighbors(iss_min_neighbors_);
        iss_detector.setNumberOfThreads(iss_threads_);
        iss_detector.setInputCloud(cloud);
        iss_detector.setNormals(normals);
        iss_detector.compute(*keypoints);

        pcl::KdTreeFLANN<PointT> kdtree; // might be possible to just use the other tree here
        kdtree.setInputCloud(cloud);
        for (const PointT& k : keypoints->points) {
            std::vector<int> ind;
            std::vector<float> dist;
            kdtree.nearestKSearchT(k, 1, ind, dist);
            //cout << "Keypoint threshold: " << k.rgb << endl;
            indices->push_back(ind[0]);
        }
    }
    else {
        pcl::PointCloud<int>::Ptr keypoints_ind(new pcl::PointCloud<int>);
        pcl::UniformSampling<PointT> us_detector;
        us_detector.setRadiusSearch(0.07);
        us_detector.setSearchMethod(tree);
        us_detector.setInputCloud(cloud);
        us_detector.compute(*keypoints_ind);

        for (int ind : keypoints_ind->points) {
            keypoints->push_back(cloud->at(ind));
            indices->push_back(ind);
        }
    }

    // PFHRGB
    pcl::PFHRGBEstimation<PointT, NormalT> se;
    se.setSearchMethod(tree);
    //se.setKSearch(100);
    se.setIndices(indices); //keypoints
    se.setInputCloud(cloud);
    se.setInputNormals(normals);
    se.setRadiusSearch(0.06); //support 0.06 orig, 0.04 still seems too big, takes time

    pcl::PointCloud<pcl::PFHRGBSignature250> pfhrgb_cloud;
    se.compute(pfhrgb_cloud); //descriptors

    const int N = 250;
    features->resize(pfhrgb_cloud.size());
    for (size_t i = 0; i < pfhrgb_cloud.size(); ++i) {
        std::copy(pfhrgb_cloud.at(i).histogram, pfhrgb_cloud.at(i).histogram+N, features->at(i).histogram);
    }

    std::cout << "Number of features: " << pfhrgb_cloud.size() << std::endl;
}
*/

bool features_service(quasimodo_msgs::transform_cloud::Request& req, quasimodo_msgs::transform_cloud::Response& resp)
{
    double threshold = 0.4;

    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    pcl::fromROSMsg(req.cloud, *surfel_cloud);
    cout << "Got service cloud with size: " << surfel_cloud->size() << endl;

    HistCloudT::Ptr desc_cloud(new HistCloudT);
    CloudT::Ptr kp_cloud(new CloudT);

    NormalCloudT::Ptr normals(new NormalCloudT);
    CloudT::Ptr cloud(new CloudT);
    cloud->reserve(surfel_cloud->size());
    normals->reserve(surfel_cloud->size());
    for (const SurfelT& s : surfel_cloud->points) {
        if (s.confidence < threshold) {
            continue;
        }
        PointT p;
        p.getVector3fMap() = s.getVector3fMap();
        p.rgba = s.rgba;
        NormalT n;
        n.getNormalVector3fMap() = s.getNormalVector3fMap();
        cloud->push_back(p);
        normals->push_back(n);
    }
    cout << "Service points after confidence " << threshold << ": " << cloud->size() << endl;

    dynamic_object_retrieval::compute_features(desc_cloud, kp_cloud, cloud, normals);
    pcl::toROSMsg(*desc_cloud, resp.cloud1);
    pcl::toROSMsg(*kp_cloud, resp.cloud2);

    return true;
}

void features_callback(const std_msgs::String::ConstPtr& msg)
{
    cout << "Received callback with path " << msg->data << endl;
    boost::filesystem::path sweep_xml(msg->data);
    boost::filesystem::path surfel_path = sweep_xml.parent_path() / "surfel_map.pcd";

    SurfelCloudT::Ptr surfel_map(new SurfelCloudT);
    pcl::io::loadPCDFile(surfel_path.string(), *surfel_map);

    dynamic_object_retrieval::sweep_convex_segment_cloud_map clouds(sweep_xml.parent_path());
    dynamic_object_retrieval::sweep_convex_feature_map features(sweep_xml.parent_path());
    dynamic_object_retrieval::sweep_convex_keypoint_map keypoints(sweep_xml.parent_path());
    for (auto tup : dynamic_object_retrieval::zip(clouds, features, keypoints)) {
        CloudT::Ptr segment;
        boost::filesystem::path feature_path;
        boost::filesystem::path keypoint_path;
        tie(segment, feature_path, keypoint_path) = tup;

        if (boost::filesystem::exists(feature_path) && boost::filesystem::exists(keypoint_path)) {
            cout << "Features " << feature_path.string() << " already exist, finishing sweep " << msg->data << "..." << endl;
            break;
        }

        HistCloudT::Ptr desc_cloud(new HistCloudT);
        CloudT::Ptr kp_cloud(new CloudT);
        dynamic_object_retrieval::compute_features(desc_cloud, kp_cloud, segment, surfel_map);

        cout << "Saving " << feature_path.string() << " with " << desc_cloud->size() << " number of features. " << endl;

        //test_compute_features(desc_cloud, kp_cloud, segment, surfel_map);
        if (desc_cloud->empty()) {
            // push back one inf point on descriptors and keypoints
            HistT sp;
            for (int i = 0; i < N; ++i) {
                sp.histogram[i] = std::numeric_limits<float>::infinity();
            }
            desc_cloud->push_back(sp);
            PointT p;
            p.x = p.y = p.z = std::numeric_limits<float>::infinity();
            kp_cloud->push_back(p);
        }
        pcl::io::savePCDFileBinary(feature_path.string(), *desc_cloud);
        pcl::io::savePCDFileBinary(keypoint_path.string(), *kp_cloud);
    }

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
    ros::init(argc, argv, "retrieval_features");
    ros::NodeHandle n;

    ros::NodeHandle pn("~");
    //string retrieval_output;
    //pn.param<double>("threshold", threshold, 0.4);
    bool bypass;
    pn.param<bool>("bypass", bypass, 0);
    string input;
    pn.param<string>("input", input, "/segmentation_done");
    string service_name;
    pn.param<string>("service", service_name, "/retrieval_features_service");

    pub = n.advertise<std_msgs::String>("/features_done", 1);
    service = n.advertiseService(service_name, features_service);

    ros::Subscriber sub;
    if (bypass) {
        sub = n.subscribe(input, 1, bypass_callback);
    }
    else {
        sub = n.subscribe(input, 1, features_callback);
    }

    ros::spin();

    return 0;
}
