#include "ros/ros.h"
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <dynamic_object_retrieval/visualize.h>
#include <object_3d_benchmark/benchmark_retrieval.h>
#include "quasimodo_msgs/query_cloud.h"
#include <pcl_ros/point_cloud.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;

template<typename VocabularyT>
class retrieval_server {
public:
    ros::NodeHandle n;
    ros::ServiceServer service;

    boost::filesystem::path vocabulary_path;

    VocabularyT vt;
    dynamic_object_retrieval::vocabulary_summary summary;

    retrieval_server(const std::string& name)
    {
        ros::NodeHandle pn("~");
        string string_path;
        pn.param<std::string>("vocabulary_path", string_path, std::string(""));
        vocabulary_path = boost::filesystem::path(string_path);

        summary.load(vocabulary_path);

        if (vt.empty()) {
            dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
            vt.set_min_match_depth(3);
            vt.compute_normalizing_constants();
        }

        service = n.advertiseService(name, &retrieval_server::service_callback, this);
    }

    bool service_callback(quasimodo_msgs::query_cloud::Request& req,
                          quasimodo_msgs::query_cloud::Response& res)
    {
        using result_type = vector<pair<typename dynamic_object_retrieval::path_result<VocabularyT>::type, typename VocabularyT::result_type> >;

        CloudT::Ptr cloud(new CloudT);
        pcl::fromROSMsg(req.query.cloud, *cloud);

        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(req.query.camera);
        cv::Matx33d cvK = cam_model.intrinsicMatrix();
        Eigen::Matrix3f K = Eigen::Map<Eigen::Matrix3d>(cvK.val).cast<float>();

        result_type retrieved_paths;
        result_type reweighted_paths;
        VocabularyT vt;
        tie(retrieved_paths, reweighted_paths) = dynamic_object_retrieval::query_reweight_vocabulary(vt, cloud, K, 10, vocabulary_path, summary, false);

        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;
        tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(retrieved_paths);

        res.result.retrieved_clouds.resize(retrieved_clouds.size());
        res.result.retrieved_initial_poses.resize(retrieved_clouds.size());
        //res.retrieved_images.resize(retrieved_clouds.size());
        res.result.retrieved_image_paths.resize(retrieved_clouds.size());
        res.result.retrieved_distance_scores.resize(retrieved_clouds.size());

        for (int i = 0; i < retrieved_clouds.size(); ++i) {
            pcl::toROSMsg(*retrieved_clouds[i], res.result.retrieved_clouds[i]);
            //res.retrieved_initial_poses = geometry_msgs::Pose();
            res.result.retrieved_image_paths[i].strings.push_back(sweep_paths[i].string());
            res.result.retrieved_distance_scores[i] = retrieved_paths[i].second.score;
        }

        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_retrieval_service");

    retrieval_server<vocabulary_tree<HistT, 8> > rs(ros::this_node::getName());

    ros::spin();

    return 0;
}
