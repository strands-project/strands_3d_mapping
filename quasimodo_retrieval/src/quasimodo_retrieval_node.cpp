#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <object_3d_benchmark/benchmark_retrieval.h>

#include <metaroom_xml_parser/load_utilities.h>
#include <dynamic_object_retrieval/definitions.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>
#include <dynamic_object_retrieval/extract_surfel_features.h>

#include <k_means_tree/k_means_tree.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <grouped_vocabulary_tree/grouped_vocabulary_tree.h>

#include "ros/ros.h"
#include "quasimodo_msgs/retrieval_query_result.h"
#include "quasimodo_msgs/retrieval_query.h"
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

template<typename VocabularyT>
class retrieval_node {
public:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;

    boost::filesystem::path vocabulary_path;
    //boost::filesystem::path data_path;
    int32_t number_query;
    string retrieval_output;
    string retrieval_input;

    VocabularyT vt;
    dynamic_object_retrieval::vocabulary_summary summary;

    retrieval_node(const std::string& name)
    {
        ros::NodeHandle pn("~");

        string vocabulary_string;
        pn.param<std::string>("vocabulary_path", vocabulary_string, std::string(""));
        vocabulary_path = boost::filesystem::path(vocabulary_string);
        cout << "Vocabulary path: " << vocabulary_path.string() << endl;

        //string data_string;
        //pn.param<std::string>("data_path", data_string, std::string(""));
        //data_path = boost::filesystem::path(data_string);

        pn.param<int32_t>("number_query", number_query, 10);

        pn.param<std::string>("output", retrieval_output, std::string("retrieval_result"));
        pn.param<std::string>("input", retrieval_input, std::string("/models/query"));

        summary.load(vocabulary_path);

        if (vt.empty()) {
            dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
            vt.set_cache_path(vocabulary_path.string());
            vt.set_min_match_depth(3);
            vt.compute_normalizing_constants();
        }

        pub = n.advertise<quasimodo_msgs::retrieval_query_result>(retrieval_output, 1);
        sub = n.subscribe(retrieval_input, 1, &retrieval_node::run_retrieval, this);
        /*
        vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);
        for (const string& xml : folder_xmls) {
            run_retrieval(xml);
        }
        */
    }

    tuple<vector<cv::Mat>, vector<cv::Mat>, vector<cv::Mat>, vector<int> >
    generate_images_for_object(const CloudT::Ptr& cloud, const Eigen::Matrix3f& K,
        const boost::filesystem::path& sweep_xml,
        const vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& transforms)
    {
        tuple<vector<cv::Mat>, vector<cv::Mat>, vector<cv::Mat>, vector<int> > images;

        int height = 480;
        int width = 640;

        for (int i = 0; i < transforms.size(); ++i) {
            cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
            int sum = 0;
            for (const PointT& p : cloud->points) {
                Eigen::Vector4f q = transforms[i]*p.getVector4fMap();
                if (q(2)/q(3) < 0) {
                    continue;
                }
                Eigen::Vector3f r = K*q.head<3>();
                int x = int(r(0)/r(2));
                int y = int(r(1)/r(2));
                if (x >= width || x < 0 || y >= height || y < 0) {
                    continue;
                }
                mask.at<uint8_t>(y, x) = 255;
                ++sum;
            }
            if (sum < 100) {
                continue;
            }

            int erosion_size = 4;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                       cv::Size(2*erosion_size + 1, 2*erosion_size+1),
                                       cv::Point(erosion_size, erosion_size));
            cv::dilate(mask, mask, element);
            cv::erode(mask, mask, element);


            get<0>(images).push_back(mask);
            get<1>(images).push_back(benchmark_retrieval::sweep_get_rgb_at(sweep_xml, i));
            get<2>(images).push_back(benchmark_retrieval::sweep_get_depth_at(sweep_xml, i));
            get<3>(images).push_back(i);

            /*
            cv::imshow("Mask", get<0>(images).back());
            cv::imshow("Rgb", get<1>(images).back());
            cv::imshow("Depth", get<2>(images).back());
            cv::waitKey();
            */
        }

        // now probably do some dilation on the masks

        return images;
    }

    void run_retrieval(const quasimodo_msgs::retrieval_query::ConstPtr& query_msg) //const string& sweep_xml)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::fromROSMsg(query_msg->cloud, *normal_cloud);
        CloudT::Ptr cloud(new CloudT);
        NormalCloudT::Ptr normals(new NormalCloudT);
        cloud->reserve(normal_cloud->size()); normals->reserve(normal_cloud->size());
        for (const pcl::PointXYZRGBNormal& pn : normal_cloud->points) {
            PointT p; p.getVector3fMap() = pn.getVector3fMap(); p.rgba = pn.rgba;
            NormalT n; n.getNormalVector3fMap() = pn.getNormalVector3fMap();
            cloud->push_back(p); normals->push_back(n);
        }

        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(query_msg->camera);
        cv::Matx33d cvK = cam_model.intrinsicMatrix();
        Eigen::Matrix3f K = Eigen::Map<Eigen::Matrix3d>(cvK.val).cast<float>();
        K << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f, 0.0f, 0.0f, 1.0f;

        HistCloudT::Ptr features(new HistCloudT);
        CloudT::Ptr keypoints(new CloudT);
        dynamic_object_retrieval::compute_features(features, keypoints, cloud, normals, false, true);

        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;
        //auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, refined_query, query_image, query_depth,
        //                                                                   K, number_query, vocabulary_path, summary, false);
        auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, features, number_query, vocabulary_path, summary);
        tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);

        vector<float> scores;
        for (auto s : results.first) {
            scores.push_back(s.second.score);
        }

        vector<vector<Eigen::Matrix4f>, Eigen::aligned_allocator<Eigen::Matrix4f > > initial_poses;
        vector<vector<cv::Mat> > masks(retrieved_clouds.size());
        vector<vector<cv::Mat> > images(retrieved_clouds.size());
        vector<vector<cv::Mat> > depths(retrieved_clouds.size());
        vector<vector<string> > paths(retrieved_clouds.size());
        for (int i = 0; i < retrieved_clouds.size(); ++i) {
            vector<int> inds;
            auto sweep_data = SimpleXMLParser<PointT>::loadRoomFromXML(sweep_paths[i].string(), std::vector<std::string>{"RoomIntermediateCloud"}, false, false);
            vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > sweep_transforms;
            for (tf::StampedTransform& t : sweep_data.vIntermediateRoomCloudTransformsRegistered) {
                Eigen::Affine3d e;
                tf::transformTFToEigen(t, e);
                sweep_transforms.push_back(e.inverse().matrix().cast<float>());
            }
            tie(masks[i], images[i], depths[i], inds) = generate_images_for_object(retrieved_clouds[i], K, sweep_paths[i], sweep_transforms);
            for (int j = 0; j < inds.size(); ++j) {
                paths[i].push_back(sweep_paths[i].string() + " " + to_string(inds[j]));
            }
        }

        //cv::Mat full_query_image = benchmark_retrieval::sweep_get_rgb_at(sweep_xml, scan_index);
        quasimodo_msgs::retrieval_query_result result;
        result.query = *query_msg;
        result.result = construct_msgs(retrieved_clouds, initial_poses, images, depths, masks, paths, scores);
        pub.publish(result);

    }

    void convert_to_img_msg(const cv::Mat& cv_image, sensor_msgs::Image& ros_image)
    {
        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->image = cv_image;
        cv_pub_ptr->encoding = "bgr8";
        ros_image = *cv_pub_ptr->toImageMsg();
    }

    void convert_to_depth_msg(const cv::Mat& cv_image, sensor_msgs::Image& ros_image)
    {
        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->image = cv_image;
        cv_pub_ptr->encoding = "mono16";
        ros_image = *cv_pub_ptr->toImageMsg();
    }

    void convert_to_mask_msg(const cv::Mat& cv_image, sensor_msgs::Image& ros_image)
    {
        cv_bridge::CvImagePtr cv_pub_ptr(new cv_bridge::CvImage);
        cv_pub_ptr->image = cv_image;
        cv_pub_ptr->encoding = "mono8";
        ros_image = *cv_pub_ptr->toImageMsg();
    }

    quasimodo_msgs::retrieval_result construct_msgs(const vector<CloudT::Ptr>& clouds,
                                                    const vector<vector<Eigen::Matrix4f>, Eigen::aligned_allocator<Eigen::Matrix4f > >& initial_poses,
                                                    const vector<vector<cv::Mat> >& images,
                                                    const vector<vector<cv::Mat> >& depths,
                                                    const vector<vector<cv::Mat> >& masks,
                                                    const vector<vector<string> >& paths,
                                                    const vector<float>& scores)
    {
        quasimodo_msgs::retrieval_result res;

        /*
        retrieval_result result
        =======================
        sensor_msgs/PointCloud2[] retrieved_clouds
        geometry_msgs/Pose[][] retrieved_initial_poses
        sensor_msgs/Image[][] retrieved_images
        sensor_msgs/Image[][] retrieved_depths
        sensor_msgs/Image[][] retrieved_masks
        string[][] retrieved_image_paths
        float64[] retrieved_distance_scores
        */

        int number_retrieved = clouds.size();

        res.retrieved_clouds.resize(number_retrieved);
        res.retrieved_initial_poses.resize(number_retrieved);
        res.retrieved_images.resize(number_retrieved);
        res.retrieved_depths.resize(number_retrieved);
        res.retrieved_masks.resize(number_retrieved);
        res.retrieved_image_paths.resize(number_retrieved);
        res.retrieved_distance_scores.resize(number_retrieved);

        for (int i = 0; i < number_retrieved; ++i) {
            pcl::toROSMsg(*clouds[i], res.retrieved_clouds[i]);
            int nbr_images = images[i].size();

            res.retrieved_initial_poses[i].poses.resize(nbr_images);
            res.retrieved_images[i].images.resize(nbr_images);
            res.retrieved_depths[i].images.resize(nbr_images);
            res.retrieved_masks[i].images.resize(nbr_images);
            res.retrieved_image_paths[i].strings.resize(nbr_images);

            for (int j = 0; j < nbr_images; ++j) {
                //res.result.retrieved_initial_poses[i][j] = geometry_msgs::Pose();
                convert_to_img_msg(images[i][j], res.retrieved_images[i].images[j]);
                convert_to_depth_msg(depths[i][j], res.retrieved_depths[i].images[j]);
                convert_to_mask_msg(masks[i][j], res.retrieved_masks[i].images[j]);
                res.retrieved_image_paths[i].strings[j] = paths[i][j];
            }
            res.retrieved_distance_scores[i] = scores[i];
        }

        return res;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quasimodo_retrieval_node");

    retrieval_node<grouped_vocabulary_tree<HistT, 8> > rs(ros::this_node::getName());

    ros::spin();

    return 0;
}
