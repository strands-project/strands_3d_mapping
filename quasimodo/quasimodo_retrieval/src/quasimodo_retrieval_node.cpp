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

#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/pfhrgb.h>

#include "ros/ros.h"
#include "quasimodo_msgs/retrieval_query_result.h"
#include "quasimodo_msgs/retrieval_query.h"
#include "quasimodo_msgs/simple_query_cloud.h"
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <quasimodo_retrieval/parametersConfig.h>

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
    ros::ServiceServer service;
    ros::Publisher keypoint_pub;
    ros::Subscriber sub;

    dynamic_reconfigure::Server<quasimodo_retrieval::parametersConfig> server;

    boost::filesystem::path vocabulary_path;
    //boost::filesystem::path data_path;
    int32_t number_query;
    string retrieval_output;
    string retrieval_input;

    VocabularyT vt;
    dynamic_object_retrieval::vocabulary_summary summary;

    double iss_model_resolution; // 0.004
    double pfhrgb_radius_search; // 0.04

    void parameters_callback(quasimodo_retrieval::parametersConfig& config, uint32_t level) {
        iss_model_resolution = config.iss_model_resolution;
        pfhrgb_radius_search = config.pfhrgb_radius_search;
    }

    retrieval_node(const std::string& name)
    {
        ros::NodeHandle pn("~");

        string vocabulary_string;
        pn.param<std::string>("vocabulary_path", vocabulary_string, std::string(""));
        vocabulary_path = boost::filesystem::path(vocabulary_string);
        while (!boost::filesystem::exists(vocabulary_path / "grouped_vocabulary.cereal")) {
            cout << "Vocabulary " << vocabulary_path.string() << " does not exist yet, sleeping for 60s" << endl;
            ros::Duration(60.0).sleep();
        }
        cout << "Vocabulary path: " << vocabulary_path.string() << endl;

        //string data_string;
        //pn.param<std::string>("data_path", data_string, std::string(""));
        //data_path = boost::filesystem::path(data_string);

        pn.param<int32_t>("number_query", number_query, 10);

        pn.param<std::string>("output", retrieval_output, std::string("retrieval_result"));
        pn.param<std::string>("input", retrieval_input, std::string("/models/query"));

        summary.load(vocabulary_path);

        // Load and save data in the relative format
        /*
        //summary.save(vocabulary_path);
        boost::filesystem::path data_path(summary.noise_data_path);
        dynamic_object_retrieval::data_summary data_summary;
        cout << data_path << endl;
        data_summary.load(data_path);
        data_summary.save(data_path);
        exit(0);
        */

        if (vt.empty()) {
            dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
            vt.set_cache_path(vocabulary_path.string());
            vt.set_min_match_depth(3);
            vt.compute_normalizing_constants();
        }


        //dynamic_reconfigure::Server<quasimodo_retrieval::parametersConfig>::CallbackType f;
        //f = boost::bind(&retrieval_node::parameters_callback, this, _1, _2);
        server.setCallback(boost::bind(&retrieval_node::parameters_callback, this, _1, _2));

        pub = n.advertise<quasimodo_msgs::retrieval_query_result>(retrieval_output, 1);
        keypoint_pub = n.advertise<sensor_msgs::PointCloud2>("/models/keypoints", 1);
        sub = n.subscribe(retrieval_input, 10, &retrieval_node::run_retrieval, this);
        service = n.advertiseService("/query_normal_cloud", &retrieval_node::service_callback, this);
        /*
        vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);
        for (const string& xml : folder_xmls) {
            run_retrieval(xml);
        }
        */
    }

    pair<cv::Mat, cv::Mat> sweep_get_rgbd_at(const boost::filesystem::path& sweep_xml, int i)
    {
        stringstream ss;
        ss << "intermediate_cloud" << std::setfill('0') << std::setw(4) << i << ".pcd";
        boost::filesystem::path cloud_path = sweep_xml.parent_path() / ss.str();
        CloudT::Ptr cloud(new CloudT);
        pcl::io::loadPCDFile(cloud_path.string(), *cloud);
        pair<cv::Mat, cv::Mat> images = SimpleXMLParser<PointT>::createRGBandDepthFromPC(cloud);
        return images;
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
            pair<cv::Mat, cv::Mat> intermediate_images = sweep_get_rgbd_at(sweep_xml, i);
            get<1>(images).push_back(intermediate_images.first);
            get<2>(images).push_back(intermediate_images.second);
            //get<1>(images).push_back(benchmark_retrieval::sweep_get_rgb_at(sweep_xml, i));
            //get<2>(images).push_back(benchmark_retrieval::sweep_get_depth_at(sweep_xml, i));
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

    void test_compute_features(HistCloudT::Ptr& features, CloudT::Ptr& keypoints, CloudT::Ptr& cloud,
                               NormalCloudT::Ptr& normals, bool do_visualize = false, bool is_query = false)
    {
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
            double model_resolution = iss_model_resolution;

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

        /*
        if (do_visualize) {
            CloudT::Ptr vis_cloud(new CloudT());
            *vis_cloud += *cloud;
            for (PointT p : keypoints->points) {
                p.r = 255; p.g = 0; p.b = 0;
                vis_cloud->push_back(p);
            }
            visualize_features(vis_cloud);
        }
        */
        // ISS3D

        // PFHRGB
        pcl::PFHRGBEstimation<PointT, NormalT> se;
        se.setSearchMethod(tree);
        //se.setKSearch(100);
        se.setIndices(indices); //keypoints
        se.setInputCloud(cloud);
        se.setInputNormals(normals);
        se.setRadiusSearch(pfhrgb_radius_search); //support 0.06 orig, 0.04 still seems too big, takes time

        pcl::PointCloud<pcl::PFHRGBSignature250> pfhrgb_cloud;
        se.compute(pfhrgb_cloud); //descriptors

        const int N = 250;
        features->resize(pfhrgb_cloud.size());
        for (size_t i = 0; i < pfhrgb_cloud.size(); ++i) {
            std::copy(pfhrgb_cloud.at(i).histogram, pfhrgb_cloud.at(i).histogram+N, features->at(i).histogram);
        }

        std::cout << "Number of features: " << pfhrgb_cloud.size() << std::endl;
    }

    bool service_callback(quasimodo_msgs::simple_query_cloud::Request& req,
                          quasimodo_msgs::simple_query_cloud::Response& res)
    {
        // we assume that req.query_cloud contains a cloud with PointXYZRGBNormal
        cout << "Received query msg..." << endl;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::fromROSMsg(req.query_cloud, *normal_cloud);

        CloudT::Ptr cloud(new CloudT);
        NormalCloudT::Ptr normals(new NormalCloudT);
        cloud->reserve(normal_cloud->size()); normals->reserve(normal_cloud->size());
        for (const pcl::PointXYZRGBNormal& pn : normal_cloud->points) {
            PointT p; p.getVector3fMap() = pn.getVector3fMap(); p.rgba = pn.rgba;
            NormalT n; n.getNormalVector3fMap() = pn.getNormalVector3fMap();
            cloud->push_back(p); normals->push_back(n);
        }

        HistCloudT::Ptr features(new HistCloudT);
        CloudT::Ptr keypoints(new CloudT);
        cout << "Computing features..." << endl;
        test_compute_features(features, keypoints, cloud, normals, false, true);
        cout << "Done computing features..." << endl;

        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;
        // we should retrieve more than we want and only keep the results with valid files on the system
        auto results = dynamic_object_retrieval::query_reweight_vocabulary((vocabulary_tree<HistT, 8>&)vt, features, 200, vocabulary_path, summary);

        // This is just to make sure that we have valid results even when some meta rooms have been deleted
        size_t counter = 0;
        while (counter < number_query) {
            if (!boost::filesystem::exists(results.first[counter].first)) {
                auto iter = std::find_if(results.first.begin()+counter, results.first.end(), [](const pair<boost::filesystem::path, vocabulary_tree<HistT, 8>::result_type>& res) {
                    return boost::filesystem::exists(res.first);
                });
                if (iter == results.first.end()) {
                    break;
                }
                std::swap(results.first[counter], *iter);
            }
            else {
                ++counter;
            }
        }
        results.first.resize(counter);

        tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);

        cout << "Query cloud size: " << cloud->size() << endl;
        for (CloudT::Ptr& c : retrieved_clouds) {
            cout << "Retrieved cloud size: " << c->size() << endl;
        }

        vector<float> scores;
        vector<int> indices;
        for (auto s : results.first) {
            boost::filesystem::path segment_path = s.first;
            string name = segment_path.stem().string();
            size_t last_index = name.find_last_not_of("0123456789");
            int index = stoi(name.substr(last_index + 1));
            indices.push_back(index);
            scores.push_back(s.second.score);
        }

        vector<vector<Eigen::Matrix4f>, Eigen::aligned_allocator<Eigen::Matrix4f > > initial_poses;
        vector<vector<cv::Mat> > masks(retrieved_clouds.size());
        vector<vector<cv::Mat> > images(retrieved_clouds.size());
        vector<vector<cv::Mat> > depths(retrieved_clouds.size());
        vector<vector<string> > paths(retrieved_clouds.size());
        Eigen::Matrix3f K;
        // The new parameters in surfelize_it: 528, 525, 317, 245
        // The old parameters in surfelize_it: 540, 540, 320, 240

        // The old parameters in this node:
        // K << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f, 0.0f, 0.0f, 1.0f;

        // Just for johan to test:
        K << 540.0f, 0.0f, 320.0f, 0.0f, 540.0f, 240.0f, 0.0f, 0.0f, 1.0f;
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

        res.result = construct_msgs(retrieved_clouds, initial_poses, images, depths, masks, paths, scores, indices);

        cout << "Finished retrieval..." << endl;
    }

    void run_retrieval(const quasimodo_msgs::retrieval_query::ConstPtr& query_msg) //const string& sweep_xml)
    {
        cout << "Received query msg..." << endl;

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
        K << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f, 0.0f, 0.0f, 1.0f; // should be removed if Johan returns K?

        HistCloudT::Ptr features(new HistCloudT);
        CloudT::Ptr keypoints(new CloudT);
        //dynamic_object_retrieval::compute_features(features, keypoints, cloud, normals, false, true);
        cout << "Computing features..." << endl;
        test_compute_features(features, keypoints, cloud, normals, false, true);
        cout << "Done computing features..." << endl;

        sensor_msgs::PointCloud2 keypoint_msg;
        pcl::toROSMsg(*keypoints, keypoint_msg);
        keypoint_msg.header.frame_id = "/map";
        keypoint_msg.header.stamp = ros::Time::now();
        keypoint_pub.publish(keypoint_msg);

        vector<CloudT::Ptr> retrieved_clouds;
        vector<boost::filesystem::path> sweep_paths;
        //auto results = dynamic_object_retrieval::query_reweight_vocabulary(vt, refined_query, query_image, query_depth,
        //                                                                   K, number_query, vocabulary_path, summary, false);
        auto results = dynamic_object_retrieval::query_reweight_vocabulary((vocabulary_tree<HistT, 8>&)vt, features, 200, vocabulary_path, summary);

        // This is just to make sure that we have valid results even when some meta rooms have been deleted
        size_t counter = 0;
        while (counter < number_query) {
            if (!boost::filesystem::exists(results.first[counter].first)) {
                auto iter = std::find_if(results.first.begin()+counter, results.first.end(), [](const pair<boost::filesystem::path, vocabulary_tree<HistT, 8>::result_type>& v) {
                    return boost::filesystem::exists(v.first);
                });
                if (iter == results.first.end()) {
                    break;
                }
                std::swap(results.first[counter], *iter);
            }
            else {
                ++counter;
            }
        }
        results.first.resize(counter);

        tie(retrieved_clouds, sweep_paths) = benchmark_retrieval::load_retrieved_clouds(results.first);

        auto data = SimpleXMLParser<PointT>::loadRoomFromXML(sweep_paths[0].string(), std::vector<std::string>{"RoomIntermediateCloud"}, false, false);
        tf::StampedTransform room_transform = data.vIntermediateRoomCloudTransforms[0];
        room_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

        cout << "Query cloud size: " << cloud->size() << endl;
        for (CloudT::Ptr& c : retrieved_clouds) {
            cout << "Retrieved cloud size: " << c->size() << endl;
        }

        vector<float> scores;
        vector<int> indices;
        for (auto s : results.first) {
            boost::filesystem::path segment_path = s.first;
            string name = segment_path.stem().string();
            size_t last_index = name.find_last_not_of("0123456789");
            int index = stoi(name.substr(last_index + 1));
            indices.push_back(index);
            // indices.push_back(s.second.index); // global index in the vocabulary
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

            //Eigen::Affine3d AT;
            //tf::transformTFToEigen(sweep_data.vIntermediateRoomCloudTransforms[0], AT);
            //pcl::transformPointCloud(*retrieved_clouds[i], *retrieved_clouds[i], AT);
        }

        //cv::Mat full_query_image = benchmark_retrieval::sweep_get_rgb_at(sweep_xml, scan_index);
        quasimodo_msgs::retrieval_query_result result;
        result.query = *query_msg;

        tf::transformTFToMsg(room_transform, result.query.room_transform);
        result.result = construct_msgs(retrieved_clouds, initial_poses, images, depths, masks, paths, scores, indices);
        pub.publish(result);

        cout << "Finished retrieval..." << endl;
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
                                                    const vector<float>& scores,
                                                    const vector<int>& indices)
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
        res.segment_indices.resize(number_retrieved);

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
            res.segment_indices[i].ints.push_back(indices[i]);
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
