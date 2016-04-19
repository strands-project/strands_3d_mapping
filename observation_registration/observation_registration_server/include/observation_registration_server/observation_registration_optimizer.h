#ifndef __OBSERVATION_REGISTRATION_OPTIMIZER__
#define __OBSERVATION_REGISTRATION_OPTIMIZER__

#include <ceres/ceres.h>
#include <siftgpu/SiftGPU.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <tf/tf.h>
#include <pcl_ros/transforms.h>

#include "observation_registration_server/sift_wrapper.h"
#include "observation_registration_server/observation_residual.h"

class ObservationRegistrationOptimizer{

public:
    ObservationRegistrationOptimizer(bool verbose=false);
    ~ObservationRegistrationOptimizer();

    template <class PointType>
    tf::Transform registerObservation(const std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& all_clouds_1, const std::vector<tf::StampedTransform>& all_initial_poses_1,
                                              const std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& all_clouds_2, const std::vector<tf::StampedTransform>& all_initial_poses_2,
                                             std::vector<int>& number_of_constraints, tf::StampedTransform origin_1, tf::StampedTransform origin_2){
        using namespace std;
        using namespace cv;
        using namespace ceres;

        // consistency check
        tf::Transform empty; empty.setIdentity();
        if ((all_clouds_1.size() != all_clouds_2.size()) ||
                (all_clouds_1.size() != all_initial_poses_1.size()) ||
                (all_initial_poses_1.size() != all_initial_poses_2.size())){
            ROS_ERROR_STREAM("Unequal number of clouds or poses. Aborting observation registration.");
            return empty;
        }

        tf::Transform registered_pose; registered_pose.setIdentity();
        vector<Mat> vRGBImages_1, vRGBImages_2;
        vector<Mat> vDepthImages_1, vDepthImages_2;

        // set up initial data
        for (auto cloud : all_clouds_1){
            auto image_pair = createRGBandDepthFromCloud(cloud);
            vRGBImages_1.push_back(image_pair.first);
            vDepthImages_1.push_back(image_pair.second);
        }
        for (auto cloud : all_clouds_2){
            auto image_pair = createRGBandDepthFromCloud(cloud);
            vRGBImages_2.push_back(image_pair.first);
            vDepthImages_2.push_back(image_pair.second);
        }

        // extract SIFT keypoints and descriptors
        struct SIFTData{
            int image_number;
            int desc_number;
            vector<float> descriptors;
            vector<SiftGPU::SiftKeypoint> keypoints;
        };

        SIFTWrapper sift_wrapper;
        vector<SIFTData> vSIFTData_1, vSIFTData_2;


        for (size_t i=0; i<vRGBImages_1.size();i++){
            auto image = vRGBImages_1[i];
            SIFTData sift;
            sift_wrapper.extractSIFT(image, sift.desc_number, sift.descriptors, sift.keypoints);
            vSIFTData_1.push_back(sift);
            if (m_bVerbose){
                ROS_INFO_STREAM("Extracted "<<vSIFTData_1[vSIFTData_1.size()-1].keypoints.size()<<" SIFT keypoints for observation 1 -- image "<<i);
            }
        }

        for (size_t i=0; i<vRGBImages_2.size();i++){
            auto image = vRGBImages_2[i];
            SIFTData sift;
            sift_wrapper.extractSIFT(image, sift.desc_number, sift.descriptors, sift.keypoints);
            vSIFTData_2.push_back(sift);
            if (m_bVerbose){
                ROS_INFO_STREAM("Extracted "<<vSIFTData_2[vSIFTData_2.size()-1].keypoints.size()<<" SIFT keypoints for observation 2 -- image "<<i);
            }
        }

//        // transform input clouds using input transforms
        for (size_t i=0; i<all_clouds_1.size();i++){
            pcl_ros::transformPointCloud(*all_clouds_1[i], *all_clouds_1[i],all_initial_poses_1[i]);
            pcl_ros::transformPointCloud(*all_clouds_2[i], *all_clouds_2[i],all_initial_poses_2[i]);

            pcl_ros::transformPointCloud(*all_clouds_1[i], *all_clouds_1[i],origin_1);
            pcl_ros::transformPointCloud(*all_clouds_2[i], *all_clouds_2[i],origin_2);
        }



        // set up constraint problem
        struct ConstraintStructure{
            int image1;
            int image2;
            double depth_threshold;
            vector<pair<PointType, PointType>> correspondences;
            vector<pair<cv::Point2d, cv::Point2d>> correspondences_2d;
        };
        vector<ConstraintStructure> constraints_and_correspondences;

        for (size_t i=0; i<all_clouds_1.size(); i++){
                ConstraintStructure constr;
                constr.image1 = i;
                constr.image2 = i; // from all_clouds_2
                constr.depth_threshold = 5.0;
                constraints_and_correspondences.push_back(constr);
        }

        // get and validate sift matches
        for (ConstraintStructure& constr: constraints_and_correspondences){
            SIFTData image1_sift = vSIFTData_1[constr.image1];
            SIFTData image2_sift = vSIFTData_2[constr.image2];
            vector<pair<SiftGPU::SiftKeypoint,SiftGPU::SiftKeypoint>> matches;

            sift_wrapper.matchSIFT(image1_sift.desc_number, image2_sift.desc_number,
                                   image1_sift.descriptors, image2_sift.descriptors,
                                   image1_sift.keypoints, image2_sift.keypoints,
                                   matches);
            if (m_bVerbose){
                ROS_INFO_STREAM("Resulting number of matches for images "<<constr.image1<<" "<<constr.image2<<" is "<<matches.size());
            }
            constr.correspondences = validateMatches(matches,
                                                     vRGBImages_1[constr.image1], vRGBImages_1[constr.image2],
                    all_clouds_1[constr.image1], all_clouds_2[constr.image2],
                    constr.depth_threshold,
                    constr.correspondences_2d);
            if (m_bVerbose){
                ROS_INFO_STREAM("After validating "<<constr.correspondences.size()<<"  "<<constr.correspondences_2d.size()<<" are left ");
            }
        }


        // set up ceres problem
        struct Camera{
            double quaternion[4] = {1.0,0.0,0.0,0.0};
            double translation[3] = {0.0,0.0,0.0};
            Camera(){};
        };

        // ceres setup
        Problem problem;
        Solver::Options options;
        options.function_tolerance = 1e-30;
        options.parameter_tolerance = 1e-20;
        options.max_num_iterations = 1000;
//        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.num_threads = 8;
        options.num_linear_solver_threads = 8;
        double min_correspondences = 12;
        options.preconditioner_type = ceres::CLUSTER_JACOBI;
        options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        options.visibility_clustering_type = ceres::SINGLE_LINKAGE;


        Camera* cam = new Camera();
        number_of_constraints.resize(all_clouds_1.size());
        number_of_constraints.push_back(0);

        // cost functions
        vector<CostFunction*> rosrallCFs;
        vector<LossFunction*> allLFs;
        int total_constraints = 0;
        double reprojection_threshold_1 = 1.5;
        double reprojection_threshold_2 = 3.5;

        for (auto constr : constraints_and_correspondences){
            if (constr.correspondences.size() < min_correspondences){
                continue;
            }

            for (size_t k=0; k<constr.correspondences.size(); k++){
                auto corresp = constr.correspondences[k];
                auto corresp_2d = constr.correspondences_2d[k];
                tf::Vector3 p1_t(corresp.first.x, corresp.first.y, corresp.first.z);
                tf::Vector3 p2_t(corresp.second.x, corresp.second.y, corresp.second.z);
//                tf::Vector3 p1_t = origin_1 * all_initial_poses_1[constr.image1] * tf::Vector3(corresp.first.x, corresp.first.y, corresp.first.z);
//                tf::Vector3 p2_t = origin_2 * all_initial_poses_2[constr.image2] * tf::Vector3(corresp.second.x, corresp.second.y, corresp.second.z);
                double point_original1[3] = {p1_t.x(), p1_t.y(), p1_t.z()};
                double point_original2[3] = {p2_t.x(), p2_t.y(), p2_t.z()};

                double weight = (corresp.first.getVector3fMap().squaredNorm() + corresp.second.getVector3fMap().squaredNorm())/2;

                ceres::CostFunction *cost_function_proj = ObservationResidual::Create(point_original1, point_original2, weight);
                ceres::LossFunction *loss_function_proj = new ceres::HuberLoss(weight * 0.005);
                problem.AddResidualBlock(cost_function_proj,
                                         loss_function_proj,
                                         cam->quaternion,
                                         cam->translation);
                total_constraints++;
            }
            number_of_constraints[constr.image1] += constr.correspondences.size();
        }

        ROS_INFO_STREAM("Total constraints "<<total_constraints);
        Solver::Summary summary;
        Solve(options, &problem, &summary);
        if (m_bVerbose){
            ROS_INFO_STREAM(summary.FullReport() << "\n");
        }

        tf::Quaternion tf_q(cam->quaternion[1],cam->quaternion[2],cam->quaternion[3], cam->quaternion[0]);
        tf::Vector3 tf_v(cam->translation[0], cam->translation[1], cam->translation[2]);
        tf::Transform ceres_transform(tf_q, tf_v);
        registered_pose.setBasis(ceres_transform.getBasis());
        registered_pose.setOrigin(ceres_transform.getOrigin());

        for (size_t i=0; i<all_clouds_1.size();i++){
            pcl_ros::transformPointCloud(*all_clouds_1[i], *all_clouds_1[i],origin_1.inverse());
            pcl_ros::transformPointCloud(*all_clouds_2[i], *all_clouds_2[i],origin_2.inverse());

            pcl_ros::transformPointCloud(*all_clouds_1[i], *all_clouds_1[i],all_initial_poses_1[i].inverse());
            pcl_ros::transformPointCloud(*all_clouds_2[i], *all_clouds_2[i],all_initial_poses_2[i].inverse());


        }

        return registered_pose;
    }

    template<class PointType>
    std::vector<std::pair<PointType, PointType>>
    validateMatches(const std::vector<std::pair<SiftGPU::SiftKeypoint,SiftGPU::SiftKeypoint>>& matches,
                    const cv::Mat& image1, const cv::Mat& image2,
                    const boost::shared_ptr<pcl::PointCloud<PointType>>& cloud1, const boost::shared_ptr<pcl::PointCloud<PointType>>& cloud2,
                    const double& depth_threshold,
                    std::vector<std::pair<cv::Point2d, cv::Point2d>>& remaining_image_matches){
        using namespace std;

        std::vector<std::pair<PointType, PointType>> filtered_matches;

        // intermediate data structure
        pcl::CorrespondencesPtr correspondences_sift(new pcl::Correspondences);
        // filter nans and depth outside range
        for (size_t i=0; i<matches.size(); i++){
            auto keypoint_pair = matches[i];
            // get point indices in point clouds
            long point_index1 = (int)keypoint_pair.first.y * image1.cols + (int)keypoint_pair.first.x;
            long point_index2 = (int)keypoint_pair.second.y * image2.cols + (int)keypoint_pair.second.x;
            PointType p1 = cloud1->points[point_index1];
            PointType p2 = cloud2->points[point_index2];

            if (!pcl::isFinite(p1) || !pcl::isFinite(p2)){
                continue; // skip nan matches
            }

//            if ((p1.z > depth_threshold) || (p2.z > depth_threshold)){
//                continue; // skip points with depth outside desired range
//            }

            pcl::Correspondence c;
            c.index_match = point_index2;
            c.index_query = point_index1;
            c.distance = 1.0;
            correspondences_sift->push_back(c);
        }

//        cout<<"After depth filtering "<<correspondences_sift->size()<<endl;

        // next do RANSAC filtering
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> cr;
        pcl::Correspondences sac_correspondences;
        cr.setInputSource(cloud1);
        cr.setInputTarget(cloud2);
        cr.setInputCorrespondences(correspondences_sift);
        cr.setInlierThreshold(0.05);
        cr.setMaximumIterations(10000);
        cr.getCorrespondences(sac_correspondences);

        if (sac_correspondences.size() == correspondences_sift->size()){ // could not find a consensus
            return filtered_matches;
        }

        for (auto corresp : sac_correspondences){
            pair<PointType, PointType> filtered_match(cloud1->points[corresp.index_query], cloud2->points[corresp.index_match]);
            filtered_matches.push_back(filtered_match);

            cv::Point2d p1_2d, p2_2d;
            p1_2d.y =  (corresp.index_query / image1.cols);
            p1_2d.x =  (corresp.index_query % image1.cols);
            p2_2d.y =  (corresp.index_match / image2.cols);
            p2_2d.x =  (corresp.index_match % image2.cols);
            pair<cv::Point2d, cv::Point2d> filtered_match_2d(p1_2d, p2_2d);
            remaining_image_matches.push_back(filtered_match_2d);
        }

        return filtered_matches;
    }

    template <class PointType>
    static std::pair<cv::Mat, cv::Mat> createRGBandDepthFromCloud(boost::shared_ptr<pcl::PointCloud<PointType>>& cloud)
    {
        std::pair<cv::Mat, cv::Mat> toRet;
        toRet.first = cv::Mat::zeros(480, 640, CV_8UC3); // RGB image
        toRet.second = cv::Mat::zeros(480, 640, CV_16UC1); // Depth image
        pcl::PointXYZRGB point;
        for (size_t y = 0; y < toRet.first.rows; ++y) {
            for (size_t x = 0; x < toRet.first.cols; ++x) {
                point = cloud->points[y*toRet.first.cols + x];
                // RGB
                toRet.first.at<cv::Vec3b>(y, x)[0] = point.b;
                toRet.first.at<cv::Vec3b>(y, x)[1] = point.g;
                toRet.first.at<cv::Vec3b>(y, x)[2] = point.r;
                // Depth
                if (!pcl::isFinite(point)) {
                    toRet.second.at<u_int16_t>(y, x) = 0.0; // convert to uint 16 from meters
                } else {
                    toRet.second.at<u_int16_t>(y, x) = point.z*1000; // convert to uint 16 from meters
                }
            }
        }
        return toRet;
    }

private:
    bool m_bVerbose;
};


#endif
