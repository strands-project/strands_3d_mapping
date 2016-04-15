#ifndef __CERES_VIEW_REGISTRATION__
#define __CERES_VIEW_REGISTRATION__

#include <metaroom_xml_parser/load_utilities.h>
#include <ceres/ceres.h>
#include <siftgpu/SiftGPU.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "additional_view_registration_server/sift_wrapper.h"
#include "additional_view_registration_server/additional_view_registration_residual.h"

class AdditionalViewRegistrationOptimizer{

public:
    AdditionalViewRegistrationOptimizer(bool verbose=false);
    ~AdditionalViewRegistrationOptimizer();

    template <class PointType>
    bool registerViews(const std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& all_views, const std::vector<tf::StampedTransform>& all_initial_poses,
                                             std::vector<int>& number_of_constraints, std::vector<tf::Transform>& registered_poses){
        using namespace std;
        using namespace cv;
        using namespace ceres;

        vector<Mat> vRGBImages;
        vector<Mat> vDepthImages;

        // set up initial data
        for (auto view : all_views){
            auto image_pair = createRGBandDepthFromCloud(view);
            vRGBImages.push_back(image_pair.first);
            vDepthImages.push_back(image_pair.second);
        }

        // extract SIFT keypoints and descriptors
        struct SIFTData{
            int image_number;
            int desc_number;
            vector<float> descriptors;
            vector<SiftGPU::SiftKeypoint> keypoints;
        };

        SIFTWrapper sift_wrapper;
        vector<SIFTData> vSIFTData;

        for (size_t i=0; i<vRGBImages.size();i++){
            auto image = vRGBImages[i];
            SIFTData sift;
            sift_wrapper.extractSIFT(image, sift.desc_number, sift.descriptors, sift.keypoints);
            vSIFTData.push_back(sift);
            if (m_bVerbose){
                ROS_INFO_STREAM("Extracted "<<vSIFTData[vSIFTData.size()-1].keypoints.size()<<" SIFT keypoints for image "<<i);
            }
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

        for (size_t i=0; i<all_views.size()-1; i++){
            for (size_t j=i+1; j<all_views.size(); j++){
                ConstraintStructure constr;
                constr.image1 = i;
                constr.image2 = j;
                if (j!=i+1){
                    constr.depth_threshold = 5.0; // 2.5
                } else {
                    constr.depth_threshold = 5.0; //3.0; // 1.5
                }
                constraints_and_correspondences.push_back(constr);
            }
        }

        // get and validate sift matches
        for (ConstraintStructure& constr: constraints_and_correspondences){
            SIFTData image1_sift = vSIFTData[constr.image1];
            SIFTData image2_sift = vSIFTData[constr.image2];
            vector<pair<SiftGPU::SiftKeypoint,SiftGPU::SiftKeypoint>> matches;

            sift_wrapper.matchSIFT(image1_sift.desc_number, image2_sift.desc_number,
                                   image1_sift.descriptors, image2_sift.descriptors,
                                   image1_sift.keypoints, image2_sift.keypoints,
                                   matches);
            if (m_bVerbose){
                ROS_INFO_STREAM("Resulting number of matches for images "<<constr.image1<<" "<<constr.image2<<" is "<<matches.size());
            }
            constr.correspondences = validateMatches(matches,
                                                     vRGBImages[constr.image1], vRGBImages[constr.image2],
                                                     all_views[constr.image1], all_views[constr.image2],
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
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.num_threads = 8;
        options.num_linear_solver_threads = 8;
        double min_correspondences = 12;


        vector<Camera*>  all_cameras;
        // initial solution
        for (size_t i=0; i<all_views.size(); i++){
            Camera* cam = new Camera();

            if (all_initial_poses.size() == all_views.size()){
                cam->translation[0] = all_initial_poses[i].getOrigin().x();
                cam->translation[1] = all_initial_poses[i].getOrigin().y();
                cam->translation[2] = all_initial_poses[i].getOrigin().z();
                cam->quaternion[0]  = all_initial_poses[i].getRotation().w();
                cam->quaternion[1]  = all_initial_poses[i].getRotation().x();
                cam->quaternion[2]  = all_initial_poses[i].getRotation().y();
                cam->quaternion[3]  = all_initial_poses[i].getRotation().z();
            }

            all_cameras.push_back(cam);
            problem.AddParameterBlock(cam->quaternion,4);
            problem.AddParameterBlock(cam->translation,3);
            number_of_constraints.push_back(0);
        }

        // cost functions
        vector<CostFunction*> rosrallCFs;
        vector<LossFunction*> allLFs;
        int total_constraints = 0;

        for (auto constr : constraints_and_correspondences){
            Camera* cam1 = all_cameras[constr.image1];
            Camera* cam2 = all_cameras[constr.image2];

            if (constr.correspondences.size() < min_correspondences){
                continue;
            }

            for (size_t k=0; k<constr.correspondences.size(); k++){
                auto corresp = constr.correspondences[k];
                auto corresp_2d = constr.correspondences_2d[k];
                double point_original1[3] = {corresp.first.x, corresp.first.y, corresp.first.z};
                double point_original2[3] = {corresp.second.x, corresp.second.y, corresp.second.z};

                double point_original1_2d[2] = {corresp_2d.first.x, corresp_2d.first.y};
                double point_original2_2d[2] = {corresp_2d.second.x, corresp_2d.second.y};
                double depth1 = corresp.first.z;
                double depth2 = corresp.second.z;

                double weight = (corresp.first.getVector3fMap().squaredNorm() + corresp.second.getVector3fMap().squaredNorm())/2;

                ceres::CostFunction *cost_function_proj = ProjectionResidual::Create(point_original1, point_original2, weight);
                ceres::LossFunction *loss_function_proj = new ceres::HuberLoss(weight * 0.005);
                problem.AddResidualBlock(cost_function_proj,
                                         loss_function_proj,
                                         cam1->quaternion,
                                         cam1->translation,
                                         cam2->quaternion,
                                         cam2->translation);
                total_constraints++;
            }
            number_of_constraints[constr.image1] += constr.correspondences.size();
            number_of_constraints[constr.image2] += constr.correspondences.size();
        }

        if (!all_cameras.size()){
            ROS_WARN_STREAM("AdditionalViewRegistrationOptimizer ---- WARNING - no cameras defined for this object");
        }

        problem.SetParameterBlockConstant(all_cameras[0]->quaternion);
        problem.SetParameterBlockConstant(all_cameras[0]->translation);


        Solver::Summary summary;
        Solve(options, &problem, &summary);
        if (m_bVerbose){
            ROS_INFO_STREAM(summary.FullReport() << "\n");
        }

        cout<<"Total constraints "<<total_constraints<<endl;

        for (size_t i=0; i<all_cameras.size();i++){
            tf::Quaternion tf_q(all_cameras[i]->quaternion[1],all_cameras[i]->quaternion[2],all_cameras[i]->quaternion[3], all_cameras[i]->quaternion[0]);
            tf::Vector3 tf_v(all_cameras[i]->translation[0], all_cameras[i]->translation[1], all_cameras[i]->translation[2]);
            tf::Transform ceres_transform(tf_q, tf_v);
            registered_poses.push_back(ceres_transform);
        }

        // free memory
        // TODO check if this is necessary
        //        for (size_t i=0; i<allCFs.size();i++){
        //            delete allCFs[i];
        //        }
        //        for (size_t i=0; i<allLFs.size();i++){
        //            delete allLFs[i];
        //        }

        return true;
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

            if ((p1.z > depth_threshold) || (p2.z > depth_threshold)){
                continue; // skip points with depth outside desired range
            }

            pcl::Correspondence c;
            c.index_match = point_index2;
            c.index_query = point_index1;
            c.distance = 1.0;
            correspondences_sift->push_back(c);
        }

        cout<<"After depth filtering "<<correspondences_sift->size()<<endl;

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
