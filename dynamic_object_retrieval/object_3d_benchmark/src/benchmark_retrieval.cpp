#include "object_3d_benchmark/benchmark_retrieval.h"
#include "object_3d_benchmark/benchmark_overlap.h"
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

namespace benchmark_retrieval {

vector<pair<CloudT::Ptr, string> > find_labels(vector<CloudT::Ptr>& input_segmented_dynamics, const vector<boost::filesystem::path>& sweep_paths)
                                               //semantic_map_load_utilties::LabelledData<PointT>& labelled_clusters)
{
    vector<pair<CloudT::Ptr, string> > labelled_segmented_dynamics;

    for (auto tup : dynamic_object_retrieval::zip(input_segmented_dynamics, sweep_paths)) {
        CloudT::Ptr segmented_dynamic;
        boost::filesystem::path sweep_xml;
        tie(segmented_dynamic, sweep_xml) = tup;

        if (sweep_xml.empty() || !segmented_dynamic || segmented_dynamic->empty()) {
            // TODO: this is a bug that might hurt results, check this
            //labelled_segmented_dynamics.push_back(make_pair(segmented_dynamic, string("")));
            CloudT::Ptr temp_cloud(new CloudT);
            temp_cloud->push_back(PointT());
            labelled_segmented_dynamics.push_back(make_pair(temp_cloud, string("")));
            continue;
        }

        LabelT labelled_clusters = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml.string());

        bool found = false;
        double max_overlap = 0.0;
        size_t max_index;
        for (size_t i = 0; i < labelled_clusters.objectClouds.size(); ++i) {
            double overlap_ratio = compute_overlap(segmented_dynamic, labelled_clusters.objectClouds[i]);
            if (overlap_ratio > 0.25 && overlap_ratio > max_overlap) {
                max_overlap = overlap_ratio;
                max_index = i;
                found = true;
            }
        }

        if (found) {
            labelled_segmented_dynamics.push_back(make_pair(segmented_dynamic, labelled_clusters.objectLabels[max_index]));
        }
        else {
            labelled_segmented_dynamics.push_back(make_pair(segmented_dynamic, string("")));
        }
    }

    return labelled_segmented_dynamics;
}

pair<Eigen::Matrix3f, vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > get_camera_matrix_and_transforms(const string& sweep_xml)
{
    semantic_map_load_utilties::IntermediateCloudCompleteData<PointT> data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(sweep_xml);

    image_geometry::PinholeCameraModel model = data.vIntermediateRoomCloudCamParams[0];

    cv::Matx33d cvK = model.intrinsicMatrix();
    Eigen::Matrix3d dK = Eigen::Map<Eigen::Matrix3d>(cvK.val);

    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    //Eigen::Affine3d first;
    //tf::transformTFToEigen(data.vIntermediateRoomCloudTransforms[0], first);
    //first = first.inverse();
    //for (tf::StampedTransform t : data.vIntermediateRoomCloudTransforms) {
    for (tf::StampedTransform t : data.vIntermediateRoomCloudTransformsRegistered) {
        Eigen::Affine3d e;
        tf::transformTFToEigen(t, e);
        camera_transforms.push_back(e.inverse().matrix().cast<float>());
        //camera_transforms.push_back((e.inverse()*first).matrix().cast<float>());
    }

    return make_pair(dK.cast<float>().transpose(), camera_transforms);
}

tuple<Eigen::Matrix3f,
      vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >,
      sensor_msgs::CameraInfo>
get_camera_info_and_transforms(const string& sweep_xml)
{
    semantic_map_load_utilties::IntermediateCloudCompleteData<PointT> data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(sweep_xml);

    image_geometry::PinholeCameraModel model = data.vIntermediateRoomCloudCamParams[0];

    cv::Matx33d cvK = model.intrinsicMatrix();
    Eigen::Matrix3d dK = Eigen::Map<Eigen::Matrix3d>(cvK.val);

    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > camera_transforms;
    //Eigen::Affine3d first;
    //tf::transformTFToEigen(data.vIntermediateRoomCloudTransforms[0], first);
    //first = first.inverse();
    //for (tf::StampedTransform t : data.vIntermediateRoomCloudTransforms) {
    for (tf::StampedTransform t : data.vIntermediateRoomCloudTransformsRegistered) {
        Eigen::Affine3d e;
        tf::transformTFToEigen(t, e);
        camera_transforms.push_back(e.inverse().matrix().cast<float>());
        //camera_transforms.push_back((e.inverse()*first).matrix().cast<float>());
    }

    return make_tuple(dK.cast<float>().transpose(), camera_transforms, model.cameraInfo());
}

Eigen::Matrix4f get_global_camera_rotation(semantic_map_load_utilties::LabelledData<PointT>& labels)
{
    tf::StampedTransform tt = labels.transformToGlobal;
    Eigen::Affine3d e;
    tf::transformTFToEigen(tt, e);
    Eigen::Matrix4f T = e.matrix().cast<float>();
    T.col(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    return T;
}

cv::Mat sweep_get_depth_at(const boost::filesystem::path& sweep_xml, size_t scan_index)
{
    stringstream ss;
    ss << "depth_" << std::setw(4) << std::setfill('0') << scan_index;
    boost::filesystem::path depth_path = sweep_xml.parent_path() / (ss.str() + ".png");
    cv::Mat depth_image = cv::imread(depth_path.string(), cv::IMREAD_UNCHANGED);
    return depth_image;
}

cv::Mat sweep_get_rgb_at(const boost::filesystem::path& sweep_xml, size_t scan_index)
{
    stringstream ss;
    ss << "rgb_" << std::setw(4) << std::setfill('0') << scan_index;
    boost::filesystem::path rgb_path = sweep_xml.parent_path() / (ss.str() + ".jpg");
    cv::Mat rgb_image = cv::imread(rgb_path.string());
    return rgb_image;
}

CloudT::Ptr get_cloud_from_sweep_mask(CloudT::Ptr& sweep, cv::Mat& mask, const Eigen::Matrix4f& mask_transform,
                                      const Eigen::Matrix3f& K)
{
    int height = mask.rows;
    int width = mask.cols;

    //Eigen::Matrix4f inv = mask_transform.inverse();

    CloudT::Ptr segment(new CloudT);
    for (const PointT& p : sweep->points) {
        Eigen::Vector4f q = mask_transform*p.getVector4fMap();
        Eigen::Vector3f r = K*q.head<3>();
        int x = int(r(0)/r(2));
        int y = int(r(1)/r(2));
        if (x >= width || x < 0 || y >= height || y < 0) {
            continue;
        }
        if (mask.at<uint8_t>(y, x) != 0) {
            segment->push_back(p);
        }
    }

    return segment;
}

} // namespace benchmark_retrieval
