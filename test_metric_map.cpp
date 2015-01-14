#include <iostream>

#include "convex_voxel_segmentation.h"
#include "segment_features.h"
#include "k_means_tree/k_means_tree.h"
#include "vocabulary_tree/vocabulary_tree.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "simpleXMLparser.h"
#include "simpleSummaryParser.h"

#include <tf_conversions/tf_eigen.h>

#define VISUALIZE false

using namespace std;

typedef typename SimpleSummaryParser::EntityStruct Entities;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<131>;
using HistCloudT = pcl::PointCloud<HistT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;

void visualize_cloud(CloudT::Ptr& cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void subsample_cloud(CloudT::Ptr& cloud_in, CloudT::Ptr& cloud_out)
{
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_out);
}

void translate_cloud(CloudT::Ptr& cloud, const Eigen::Vector3f& offset)
{
    for (PointT& p : cloud->points) {
        p.getVector3fMap() += offset;
    }
}

void read_clouds(vector<CloudT::Ptr>& sweeps, vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& intrinsics, size_t max_clouds = 20)
{
    //string folderPath = argv[1];
    string folderPath = "/home/nbore/Data/semantic_map/";
    string summaryXMLPath = folderPath + "/index.xml";

    SimpleSummaryParser summary_parser(summaryXMLPath);
    summary_parser.createSummaryXML(folderPath);

    SimpleXMLParser<PointT> simple_parser;
    SimpleXMLParser<PointT>::RoomData roomData;

    vector<Entities> allSweeps = summary_parser.getRooms();

    cout << "Sweeps size: " << allSweeps.size() << endl;

    for (size_t i = 0; i < allSweeps.size() && i < max_clouds; ++i) {
        cout << "Parsing " << allSweeps[i].roomXmlFile << endl;

        roomData = simple_parser.loadRoomFromXML(allSweeps[i].roomXmlFile);
        cout << "Complete cloud size " << roomData.completeRoomCloud->points.size() << endl;
        cout << "Room waypoint id " << roomData.roomWaypointId << endl;

        image_geometry::PinholeCameraModel model = roomData.vIntermediateRoomCloudCamParams[0];
        cv::Matx33d cvK = model.intrinsicMatrix();
        Eigen::Matrix3d dK = Eigen::Map<Eigen::Matrix3d>(cvK.val);
        Eigen::Matrix3f K = dK.cast<float>().transpose(); // transpose ?

        cout << K << endl;

        CloudT::Ptr complete_cloud(new CloudT);

        for (size_t j = 0; j < roomData.vIntermediateRoomClouds.size(); ++j) {
            cout << "Intermediate cloud size " << roomData.vIntermediateRoomClouds[j]->points.size() << endl;
            cout << "Fx: "<<roomData.vIntermediateRoomCloudCamParams[j].fx() << " Fy: "<<roomData.vIntermediateRoomCloudCamParams[j].fy() << endl;
            intrinsics.push_back(K);
            sweeps.push_back(roomData.vIntermediateRoomClouds[j]);
            continue;
            tf::StampedTransform Tj = roomData.vIntermediateRoomCloudTransforms[j];
            Eigen::Affine3d e;
            tf::transformTFToEigen(Tj, e);
            CloudT::Ptr temp_cloud(new CloudT);
            pcl::transformPointCloud(*(roomData.vIntermediateRoomClouds[j]), *temp_cloud, e);
            *complete_cloud += *temp_cloud;
        }
        continue;
        CloudT::Ptr subsampled_cloud(new CloudT);
        subsample_cloud(complete_cloud, subsampled_cloud);

        tf::StampedTransform T0 = roomData.vIntermediateRoomCloudTransforms[0];
        Eigen::Vector3d v;
        tf::vectorTFToEigen(T0.getOrigin(), v);
        translate_cloud(subsampled_cloud, -v.cast<float>());

        intrinsics.push_back(K);
        sweeps.push_back(subsampled_cloud);
    }

}

void extract_features(vector<int>& inds, HistCloudT::Ptr& features, vector<CloudT::Ptr> segments,
                      vector<NormalCloudT::Ptr> normals, vector<CloudT::Ptr> hd_segments, const Eigen::Matrix3f& K)
{
    int counter = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        segment_features sf(K, false);
        HistCloudT::Ptr featuresi(new HistCloudT);
        Eigen::VectorXf globalf;
        sf.calculate_features(globalf, featuresi, segments[i], normals[i], hd_segments[i]);
        for (size_t j = 0; j < featuresi->size(); ++j) {
            inds.push_back(counter);
        }
        features->insert(features->end(), featuresi->begin(), featuresi->end());
        ++counter;
    }
}

void get_query_cloud(HistCloudT::Ptr& query_cloud, int i, vector<CloudT::Ptr> segments,
                     vector<NormalCloudT::Ptr> normals, vector<CloudT::Ptr> hd_segments, const Eigen::Matrix3f& K)
{
    segment_features sf(K, true);
    Eigen::VectorXf globalf;
    sf.calculate_features(globalf, query_cloud, segments[i], normals[i], hd_segments[i]);
    cout << "Segment to be queried: " << i << endl;
    cout << "Number of features: " << query_cloud->size() << endl;
    visualize_cloud(hd_segments[i]);
}

int main(int argc, char** argv)
{
    vector<CloudT::Ptr> sweeps;
    vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > intrinsics;
    read_clouds(sweeps, intrinsics, 3);

    vector<CloudT::Ptr> segments;
    vector<NormalCloudT::Ptr> normals;
    vector<CloudT::Ptr> hd_segments;
    vector<int> start_inds;

    size_t max_segments = 5;
    size_t counter = 0;
    for (CloudT::Ptr cloud : sweeps) {
        if (counter >= max_segments) {
            break;
        }

        if (VISUALIZE) {
            visualize_cloud(cloud);
        }
        convex_voxel_segmentation cvs(VISUALIZE, 0.012f, 0.02f);
        vector<CloudT::Ptr> segmentsi;
        vector<NormalCloudT::Ptr> normalsi;
        vector<CloudT::Ptr> hd_segmentsi;
        cvs.segment_objects(segmentsi, normalsi, hd_segmentsi, cloud);

        start_inds.push_back(segments.size());
        segments.insert(segments.end(), segmentsi.begin(), segmentsi.end());
        normals.insert(normals.end(), normalsi.begin(), normalsi.end());
        hd_segments.insert(hd_segments.end(), hd_segmentsi.begin(), hd_segmentsi.end());

        ++counter;
    }

    HistCloudT::Ptr features(new HistCloudT);
    vector<int> indices;
    extract_features(indices, features, segments, normals, hd_segments, intrinsics[0]);

    for (int i : indices) cout << i << " "; cout << endl;

    for (HistT& h : features->points) {
        eig(h).normalize();
    }

    vocabulary_tree<HistT, 8> vt;
    vt.set_input_cloud(features, indices);
    vt.add_points_from_input_cloud();

    using index_score = vocabulary_tree<HistT, 8>::cloud_idx_score;
    vector<index_score> scores;

    HistCloudT::Ptr query_cloud(new HistCloudT);
    //get_query_cloud(query_cloud, 34, segments, normals, hd_segments, intrinsics[0]); // 20 Drawer // 34 Blue Cup
    for (size_t i = 0; i < features->size(); ++i) {
        if (indices[i] == 34) {
            query_cloud->push_back(features->at(i));
        }
    }
    vt.top_similarities(scores, query_cloud, 50);

    for (index_score s : scores) {
        cout << "Index: " << s.first << " with score: " << s.second << endl;
        visualize_cloud(hd_segments[s.first]);
    }

    return 0;
}
