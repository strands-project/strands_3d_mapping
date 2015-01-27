#include <iostream>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>

#include "object_3d_retrieval/object_retrieval.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "simpleXMLparser.h"
#include "simpleSummaryParser.h"

#include <tf_conversions/tf_eigen.h>

using namespace std;

typedef typename SimpleSummaryParser::EntityStruct Entities;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<131>;
using HistCloudT = pcl::PointCloud<HistT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;

using index_score = object_retrieval::index_score;

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

void read_clouds(vector<CloudT::Ptr>& sweeps, vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& intrinsics, vector<string>& files, size_t max_clouds = 20)
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

    for (size_t i = 0; i < allSweeps.size(); ++i) {
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
            files.push_back(allSweeps[i].roomXmlFile + " " + to_string(j));
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

int main(int argc, char** argv)
{
    int i = 34;
    if (argc >= 2) {
        i = atoi(argv[1]);
    }

    object_retrieval obr("/home/nbore/Workspace/objectness_score/object_segments");
    vector<CloudT::Ptr> sweeps;
    vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > intrinsics;
    vector<string> files;
    //read_clouds(sweeps, intrinsics, files);
    //obr.compute_segments(sweeps, intrinsics, files);

    chrono::time_point<std::chrono::system_clock> start, end;
    start = chrono::system_clock::now();
    //obr.process_segments();
    end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds = end-start;
    cout << "Vocabulary creation took " << elapsed_seconds.count() << " seconds" << endl;

    vector<index_score> scores;
    obr.query_vocabulary(scores, i, 50);

    return 0;
}
