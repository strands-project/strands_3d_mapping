#include <iostream>
#include <stdint.h>
#include <boost/filesystem.hpp>

#include "object_3d_retrieval/object_retrieval.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<131>;
using HistCloudT = pcl::PointCloud<HistT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using index_score = object_retrieval::index_score;

// RGB-D camera constants
//center = [320 240];
//[imh, imw] = size(depth);
//constant = 570.3;
//MM_PER_M = 1000;

void images_to_cloud(cv::Mat& depth, cv::Mat& rgb, CloudT::Ptr& cloud, const Eigen::Matrix3f& K)
{
    int rows = depth.rows;
    int cols = depth.cols;
    cloud->reserve(rows*cols);
    Eigen::Vector3f pe;
    PointT pc;
    cv::Vec3b prgb;
    Eigen::ColPivHouseholderQR<Eigen::Matrix3f> dec(K);
    for (int x = 0; x < cols; ++x) {
        for (int y = 0; y < rows; ++y) {
            uint16_t d = depth.at<uint16_t>(y, x);
            if (d == 0) {
                continue;
            }
            float df = float(d)/1000.0f;
            pe(0) = x;
            pe(1) = y;
            pe(2) = 1.0f;
            pe = dec.solve(pe);
            pe *= df/pe(2);
            pc.getVector3fMap() = pe;
            prgb = rgb.at<cv::Vec3b>(y, x);
            pc.r = prgb[2];
            pc.g = prgb[1];
            pc.b = prgb[0];
            cloud->push_back(pc);
            //cout << p.getVector3fMap().transpose() << endl;
        }
    }
    //object_retrieval obr("/random");
    //obr.visualize_cloud(cloud);
}

void read_clouds(vector<CloudT::Ptr>& clouds, vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& intrinsics, vector<string>& files)
{
    Eigen::Matrix3f K;
    K << 570.3, 0.0, 320.0,
            0.0, 570.3, 240.0,
            0.0, 0.0, 1.0;

    std::string depth_string = "_depth";
    boost::filesystem::path p1 = "/home/nbore/Data/rgbd-scenes/";
    typedef vector<boost::filesystem::path> vec;
    vec v1; // so we can sort them later
    copy(boost::filesystem::directory_iterator(p1), boost::filesystem::directory_iterator(), back_inserter(v1));
    sort(v1.begin(), v1.end()); // sort, since directory iteration
    cout << "Opening path " << p1.string() << endl;
    for (const boost::filesystem::path& p2 : v1) {
        if (!boost::filesystem::is_directory(p2) || p2.stem().string() == "background") {
            continue;
        }
        vec v2; // so we can sort them later
        copy(boost::filesystem::directory_iterator(p2), boost::filesystem::directory_iterator(), back_inserter(v2));
        sort(v2.begin(), v2.end()); // sort, since directory iteration
        cout << "Opening path " << p2.string() << endl;
        for (const boost::filesystem::path& p3 : v2) {
            if (!boost::filesystem::is_directory(p3)) {
                continue;
            }
            vec v3; // so we can sort them later
            copy(boost::filesystem::directory_iterator(p3), boost::filesystem::directory_iterator(), back_inserter(v3));
            sort(v3.begin(), v3.end()); // sort, since directory iteration
            cout << "Opening path " << p3.string() << endl;
            for (const boost::filesystem::path& p4 : v3) {
                if (!boost::filesystem::is_regular_file(p4)) {
                    continue;
                }
                // read point clouds
                cout << "Trying file " << p4.string() << endl;
                string depth_path = p4.string();
                string depth_name = p4.stem().string();
                if (!std::equal(depth_name.rbegin(), depth_name.rbegin()+depth_string.size(), depth_string.rbegin())) {
                    continue;
                }
                cout << "Converting file " << p4.string() << endl;
                string rgb_path = depth_path.substr(0, depth_path.length() - 10) + ".png";
                cv::Mat depth = cv::imread(depth_path, CV_16UC1);
                cv::Mat rgb = cv::imread(rgb_path);
                clouds.push_back(CloudT::Ptr(new CloudT));
                images_to_cloud(depth, rgb, clouds.back(), K);
                intrinsics.push_back(K);
                files.push_back(rgb_path);
            }
        }
    }
}

// This program just reads in the point cloud, segments them and then extracts and saves features
int main(int argc, char** argv)
{
    vector<CloudT::Ptr> clouds;
    vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > intrinsics;
    vector<string> files;
    //read_clouds(clouds, intrinsics, files);
    object_retrieval obr("/home/nbore/Data/rgbd-scenes/object_segments");
    //obr.compute_segments(clouds, intrinsics, files);
    obr.process_segments_incremental();

    return 0;
}
