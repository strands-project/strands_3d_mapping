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

#include "simple_xml_parser.h"
#include "simple_summary_parser.h"
#include "simple_xml_parser.h"

#include <tf_conversions/tf_eigen.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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

void get_rooms(vector<SimpleSummaryParser::EntityStruct>& entities, string summary_xml_path)
{
    SimpleSummaryParser summary_parser(summary_xml_path + "index.xml");
    summary_parser.createSummaryXML(summary_xml_path);
    entities = summary_parser.getRooms();
    cout << "Entities: " << entities.size() << endl;
}

cv::Mat image;
int minx, maxx, miny, maxy;

void mouse_callback(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        minx = x;
        miny = y;
    }
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        maxx = x;
        maxy = y;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        return;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
        //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
        return;
    }

    if (minx == 0 || maxx == 0 || miny == 0 || maxy == 0) {
        return;
    }

    cv::Mat temp = image.clone();
    cv::Scalar color = cv::Scalar(255, 0, 0);
    cv::Rect rect(minx, miny, maxx-minx, maxy-miny);
    cv::rectangle(temp, rect.tl(), rect.br(), color, 2, 8, 0);
    cv::imshow("annotate", temp);
}

void annotate_cloud(CloudT::Ptr& cloud, const Eigen::Matrix3f& K, const std::string& folder, size_t counter)
{
    const int height = 480;
    const int width = 640;
    minx = maxx = miny = maxy = 0;

    image = cv::Mat::zeros(height, width, CV_8UC3);

    for (PointT& p : cloud->points) {
        Eigen::Vector3f q = K*p.getVector3fMap();
        int x = int(q(0)/q(2) + 0.5f);
        int y = int(q(1)/q(2) + 0.5f);
        if (x < 0 || x >= width || y < 0 || y >= height) {
            continue;
        }
        cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
        pixel[0] = p.b;
        pixel[1] = p.g;
        pixel[2] = p.r;
    }

    ///home/nbore/Data/Instances//kinect_1/patrol_run_2/room_2
    std::string object = boost::filesystem::path(folder).parent_path().parent_path().stem().string();

    //Create a window
    cv::namedWindow("annotate", 1);
    cv::setMouseCallback("annotate", mouse_callback, NULL);
    cv::imshow("annotate", image);
    char c = cv::waitKey(0);
    cv::destroyWindow("annotate");

    bool is_object = minx != 0 && maxx != 0 && miny != 0 && maxy != 0;

    ofstream f;
    f.open(folder + "/annotation" + to_string(counter) + ".txt");
    if (!is_object) {
        f << "null" << '\n';
    }
    else {
        f << object << " ";
        if (c == 'p') {
            cout << "got a partial object" << endl;
            f << "partial";
        }
        else if (c == 'f') {
            cout << "got a full object" << endl;
            f << "full";
        }
        else {
            cout << c << " is not a valid keypress" << endl;
            exit(0);
        }
        f << " " << minx << " " << maxx << " " << miny << " " << maxy << '\n';
    }
    f.close();
    cout << __FILE__ << ", " << __LINE__ << endl;
}

void annotate_rooms_from_xml(SimpleSummaryParser::EntityStruct& entity)
{
    SimpleXMLParser<PointT> parser;
    vector<string> xml_nodes_to_parser = {"RoomIntermediateCloud", "IntermediatePosition", "RoomStringId"};  // "RoomCompleteCloud", "RoomDynamicClusters"
    SimpleXMLParser<PointT>::RoomData room = parser.loadRoomFromXML(entity.roomXmlFile, xml_nodes_to_parser);
    boost::filesystem::path local_xml(entity.roomXmlFile);
    boost::filesystem::path sweep_dir = local_xml.parent_path();

    size_t counter;
    for (const tf::StampedTransform& Tj : room.vIntermediateRoomCloudTransforms) {
        Eigen::Affine3d e;
        tf::transformTFToEigen(Tj, e);
        Eigen::Matrix3d R = e.rotation();
        // only take the scans looking down far enough
        if (acos(Eigen::Vector3d::UnitZ().dot(R.col(2))) < 0.6*M_PI) {
            ++counter;
            continue;
        }

        image_geometry::PinholeCameraModel model = room.vIntermediateRoomCloudCamParams[counter];
        cv::Matx33d cvK = model.intrinsicMatrix();
        Eigen::Matrix3d dK = Eigen::Map<Eigen::Matrix3d>(cvK.val);
        Eigen::Matrix3f K = dK.cast<float>().transpose();

        annotate_cloud(room.vIntermediateRoomClouds[counter], K, sweep_dir.string(), counter);

        ++counter;
    }
}

int main(int argc, char** argv)
{
    using entity = SimpleSummaryParser::EntityStruct;

    string root_path = "/home/nbore/Data/Instances/";
    vector<entity> entities;
    get_rooms(entities, root_path);
    for (entity room : entities) {
        annotate_rooms_from_xml(room);
    }
}
