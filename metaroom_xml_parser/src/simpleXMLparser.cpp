#include "simpleXMLparser.h"

//    template <class PointType>
//std::pair<cv::Mat, cv::Mat> SimpleXMLParser::createRGBandDepthFromPC(boost::shared_ptr<pcl::PointCloud<PointType>> cloud)
//{
//    std::pair<cv::Mat, cv::Mat> toRet;
//    toRet.first = cv::Mat::zeros(480, 640, CV_8UC3); // RGB image
//    toRet.second = cv::Mat::zeros(480, 640, CV_16UC1); // Depth image
//    pcl::PointXYZRGB point;
//    for (size_t y = 0; y < toRet.first.rows; ++y) {
//        for (size_t x = 0; x < toRet.first.cols; ++x) {
//            point = cloud->points[y*toRet.first.cols + x];
//            // RGB
//            toRet.first.at<cv::Vec3b>(y, x)[0] = point.b;
//            toRet.first.at<cv::Vec3b>(y, x)[1] = point.g;
//            toRet.first.at<cv::Vec3b>(y, x)[2] = point.r;
//            // Depth
//            toRet.second.at<u_int16_t>(y, x) = point.z*1000; // convert to uint 16 from meters

//        }
//    }

//    return toRet;
//}


//SimpleXMLParser::SimpleXMLParser()
//{
//}

//SimpleXMLParser::~SimpleXMLParser()
//{

//}


template class SimpleXMLParser<pcl::PointXYZRGB>;
//template class SimpleXMLParser<float>;
