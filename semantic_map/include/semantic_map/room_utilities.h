#ifndef __SEMANTIC_MAP_ROOM_UTILITIES__
#define __SEMANTIC_MAP_ROOM_UTILITIES__

#include <tf/tf.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include "semantic_map/reg_features.h"
#include "semantic_map/room_xml_parser.h"
#include <pcl_ros/transforms.h>

namespace semantic_map_room_utilities
{

    template <class PointType>
    void reprojectIntermediateCloudsUsingCorrectedParams(SemanticRoom<PointType>& aRoom)
    {
        typedef pcl::PointCloud<PointType> Cloud;
        typedef typename Cloud::Ptr CloudPtr;

        auto camParamOrig = aRoom.getIntermediateCloudCameraParameters();
        auto camParamCorrected = aRoom.getIntermediateCloudCameraParametersCorrected();
        std::vector<tf::StampedTransform> cloudTransformsReg = aRoom.getIntermediateCloudTransformsRegistered();
        std::vector<tf::StampedTransform> cloudTransforms = aRoom.getIntermediateCloudTransforms();
        std::vector<CloudPtr> clouds= aRoom.getIntermediateClouds();

        double fx = camParamCorrected[0].fx();
        double fy = camParamCorrected[0].fy();
        double center_x = camParamCorrected[0].cx();
        double center_y = camParamCorrected[0].cy();
        double cx = 0.001 / fx;
        double cy = 0.001 / fy;

        aRoom.clearIntermediateClouds();

        if (cloudTransformsReg.size() == clouds.size())
        {
           for (size_t j=0; j<clouds.size(); j++)
           {
              Cloud rebuilt_cloud = *clouds[j];
              Cloud transformed_cloud;

              RegistrationFeatures reg;

              std::pair<cv::Mat,cv::Mat> rgbAndDepth = reg.createRGBandDepthFromPC(clouds[j]);

              pcl::PointXYZRGB point;
              for (size_t y = 0; y < rgbAndDepth.first.rows; ++y) {
                 for (size_t x = 0; x < rgbAndDepth.first.cols; ++x) {

                    uint16_t depth = rgbAndDepth.second.at<u_int16_t>(y, x)/* * 1000*/;
                    if (!(depth != 0))
                    {
                        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
                    } else {
                       point.x = (x - center_x) * depth * cx;
                       point.y = (y - center_y) * depth * cy;
                       point.z = depth * 0.001f;
                    }
                    uint32_t rgb = ((uint8_t)rgbAndDepth.first.at<cv::Vec3b>(y, x)[2] << 16 | rgbAndDepth.first.at<cv::Vec3b>(y, x)[1] << 8 | rgbAndDepth.first.at<cv::Vec3b>(y, x)[0]);
                    point.rgb = *reinterpret_cast<float*>(&rgb);

                    rebuilt_cloud.points[y*rgbAndDepth.first.cols + x] = point;

                 }
              }

              CloudPtr addToRoom(new Cloud());
              *addToRoom = rebuilt_cloud;
              aRoom.addIntermediateRoomCloud(addToRoom, cloudTransforms[j], camParamOrig[j]);

           }
        } else {
            ROS_INFO_STREAM("Cannot rebuild intermediate clouds as the number of intermediate transforms is less than the number of intermediate clouds.");
            return;
        }
        return;
    }

    template <class PointType>
    void rebuildRegisteredCloud(SemanticRoom<PointType>& aRoom)
    {
        typedef pcl::PointCloud<PointType> Cloud;
        typedef typename Cloud::Ptr CloudPtr;

        std::vector<tf::StampedTransform> cloudTransformsReg = aRoom.getIntermediateCloudTransformsRegistered();
        std::vector<CloudPtr> clouds= aRoom.getIntermediateClouds();

        CloudPtr mergedCloudRegistered(new Cloud);

        if (cloudTransformsReg.size() == clouds.size())
        {
            for (size_t j=0; j<clouds.size()/3; j++)
            {
                Cloud transformed_cloud;
                pcl_ros::transformPointCloud(*clouds[j], transformed_cloud,cloudTransformsReg[j]);
                *mergedCloudRegistered+=transformed_cloud;
            }
            aRoom.setCompleteRoomCloud(mergedCloudRegistered);
        } else {
            ROS_INFO_STREAM("Cannot rebuild registered cloud. Not enough registered transforms."<<cloudTransformsReg.size()<<"  "<<clouds.size());
        }


    }

}


#endif // __SEMANTIC_MAP_ROOM_UTILITIES__
