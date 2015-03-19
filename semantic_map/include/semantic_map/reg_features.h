#ifndef __SEMANTIC_MAP_REGISTRATION_FEATURES__
#define __SEMANTIC_MAP_REGISTRATION_FEATURES__

#include "room.h"
#include "room_xml_parser.h"
#include <iostream>
//#include "opencv2/core/core.hpp"
//#include "opencv2/features2d/features2d.hpp"


class RegistrationFeatures {
public:

    struct RegistrationData
    {
        std::vector<cv::KeyPoint> keypoints;
        std::vector<double> depths;
        cv::Mat descriptors;
    };


    RegistrationFeatures(bool verbose = false, std::string data_filename = "registration_features.yml");

    ~RegistrationFeatures();

    template <class PointType>
    std::string saveOrbFeatures(SemanticRoom<PointType>& aRoom, std::string roomFolder)
    {
        std::string error ="";
        typedef pcl::PointCloud<PointType> Cloud;
        typedef typename Cloud::Ptr CloudPtr;

        std::vector<bool> intCloudsLoaded = aRoom.getIntermediateCloudsLoaded();
        std::vector<std::string> intCloudsFiles = aRoom.getIntermediateCloudsFilenames();
        std::vector<CloudPtr> intClouds = aRoom.getIntermediateClouds();

        if (intClouds.size() == 0)
        {
            // clouds not loaded. Load them
            if (intCloudsFiles.size() == 0)
            {
                ROS_ERROR("Intermediate clouds could not be found.");
                return error;
            }

            for (auto filename : intCloudsFiles)
            {
                pcl::PCDReader reader;
                CloudPtr cloud (new Cloud);
                reader.read (filename, *cloud);
                intClouds.push_back(cloud);

                if (m_verbose)
                {
                    ROS_INFO_STREAM("Loading int cloud "<<filename);
                }
            }
        }

        // figure out where to save the descriptors & keypoints
        std::string dataFile = roomFolder + "/"; // just in case the user forgot to add it;
        dataFile += m_dataFilename;

        if (m_verbose)
        {
            ROS_INFO_STREAM("Saving ORB features at: "<<dataFile);
        }

        cv::FileStorage fs(dataFile,cv::FileStorage::WRITE);
        int counter =0;
        for (CloudPtr int_cloud : intClouds)
        {
            auto images = createRGBandDepthFromPC(int_cloud);
            cv::Mat rgb_image = images.first;
            cv::Mat depth_image = images.second;

            std::vector<cv::KeyPoint> keypoints;
            std::vector<double> depths;
            cv::Mat descriptors;
            cv::ORB orb = cv::ORB(600,2.5f, 1, 3, 0,2, cv::ORB::HARRIS_SCORE, 31);
            orb(rgb_image, cv::Mat(), keypoints, descriptors);

            for (size_t i=0; i<keypoints.size();i++)
            {
                uint16_t depth = depth_image.at<uint16_t>(keypoints[i].pt);
                double ddepth = (double)depth * 0.001; // convert to meters
                depths.push_back(ddepth);
            }

            if (m_verbose)
            {
                ROS_INFO_STREAM("Extracted "<<keypoints.size()<<" ORB keypoints. Matrix size: "<<descriptors.rows<<"  "<<descriptors.cols<<"  type "<<descriptors.type());
            }

            std::stringstream ss_k;ss_k<<"keypoints"<<counter;
            std::stringstream ss_d;ss_d<<"descriptors"<<counter;
            std::stringstream ss_v;ss_v<<"depths"<<counter;
            counter++;

            cv::write(fs, ss_k.str(),keypoints);
            cv::write(fs, ss_d.str(),descriptors);
            cv::write(fs, ss_v.str(),depths);

//            cv::drawKeypoints(rgb_image, keypoints, rgb_image);
//            cv::imshow( "Display window", rgb_image );                   // Show our image inside it.
//            cv::waitKey(0);                                          // Wait for a keystroke in t
        }

    }

    std::vector<RegistrationData> loadOrbFeatures(std::string sweepXmlPath, bool verbose = false, std::string registrationFeaturesFilename = "registration_features.yml")
    {
        std::vector<RegistrationData> toRet;

        unsigned found = sweepXmlPath.find_last_of("/");
        std::string base_path = sweepXmlPath.substr(0,found+1);
        std::string regFile = base_path + registrationFeaturesFilename;

        cv::FileStorage fs2;
        if (!fs2.open(regFile,cv::FileStorage::READ))
        {
            std::cout<<"Could not open file "<<regFile<<" to read registration features."<<std::endl;
            return toRet;
        }

        int counter=0;
        while (true)
        {
            std::stringstream ss_k;ss_k<<"keypoints"<<counter;
            std::stringstream ss_d;ss_d<<"descriptors"<<counter;
            std::stringstream ss_v;ss_v<<"depths"<<counter;
            counter++;

            RegistrationData reg;

            cv::FileNode kpFN = fs2[ss_k.str()];
            cv::read(kpFN, reg.keypoints);
            if (kpFN.empty() || kpFN.isNone())
            {
                break;
            }

            kpFN = fs2[ss_d.str()];
            cv::read(kpFN, reg.descriptors);
            kpFN = fs2[ss_v.str()];
            cv::read(kpFN, reg.depths);
            toRet.push_back(reg);
        }

        if (verbose)
        {
            std::cout<<"Read registration descriptors for "<<counter-1<<" intermediate images from sweep "<<sweepXmlPath<<std::endl;
        }

        return toRet;
    }

    template <class PointType>
    std::pair<cv::Mat, cv::Mat> createRGBandDepthFromPC(boost::shared_ptr<pcl::PointCloud<PointType>> cloud)
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
                toRet.second.at<u_int16_t>(y, x) = point.z*1000; // convert to uint 16 from meters

            }
        }

        return toRet;
    }

private:
    bool m_verbose;
    std::string m_dataFilename;

};

#endif //__SEMANTIC_MAP_REGISTRATION_FEATURES__
