#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

using namespace std;
using namespace cv;

CloudPtr rebuildCloud(std::vector<CloudPtr> intermediate_clouds, std::vector<tf::StampedTransform> intermediate_transforms);

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        cout<<"Please provide XML path as argument."<<endl;
        return -1;
    } else {

    }

    pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer (argc, argv, "Labelled data");
    p->addCoordinateSystem();

    auto allXmls = semantic_map_load_utilties::getSweepXmls<PointType>(argv[1],true);

    for (auto sweep_xml : allXmls){

        auto objects = semantic_map_load_utilties::loadAllDynamicObjectsFromSingleSweep<PointType>(sweep_xml,true);

        for (auto object : objects){
            cout<<"Object has "<<object.vAdditionalViews.size()<<" additional clouds "<<endl;

            if (object.intermediateCloud->points.size()){

                pcl_ros::transformPointCloud(*object.intermediateCloud, *object.intermediateCloud,object.calibratedTransform);
                pcl_ros::transformPointCloud(*object.intermediateCloud, *object.intermediateCloud,object.transformToGlobal);
                p->addPointCloud(object.intermediateCloud);


                imshow( "Display window2", object.objectDepthImage );                   // Show our image inside it.
                imshow( "Display window1", object.objectRGBImage );                   // Show our image inside it.
                waitKey(0);                                          // Wait for a keystroke in the window

                pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_handler (object.objectCloud, 255, 0, 0);
                p->addPointCloud (object.objectCloud,cloud_handler,"object_cloud");
                p->spin();
                p->removeAllPointClouds();

                if (object.vAdditionalViewsTransformsRegistered.size()){
                    // visualize registered transforms
                    // add views to both viewports
                    for (size_t i=0; i<object.vAdditionalViews.size();i++){
                        CloudPtr transformedCloud1(new Cloud);
//                        pcl_ros::transformPointCloud(*object.vAdditionalViews[i], *transformedCloud1,object.vAdditionalViewsTransforms[i]);
                        pcl_ros::transformPointCloud(*object.vAdditionalViews[i], *transformedCloud1,object.vAdditionalViewsTransformsRegistered[i]);
                        pcl_ros::transformPointCloud(*transformedCloud1, *transformedCloud1,object.additionalViewsTransformToObservation);

                        stringstream ss1; ss1<<"Cloud1";ss1<<i;
                        p->addPointCloud(transformedCloud1, ss1.str());
                    }
                    // add observation
                    //                    CloudPtr obs_cloud =semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointType>(sweep_xml,true);
                    auto room_data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointType>(sweep_xml,true);
                    CloudPtr obs_cloud;
                    obs_cloud = rebuildCloud(room_data.vIntermediateRoomClouds, room_data.vIntermediateRoomCloudTransformsRegistered);
                    pcl_ros::transformPointCloud(*obs_cloud, *obs_cloud,room_data.vIntermediateRoomCloudTransforms[0]);
                    pcl::visualization::PointCloudColorHandlerCustom<PointType> obs_cloud_handler (obs_cloud, 255, 0, 0);
                    p->addPointCloud (obs_cloud,obs_cloud_handler,"room_cloud");
                    pcl::visualization::PointCloudColorHandlerCustom<PointType> object_handler (object.objectCloud, 0, 0, 255);
                    p->addPointCloud (object.objectCloud,object_handler,"object_cloud");
                    p->spin();
                    p->removeAllPointClouds();
                }

                // additional view masks
                ROS_INFO_STREAM("The object contains "<<object.vAdditionalViewMaskImages.size()<<" additional image masks");
                for (size_t i=0; i<object.vAdditionalViewMaskImages.size(); i++){
                    ROS_INFO_STREAM("Additional image mask "<<i<<" number of indices "<<object.vAdditionalViewMaskIndices[i].size());
                    cv::imshow( "Display window", object.vAdditionalViewMaskImages[i] );
                    cv::waitKey(0);

                }
            } else {
                std::cout<<"Intermediate cloud hasn't been set for this dynamic object!!!"<<std::endl;
            }
        }
    }

    return 0;
}


CloudPtr rebuildCloud(std::vector<CloudPtr> intermediate_clouds, std::vector<tf::StampedTransform> intermediate_transforms){
    CloudPtr mergedCloud(new Cloud);

    for (size_t i=0; i<intermediate_clouds.size(); i++)
    {
        Cloud transformed_cloud;
        pcl_ros::transformPointCloud(*intermediate_clouds[i], transformed_cloud,intermediate_transforms[i]);
        *mergedCloud+=transformed_cloud;
    }
    return mergedCloud;
}
