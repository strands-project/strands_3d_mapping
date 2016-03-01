#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointType;

using namespace std;
using namespace cv;

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
            } else {
                std::cout<<"Intermediate cloud hasn't been set for this dynamic object!!!"<<std::endl;
            }
        }
    }

    return;
}
