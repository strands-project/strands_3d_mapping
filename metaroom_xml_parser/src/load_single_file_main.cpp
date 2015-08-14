#include "metaroom_xml_parser/simple_xml_parser.h"

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


    SimpleXMLParser<PointType> parser;

//    SimpleXMLParser::RoomData<PointType> roomData;
    SimpleXMLParser<PointType>::RoomData roomData;


    roomData = parser.loadRoomFromXML(argv[1]);

    cout<<"Complete cloud size "<<roomData.completeRoomCloud->points.size()<<endl;

    for (size_t i=0; i<roomData.vIntermediateRoomClouds.size(); i++)
    {
        cout<<"Intermediate cloud size "<<roomData.vIntermediateRoomClouds[i]->points.size()<<endl;
        cout<<"Fx: "<<roomData.vIntermediateRoomCloudCamParams[i].fx()<<" Fy: "<<roomData.vIntermediateRoomCloudCamParams[i].fy()<<endl;
    }

    for (SimpleXMLParser<PointType>::IntermediatePositionImages int_image : roomData.vIntermediatePositionImages)
    {
        cout<<"Depth transform "<<endl;
        cout<<"Stamp sec "<<int_image.intermediateDepthTransform.stamp_.sec<<endl;
        cout<<"Stamp nsec "<<int_image.intermediateDepthTransform.stamp_.nsec<<endl;
        cout<<"Frame "<<int_image.intermediateDepthTransform.frame_id_<<endl;
        cout<<"Child frame "<<int_image.intermediateDepthTransform.child_frame_id_<<endl;
        cout<<"Rotation "<<int_image.intermediateDepthTransform.getRotation().getW()<<" "<<int_image.intermediateDepthTransform.getRotation().getX()<<" "<<int_image.intermediateDepthTransform.getRotation().getY()<<" "<<int_image.intermediateDepthTransform.getRotation().getZ()<<" "<<endl;
        cout<<"Translation  "<<int_image.intermediateDepthTransform.getOrigin().getX()<<" "<<int_image.intermediateDepthTransform.getOrigin().getY()<<" "<<int_image.intermediateDepthTransform.getOrigin().getZ()<<" "<<endl;

        cout<<endl<<"RGB transform "<<endl;
        cout<<"Stamp sec "<<int_image.intermediateRGBTransform.stamp_.sec<<endl;
        cout<<"Stamp nsec "<<int_image.intermediateRGBTransform.stamp_.nsec<<endl;
        cout<<"Frame "<<int_image.intermediateRGBTransform.frame_id_<<endl;
        cout<<"Child frame "<<int_image.intermediateRGBTransform.child_frame_id_<<endl;
        cout<<"Rotation "<<int_image.intermediateRGBTransform.getRotation().getW()<<" "<<int_image.intermediateRGBTransform.getRotation().getX()<<" "<<int_image.intermediateRGBTransform.getRotation().getY()<<" "<<int_image.intermediateRGBTransform.getRotation().getZ()<<" "<<endl;
        cout<<"Translation  "<<int_image.intermediateRGBTransform.getOrigin().getX()<<" "<<int_image.intermediateRGBTransform.getOrigin().getY()<<" "<<int_image.intermediateRGBTransform.getOrigin().getZ()<<" "<<endl;

        cout<<endl<<"RGB camera params "<<endl;
        cout<<"frame "<<int_image.intermediateRGBCamParams.tfFrame()<<endl;
        cout<<"fx" <<int_image.intermediateRGBCamParams.fx()<<endl;
        cout<<"fy" <<int_image.intermediateRGBCamParams.fy()<<endl;
        cout<<"cx" <<int_image.intermediateRGBCamParams.cx()<<endl;
        cout<<"cy" <<int_image.intermediateRGBCamParams.cy()<<endl;

        cout<<endl<<"Depth camera params "<<endl;
        cout<<"frame "<<int_image.intermediateDepthCamParams.tfFrame()<<endl;
        cout<<"fx" <<int_image.intermediateDepthCamParams.fx()<<endl;
        cout<<"fy" <<int_image.intermediateDepthCamParams.fy()<<endl;
        cout<<"cx" <<int_image.intermediateDepthCamParams.cx()<<endl;
        cout<<"cy" <<int_image.intermediateDepthCamParams.cy()<<endl;

//        for (cv::Mat image : int_image.vIntermediateRGBImages)
//        {
//            imshow( "Display window", image );                   // Show our image inside it.
//            waitKey(0);                                          // Wait for a keystroke in the window
//        }

//        for (cv::Mat image : int_image.vIntermediateDepthImages)
//        {
//            imshow( "Display window", image );                   // Show our image inside it.
//            waitKey(0);                                          // Wait for a keystroke in the window
//        }
    }


}
