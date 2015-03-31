#ifndef __SEMANTIC_ROOM_XML_PARSER__H
#define __SEMANTIC_ROOM_XML_PARSER__H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <ros/time.h>
#include <ros/serialization.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tf/tf.h>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QFile>
#include <QDir>
#include <QXmlStreamWriter>
#include <QDebug>

#include <fstream>

#include "room.h"


template <class PointType>
class SemanticRoomXMLParser {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    typedef typename SemanticRoom<PointType>::IntermediatePositionImages IntermediatePositionImages;

    struct IntermediateCloudData{
        std::string                                filename;
        tf::StampedTransform                       transform;
        tf::StampedTransform                       regTransform;
        sensor_msgs::CameraInfo                    camInfo;
        image_geometry::PinholeCameraModel         corCamInfo;
        bool                                       hasRegTransform;
        bool                                       hasCorCamInfo;
    };


    SemanticRoomXMLParser(std::string rootFolder="home");
    ~SemanticRoomXMLParser();

    bool setRootFolderFromRoomXml(std::string roomXml);
    std::string saveRoomAsXML(SemanticRoom<PointType>& aRoom, std::string xmlFile="room.xml", bool verbose = false);
    void  saveIntermediateImagesToXML(SemanticRoom<PointType>& aRoom, QXmlStreamWriter* xmlWriter, std::string roomFolder);


    static SemanticRoom<PointType> loadRoomFromXML(const std::string& xmlFile, bool deepLoad=true, bool verbose = false);

private:

    /********************************************READING ************************************************************************/

    static IntermediateCloudData parseRoomIntermediateCloudNode(QXmlStreamReader& xmlReader);
    static IntermediatePositionImages parseIntermediatePositionImages(QXmlStreamReader* xmlReader, std::string roomFolder, bool deepLoad);

    // Reads in a image_geometry::PinholeCameraModel from the current xml node
    // Does not advance the xml reader!!

    static image_geometry::PinholeCameraModel readCamParamsFromXml(QXmlStreamReader* xmlReader, std::string nodeName, bool& errorReading);

    // Reads in a TfStampedTransform from the current xml node
    // Does not advance the xml reader!!

    static tf::StampedTransform readTfStampedTransformFromXml(QXmlStreamReader* xmlReader, std::string nodeName, bool& errorReading);

    /******************************************** WRITING  ************************************************************************/

    void saveTfStampedTransfromToXml(tf::StampedTransform transform, QXmlStreamWriter* xmlWriter, std::string nodeName);
    void saveCameraParametersToXML(image_geometry::PinholeCameraModel cam_model, QXmlStreamWriter* xmlWriter, std::string nodeName);

    /******************************************** MEMBERS ************************************************************************/

    QString                                 m_RootFolder;
};

#include "semantic_map/room_xml_parser.hpp"

#endif
