#ifndef __DYNAMIC_OBJECT_XML_PARSER__H
#define __DYNAMIC_OBJECT_XML_PARSER__H

#include <QFile>
#include <QDir>
#include <QXmlStreamWriter>
#include <QDebug>

#include <fstream>
#include "dynamic_object.h"


class DynamicObjectXMLParser {
public:

    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;


    DynamicObjectXMLParser(std::string rootFolder = "", bool verbose = false);
    ~DynamicObjectXMLParser();

    std::string saveAsXML(DynamicObject::Ptr object, std::string xml_filename = "", std::string cloud_filename = "");
    DynamicObject::Ptr loadFromXML(std::string filename, bool load_cloud = true);


    void saveObjectTrackToXml(tf::Transform pose, CloudPtr cloud, QXmlStreamWriter* xmlWriter, std::string nodeName, std::string cloudFilename);
    DynamicObject::ObjectTrack readObjectTrackFromXml(QXmlStreamReader* xmlReader, std::string nodeName, std::string objectFolder, bool& errorReading, bool load_cloud = true);

    void saveTfStampedTransfromToXml(tf::StampedTransform transform, QXmlStreamWriter* xmlWriter, std::string nodeName);
    tf::StampedTransform readTfStampedTransformFromXml(QXmlStreamReader* xmlReader, std::string nodeName, bool& errorReading);

    std::string m_rootFolderPath;
    bool m_verbose;

};

#endif // __DYNAMIC_OBJECT_XML_PARSER__H
