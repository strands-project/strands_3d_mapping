#ifndef __SIMPLE_DYNAMIC_OBJECT_PARSER__H
#define __SIMPLE_DYNAMIC_OBJECT_PARSER__H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "ros/time.h"
#include "ros/serialization.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tf/tf.h"

#include <QFile>
#include <QDir>
#include <QXmlStreamWriter>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "metaroom_xml_parser/simple_xml_parser.h"

template <class PointType>
class SimpleDynamicObjectParser {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    struct DynamicObjectData{
        std::string                      m_roomLogString;
        std::string                      m_roomStringId; // waypoint_#
        int                              m_roomRunNumber; // room_#
        std::string                      m_label;
        boost::posix_time::ptime         m_time;
        //        tf::StampedTransform             m_transformToGlobal; // from camera frame to map frame
        //        tf::StampedTransform             m_calibratedTransform; // registration transform for the intermediate cloud in which this object lies

        CloudPtr                         m_points;
        std::vector<CloudPtr>            m_vAdditionalViews;
        std::vector<tf::StampedTransform> m_vAdditionalViewsTransforms;

    };

    SimpleDynamicObjectParser(){};
    ~SimpleDynamicObjectParser(){};

    static DynamicObjectData loadDynamicObject(std::string filename, bool verbose = false, bool load_cloud=true){
        using namespace std;

        DynamicObjectData toRet;
        QFile file(filename.c_str());

        if (!file.exists())
        {
            std::cerr<<"Could not open file "<<filename<<" to load object."<<std::endl;
            return toRet;
        }

        QString xmlFileQS(filename.c_str());
        int index = xmlFileQS.lastIndexOf('/');
        QString objectFolder = xmlFileQS.left(index);

        file.open(QIODevice::ReadOnly);
        QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);
        Eigen::Vector4f centroid(0.0,0.0,0.0,0.0);

        while (!xmlReader->atEnd() && !xmlReader->hasError())
        {
            QXmlStreamReader::TokenType token = xmlReader->readNext();

            if (token == QXmlStreamReader::StartDocument)
                continue;

            QString elementName = xmlReader->name().toString();

            if (xmlReader->hasError())
            {
                std::cout << "XML error: " << xmlReader->errorString().toStdString() << std::endl;
                delete xmlReader;
                return toRet;
            }

            if (token == QXmlStreamReader::StartElement)
            {
                if (xmlReader->name() == "DynamicObject")
                {
                    QXmlStreamAttributes attributes = xmlReader->attributes();
                    if (attributes.hasAttribute("label"))
                    {
                        QString labelS = attributes.value("label").toString();
                        toRet.m_label = labelS.toStdString();
                    } else {
                        std::cerr<<"Object xml node does not have label attribute. Aborting."<<std::endl;
                        delete xmlReader;
                        return toRet;
                    }
                    if (attributes.hasAttribute("roomLogString"))
                    {
                        QString labelS = attributes.value("roomLogString").toString();
                        toRet.m_roomLogString = labelS.toStdString();
                    } else {
                        std::cerr<<"Object xml node does not have roomLogString attribute."<<std::endl; // leaving blank
                    }
                    if (attributes.hasAttribute("roomStringId"))
                    {
                        QString labelS = attributes.value("roomStringId").toString();
                        toRet.m_roomStringId = labelS.toStdString();
                    } else {
                        std::cerr<<"Object xml node does not have roomStringId attribute."<<std::endl; // leaving blank
                    }
                    if (attributes.hasAttribute("roomRunNumber"))
                    {
                        toRet.m_roomRunNumber = attributes.value("roomRunNumber").toString().toInt();
                    } else {
                        std::cerr<<"Object xml node does not have roomRunNumber attribute."<<std::endl; // leaving blank
                    }


                    if (attributes.hasAttribute("filename"))
                    {
                        if (load_cloud)
                        {
                            QString fileS = objectFolder + "/" + attributes.value("filename").toString();

                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (fileS.toStdString(), *cloud);
                            toRet.m_points = cloud;
                        }

                    } else {
                        std::cerr<<"Object xml node does not have filename attribute. Aborting."<<std::endl;
                        delete xmlReader;
                        return toRet;
                    }

                    if (attributes.hasAttribute("additionalViews"))
                    {
                        int additionalViews = attributes.value("additionalViews").toString().toInt();
                        if (load_cloud)
                        {
                            // load clouds
                            for (size_t i=0; i<additionalViews; i++)
                            {
                                stringstream ss;ss<<i;
                                string view_path = objectFolder.toStdString() + "/" + toRet.m_label + "_additional_view_"+ss.str()+".pcd";
                                pcl::PCDReader reader;
                                CloudPtr cloud (new Cloud);
                                reader.read (view_path, *cloud);
                                if (cloud->points.size() != 0)
                                {
                                    toRet.m_vAdditionalViews.push_back(cloud);
                                    if (verbose)
                                    {
                                        std::cout<<"Loaded additional view cloud "<<view_path<<std::endl;
                                    }
                                } else {
                                    std::cerr<<"Could not load additional view cloud "<<view_path<<std::endl;
                                }
                            }
                        } else {
//                            object->m_noAdditionalViews = additionalViews; // set this value but don't load anything
                        }
                    } else {
                        // additional views not set
                    }

                }

                if (xmlReader->name() == "LogTime")
                {
                    QString logTime = xmlReader->readElementText();
                    boost::posix_time::ptime objectTime = boost::posix_time::time_from_string(logTime.toStdString());
                    toRet.m_time = objectTime;
                }

                if (xmlReader->name() == "AdditonalViewTransform")
                {
                    bool errorReading = false;
//                    tf::StampedTransform transform = readTfStampedTransformFromXml(xmlReader, "AdditonalViewTransform", errorReading);
                    tf::StampedTransform transform = SimpleXMLParser<PointType>::readTfStampedTransformFromXml(xmlReader, "AdditonalViewTransform", errorReading);
                    if (!errorReading)
                    {
                        toRet.m_vAdditionalViewsTransforms.push_back(transform);
                    }
                }
            }
        }

        if (xmlReader->hasError())
        {
            std::cerr<<"Error while loading dynamic object from file "<<filename<<std::endl;
        }

        if (verbose)
        {
            cout<<"Loaded object from: "<<filename<<endl;
        }

        delete xmlReader;
        return toRet;
    };

};


#endif
