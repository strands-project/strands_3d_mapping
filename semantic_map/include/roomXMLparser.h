#ifndef __SEMANTIC_ROOM_XML_PARSER__H
#define __SEMANTIC_ROOM_XML_PARSER__H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "ros/time.h"
#include "ros/serialization.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tf/tf.h"

#include "room.h"

// QT
#include <QFile>
#include <QDir>
#include <QXmlStreamWriter>

template <class PointType>
class SemanticRoomXMLParser {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;


    SemanticRoomXMLParser(std::string rootFolder="home")
    {
        if (rootFolder == "home")
        {
            m_RootFolder =(QDir::homePath() + QString("/.semanticMap/"));
        }

        // create root folder
        if (!QDir(m_RootFolder).exists())
        {
            bool folderCreated = QDir().mkdir(m_RootFolder);
            ROS_INFO_STREAM("Creating the root folder returned "<<folderCreated<<" folder name "<<m_RootFolder.toStdString());
        }

    }

    ~SemanticRoomXMLParser()
    {

    }

    std::string saveRoomAsXML(SemanticRoom<PointType>& aRoom, std::string xmlFile="room.xml")
    {
        // create folder structure
        QString roomLogName(aRoom.getRoomLogName().c_str());
        int index = roomLogName.indexOf('_');
        QString date = roomLogName.left(index);
        QString patrol = roomLogName.right(roomLogName.size()-index-1);

        QString dateFolder = QString(m_RootFolder)+date;
        if (!QDir(dateFolder).exists())
        {
            QDir().mkdir(dateFolder);
        }

        QString patrolFolder = dateFolder + '/' + patrol + '/';
        if (!QDir(patrolFolder).exists())
        {
            QDir().mkdir(patrolFolder);
        }

        QString roomFolder = patrolFolder + "room_" + QString::number(aRoom.getRoomRunNumber())+ "/";
        if (!QDir(roomFolder).exists())
        {
            QDir().mkdir(roomFolder);
        }

        xmlFile = roomFolder.toStdString() + xmlFile;

        QFile file(xmlFile.c_str());
        if (file.exists())
        {
            file.remove();
        }


        if (!file.open(QIODevice::ReadWrite | QIODevice::Text))
        {
            std::cerr<<"Could not open file "<<xmlFile<<" to save room as XML"<<std::endl;
            return "";
        }



        QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
        xmlWriter->setDevice(&file);

        xmlWriter->writeStartDocument();
        xmlWriter->writeStartElement("SemanticRoom");

        // RoomLogName
        xmlWriter->writeStartElement("RoomLogName");
        xmlWriter->writeCharacters(QString(aRoom.getRoomLogName().c_str()));
        xmlWriter->writeEndElement();

        // RoomRunNumber
        xmlWriter->writeStartElement("RoomRunNumber");
        xmlWriter->writeCharacters(QString::number(aRoom.getRoomRunNumber()));
        xmlWriter->writeEndElement();

        // RoomStringId
        xmlWriter->writeStartElement("RoomStringId");
        xmlWriter->writeCharacters(aRoom.getRoomStringId().c_str());
        xmlWriter->writeEndElement();

        // RoomLogStartTime
        xmlWriter->writeStartElement("RoomLogStartTime");
        boost::posix_time::ptime roomLogStartTime = aRoom.getRoomLogStartTime();
        xmlWriter->writeCharacters(boost::posix_time::to_simple_string(roomLogStartTime).c_str());
        xmlWriter->writeEndElement();

        // RoomLogEndTime
        xmlWriter->writeStartElement("RoomLogEndTime");
        boost::posix_time::ptime roomLogEndTime = aRoom.getRoomLogEndTime();
        xmlWriter->writeCharacters(boost::posix_time::to_simple_string(roomLogEndTime).c_str());
        xmlWriter->writeEndElement();

        // RoomCompleteCloud
        xmlWriter->writeStartElement("RoomCompleteCloud");
        QString completeCloudFilename("complete_cloud.pcd");
        completeCloudFilename = roomFolder + completeCloudFilename; // add the folder prefix
        xmlWriter->writeAttribute("filename",completeCloudFilename);
        xmlWriter->writeEndElement();
        if (aRoom.getCompleteRoomCloudLoaded()) // only save the cloud file if it's been loaded
        {
            QFile file(completeCloudFilename);
            if (!file.exists())
            {
                pcl::io::savePCDFileBinary(completeCloudFilename.toStdString(), *aRoom.getCompleteRoomCloud());
                ROS_INFO_STREAM("Saving complete cloud file name "<<completeCloudFilename.toStdString());
            }
        }

        // RoomInteriorCloud
        xmlWriter->writeStartElement("RoomInteriorCloud");
        QString interiorCloudFilename("interior_cloud.pcd");
        interiorCloudFilename = roomFolder + interiorCloudFilename; // add the folder prefix
        xmlWriter->writeAttribute("filename",interiorCloudFilename);
        xmlWriter->writeEndElement();
        if (aRoom.getInteriorRoomCloudLoaded()) // only save the cloud file if it's been loaded
        {
            QFile file(interiorCloudFilename);
            if (!file.exists())
            {
                pcl::io::savePCDFileBinary(interiorCloudFilename.toStdString(), *aRoom.getInteriorRoomCloud());
                ROS_INFO_STREAM("Saving interior cloud file name "<<interiorCloudFilename.toStdString());
            }
        }

        // RoomDeNoisedCloud
        xmlWriter->writeStartElement("RoomDeNoisedCloud");
        QString denoisedCloudFilename("denoised_cloud.pcd");
        denoisedCloudFilename = roomFolder + denoisedCloudFilename; // add the folder prefix
        xmlWriter->writeAttribute("filename",denoisedCloudFilename);
        xmlWriter->writeEndElement();
        if (aRoom.getDeNoisedRoomCloudLoaded()) // only save the cloud file if it's been loaded
        {
            QFile file(denoisedCloudFilename);
            if (!file.exists())
            {
                pcl::io::savePCDFileBinary(denoisedCloudFilename.toStdString(), *aRoom.getDeNoisedRoomCloud());
                ROS_INFO_STREAM("Saving denoised cloud file name "<<denoisedCloudFilename.toStdString());
            }
        }

        // RoomDynamicClusters
        xmlWriter->writeStartElement("RoomDynamicClusters");
        QString dynamicClustersFilename("dynamic_clusters.pcd");
        dynamicClustersFilename = roomFolder + dynamicClustersFilename; // add the folder prefix
        xmlWriter->writeAttribute("filename",dynamicClustersFilename);
        xmlWriter->writeEndElement();
        if (aRoom.getDynamicClustersCloudLoaded()) // only save the cloud file if it's been loaded
        {
            QFile file(dynamicClustersFilename);
            if (!file.exists())
            {
                pcl::io::savePCDFileBinary(dynamicClustersFilename.toStdString(), *aRoom.getDynamicClustersCloud());
                ROS_INFO_STREAM("Saving dynamic clusters cloud file name "<<dynamicClustersFilename.toStdString());
            }
        }

        // RoomCentroid
        xmlWriter->writeStartElement("Centroid");
        Eigen::Vector4f centroid = aRoom.getCentroid();
        QString centroidS = QString::number(centroid(0))+" "+QString::number(centroid(1))+" "+QString::number(centroid(2))+" "+QString::number(centroid(3));
        xmlWriter->writeCharacters(centroidS);
        xmlWriter->writeEndElement();

        // Room Transform
        xmlWriter->writeStartElement("RoomTransform");
        Eigen::Matrix4f transform = aRoom.getRoomTransform();
        QString transformS;
        for (size_t i=0; i<16;i++)
        {
            transformS+=QString::number(transform(i))+" ";
        }
        xmlWriter->writeCharacters(transformS);
        xmlWriter->writeEndElement();

        // RoomIntermediateClouds
        xmlWriter->writeStartElement("RoomIntermediateClouds");
        std::vector<CloudPtr> roomIntermediateClouds = aRoom.getIntermediateClouds();
        std::vector<tf::StampedTransform> roomIntermediateCloudTransforms = aRoom.getIntermediateCloudTransforms();
        std::vector<bool>   roomIntermediateCloudsLoaded = aRoom.getIntermediateCloudsLoaded();
//        if (roomIntermediateClouds.size() != roomIntermediateCloudTransforms.size())
//        {
//            std::cerr<<"The size of the intermediate cloud vector and the transforms vector do not match"<<std::endl;
//            xmlWriter->writeCharacters("Error");
//        } else {
            for (size_t i=0; i<roomIntermediateCloudTransforms.size(); i++)
            {
                // RoomIntermediateCloud
                xmlWriter->writeStartElement("RoomIntermediateCloud");
                std::stringstream ss;
                QString intermediateCloudPath = "";
                if (aRoom.getSaveIntermediateClouds())
                {
                    ss << "intermediate_cloud"<<std::setfill('0')<<std::setw(4)<<i<<".pcd";
                    intermediateCloudPath = roomFolder + ss.str().c_str();
                }
                xmlWriter->writeAttribute("filename",intermediateCloudPath);

                if(roomIntermediateCloudsLoaded[i] && aRoom.getSaveIntermediateClouds())
                {
                    QFile file(intermediateCloudPath);
                    if (!file.exists())
                    {
                        pcl::io::savePCDFileBinary(intermediateCloudPath.toStdString(), *roomIntermediateClouds[i]);
                        ROS_INFO_STREAM("Saving intermediate cloud file name "<<intermediateCloudPath.toStdString());
                    }
                }

                // RoomIntermediateCloudTransform
                xmlWriter->writeStartElement("RoomIntermediateCloudTransform");

                geometry_msgs::TransformStamped msg;
                tf::transformStampedTFToMsg(roomIntermediateCloudTransforms[i], msg);

                xmlWriter->writeStartElement("Stamp");
                xmlWriter->writeStartElement("sec");
                xmlWriter->writeCharacters(QString::number(msg.header.stamp.sec));
                xmlWriter->writeEndElement();
                xmlWriter->writeStartElement("nsec");
                xmlWriter->writeCharacters(QString::number(msg.header.stamp.nsec));
                xmlWriter->writeEndElement();
                xmlWriter->writeEndElement(); // Stamp

                xmlWriter->writeStartElement("FrameId");
                xmlWriter->writeCharacters(QString(msg.header.frame_id.c_str()));
                xmlWriter->writeEndElement();

                xmlWriter->writeStartElement("ChildFrameId");
                xmlWriter->writeCharacters(QString(msg.child_frame_id.c_str()));
                xmlWriter->writeEndElement();

                xmlWriter->writeStartElement("Transform");
                xmlWriter->writeStartElement("Translation");
                xmlWriter->writeStartElement("x");
                xmlWriter->writeCharacters(QString::number(msg.transform.translation.x));
                xmlWriter->writeEndElement();
                xmlWriter->writeStartElement("y");
                xmlWriter->writeCharacters(QString::number(msg.transform.translation.y));
                xmlWriter->writeEndElement();
                xmlWriter->writeStartElement("z");
                xmlWriter->writeCharacters(QString::number(msg.transform.translation.z));
                xmlWriter->writeEndElement();
                xmlWriter->writeEndElement(); // Translation
                xmlWriter->writeStartElement("Rotation");
                xmlWriter->writeStartElement("w");
                xmlWriter->writeCharacters(QString::number(msg.transform.rotation.w));
                xmlWriter->writeEndElement();
                xmlWriter->writeStartElement("x");
                xmlWriter->writeCharacters(QString::number(msg.transform.rotation.x));
                xmlWriter->writeEndElement();
                xmlWriter->writeStartElement("y");
                xmlWriter->writeCharacters(QString::number(msg.transform.rotation.y));
                xmlWriter->writeEndElement();
                xmlWriter->writeStartElement("z");
                xmlWriter->writeCharacters(QString::number(msg.transform.rotation.z));
                xmlWriter->writeEndElement();
                xmlWriter->writeEndElement(); // Rotation
                xmlWriter->writeEndElement(); // Transform

//                ROS_INFO_STREAM("TF message "<<msg<<"\nStamp "<<msg.header.stamp.sec<<"."<<msg.header.stamp.nsec);
                xmlWriter->writeEndElement(); // RoomIntermediateCloudTransform
                xmlWriter->writeEndElement(); // RoomIntermediateCloud
            }
//        }
        xmlWriter->writeEndElement(); // RoomIntermediateClouds

        xmlWriter->writeEndElement(); // Semantic Room

        xmlWriter->writeEndDocument();
        delete xmlWriter;

        return xmlFile;

    }

    static SemanticRoom<PointType> loadRoomFromXML(const std::string& xmlFile, bool deepLoad=true)
    {
        SemanticRoom<PointType> aRoom;


        QFile file(xmlFile.c_str());

        if (!file.exists())
        {
            std::cerr<<"Could not open file "<<xmlFile<<" to load room."<<std::endl;
            return aRoom;
        }

        file.open(QIODevice::ReadOnly);

        QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);
        Eigen::Vector4f centroid(0.0,0.0,0.0,0.0);

        while (!xmlReader->atEnd() && !xmlReader->hasError())
        {
            QXmlStreamReader::TokenType token = xmlReader->readNext();
            if (token == QXmlStreamReader::StartDocument)
                continue;

            if (xmlReader->hasError())
            {
                std::cout << "XML error: " << xmlReader->errorString().toStdString() << std::endl;
                return aRoom;
            }

            QString elementName = xmlReader->name().toString();

            if (token == QXmlStreamReader::StartElement)
            {
                if (xmlReader->name() == "RoomLogName")
                {
                    QString roomLogName = xmlReader->readElementText();
                    aRoom.setRoomLogName(roomLogName.toStdString());
                }

                if (xmlReader->name() == "RoomRunNumber")
                {
                    int roomRunNumber = xmlReader->readElementText().toInt();
                    aRoom.setRoomRunNumber(roomRunNumber);
                }

                if (xmlReader->name() == "RoomStringId")
                {
                    QString roomStringId = xmlReader->readElementText();
                    aRoom.setRoomStringId(roomStringId.toStdString());
                }

                if (xmlReader->name() == "RoomLogStartTime")
                {
                    QString roomLogStartTime = xmlReader->readElementText();
                    boost::posix_time::ptime roomStartTime = boost::posix_time::time_from_string(roomLogStartTime.toStdString());
                    aRoom.setRoomLogStartTime(roomStartTime);
                }

                if (xmlReader->name() == "RoomLogEndTime")
                {
                    QString roomLogEndTime = xmlReader->readElementText();
                    boost::posix_time::ptime roomEndTime = boost::posix_time::time_from_string(roomLogEndTime.toStdString());
                    aRoom.setRoomLogEndTime(roomEndTime);
                }

                if (xmlReader->name() == "RoomCompleteCloud")
                {
                    QXmlStreamAttributes attributes = xmlReader->attributes();
                    if (attributes.hasAttribute("filename"))
                    {
                        QString roomCompleteCloudFile = attributes.value("filename").toString();
                        if (deepLoad)
                        {
                            std::cout<<"Loading complete cloud file name "<<roomCompleteCloudFile.toStdString()<<std::endl;
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (roomCompleteCloudFile.toStdString(), *cloud);
                            aRoom.setCompleteRoomCloud(cloud);
                        } else {
                            aRoom.setCompleteRoomCloud(roomCompleteCloudFile.toStdString());
                        }
                    } else {
                        std::cerr<<"RoomCompleteCloud xml node does not have filename attribute. Aborting."<<std::endl;
                        return aRoom;
                    }
                }

                if (xmlReader->name() == "RoomInteriorCloud")
                {
                    QXmlStreamAttributes attributes = xmlReader->attributes();
                    if (attributes.hasAttribute("filename"))
                    {
                        QString roomInteriorCloudFile = attributes.value("filename").toString();
                        if (deepLoad)
                        {
                            std::cout<<"Loading interior cloud file name "<<roomInteriorCloudFile.toStdString()<<std::endl;
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (roomInteriorCloudFile.toStdString(), *cloud);
                            aRoom.setInteriorRoomCloud(cloud);
                        } else {
                            aRoom.setInteriorRoomCloud(roomInteriorCloudFile.toStdString());
                        }
                    } else {
                        std::cerr<<"RoomInteriorCloud xml node does not have filename attribute. Aborting."<<std::endl;
                        return aRoom;
                    }
                }

                if (xmlReader->name() == "RoomDeNoisedCloud")
                {
                    QXmlStreamAttributes attributes = xmlReader->attributes();
                    if (attributes.hasAttribute("filename"))
                    {
                        QString roomDenoisedCloudFile = attributes.value("filename").toString();
                        if (deepLoad)
                        {
                            std::cout<<"Loading denoised cloud file name "<<roomDenoisedCloudFile.toStdString()<<std::endl;
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (roomDenoisedCloudFile.toStdString(), *cloud);
                            aRoom.setDeNoisedRoomCloud(cloud);
                        } else {
                            aRoom.setDeNoisedRoomCloud(roomDenoisedCloudFile.toStdString());
                        }
                    } else {
                        std::cerr<<"RoomDeNoisedCloud xml node does not have filename attribute. Aborting."<<std::endl;
                        return aRoom;
                    }
                }

                if (xmlReader->name() == "RoomDynamicClusters")
                {
                    QXmlStreamAttributes attributes = xmlReader->attributes();
                    if (attributes.hasAttribute("filename"))
                    {
                        QString roomDynamicClustersFile = attributes.value("filename").toString();
                        if (deepLoad)
                        {
                            std::cout<<"Loading dynamic clusters cloud file name "<<roomDynamicClustersFile.toStdString()<<std::endl;
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (roomDynamicClustersFile.toStdString(), *cloud);
                            aRoom.setDynamicClustersCloud(cloud);
                        } else {
                            aRoom.setDynamicClustersCloud(roomDynamicClustersFile.toStdString());
                        }
                    } else {
                        std::cerr<<"RoomDynamicClusters xml node does not have filename attribute. Aborting."<<std::endl;
                        return aRoom;
                    }
                }

                if (xmlReader->name() == "Centroid")
                {
                    QString centroidS = xmlReader->readElementText();
                    Eigen::Vector4f centroid;
                    QStringList centroidSlist = centroidS.split(' ');
                    centroid(0) = centroidSlist[0].toDouble();centroid(1) = centroidSlist[1].toDouble();
                    centroid(2) = centroidSlist[2].toDouble();centroid(3) = centroidSlist[3].toDouble();
                    aRoom.setCentroid(centroid);
                }

                if (xmlReader->name() == "RoomTransform")
                {
                    QString transformS = xmlReader->readElementText();
                    Eigen::Matrix4f transform;
                    QStringList transformSlist = transformS.split(' ');
                    for (size_t i=0; i<16;i++)
                    {
                        transform(i)=transformSlist[i].toDouble();
                    }
                    aRoom.setRoomTransform(transform);
                }



                if (xmlReader->name() == "RoomIntermediateCloud")
                {
                    std::pair<std::string, tf::StampedTransform> cloudAndTransform = parseRoomIntermediateCloudNode(*xmlReader);
                    if (deepLoad)
                    {
                        std::cout<<"Loading intermediate cloud file name "<<cloudAndTransform.first<<std::endl;
                        pcl::PCDReader reader;
                        CloudPtr cloud (new Cloud);
                        reader.read (cloudAndTransform.first, *cloud);
                        aRoom.addIntermediateRoomCloud(cloud, cloudAndTransform.second);
                    } else {
                        aRoom.addIntermediateRoomCloud(cloudAndTransform.first, cloudAndTransform.second);
                    }

                }
            }
        }

        delete xmlReader;


        return aRoom;
    }

private:

    static std::pair<std::string, tf::StampedTransform> parseRoomIntermediateCloudNode(QXmlStreamReader& xmlReader)
    {        
        tf::StampedTransform transform;
        geometry_msgs::TransformStamped tfmsg;
        std::pair<std::string, tf::StampedTransform> toRet;
//        toRet.first = CloudPtr(new Cloud);
        QString intermediateParentNode("");

        if (xmlReader.name()!="RoomIntermediateCloud")
        {
            std::cerr<<"Cannot parse RoomIntermediateCloud node, it has a different name: "<<xmlReader.name().toString().toStdString()<<std::endl;
        }
        QXmlStreamAttributes attributes = xmlReader.attributes();
        if (attributes.hasAttribute("filename"))
        {
            QString roomIntermediateCloudFile = attributes.value("filename").toString();            
            toRet.first = roomIntermediateCloudFile.toStdString();
//            pcl::PCDReader reader;
//            reader.read (roomIntermediateCloudFile.toStdString(), *toRet.first);

        } else {
            std::cerr<<"RoomIntermediateCloud xml node does not have filename attribute. Aborting."<<std::endl;
            return toRet;
        }
        QXmlStreamReader::TokenType token = xmlReader.readNext();

        while(!((token == QXmlStreamReader::EndElement) && (xmlReader.name() == "RoomIntermediateCloud")) )
        {

            if (token == QXmlStreamReader::StartElement)
            {
                if (xmlReader.name() == "sec")
                {
                    int sec = xmlReader.readElementText().toInt();
                    tfmsg.header.stamp.sec = sec;
                }
                if (xmlReader.name() == "nsec")
                {
                    int nsec = xmlReader.readElementText().toInt();
                    tfmsg.header.stamp.nsec = nsec;
                }
                if (xmlReader.name() == "Translation")
                {
                    intermediateParentNode = xmlReader.name().toString();
                }
                if (xmlReader.name() == "Rotation")
                {
                    intermediateParentNode = xmlReader.name().toString();
                }
                if (xmlReader.name() == "w")
                {
                    double w = xmlReader.readElementText().toDouble();
                    tfmsg.transform.rotation.w = w;
                }
                if (xmlReader.name() == "x")
                {
                    double x = xmlReader.readElementText().toDouble();
                    if (intermediateParentNode == "Rotation")
                    {
                        tfmsg.transform.rotation.x = x;
                    }
                    if (intermediateParentNode == "Translation")
                    {
                        tfmsg.transform.translation.x = x;
                    }
                }
                if (xmlReader.name() == "y")
                {
                    double y = xmlReader.readElementText().toDouble();
                    if (intermediateParentNode == "Rotation")
                    {
                        tfmsg.transform.rotation.y = y;
                    }
                    if (intermediateParentNode == "Translation")
                    {
                        tfmsg.transform.translation.y = y;
                    }
                }
                if (xmlReader.name() == "z")
                {
                    double z = xmlReader.readElementText().toDouble();
                    if (intermediateParentNode == "Rotation")
                    {
                        tfmsg.transform.rotation.z = z;
                    }
                    if (intermediateParentNode == "Translation")
                    {
                        tfmsg.transform.translation.z = z;
                    }
                }
            }
            token = xmlReader.readNext();
        }

        tf::transformStampedMsgToTF(tfmsg, transform);
//        toRet.first = cloud;
        toRet.second = transform;
        return toRet;
    }

    QString                                 m_RootFolder;
};

#endif
