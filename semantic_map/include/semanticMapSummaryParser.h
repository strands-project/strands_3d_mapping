#ifndef __SEMANTIC_MAP_SUMMARY_PARSER__H
#define __SEMANTIC_MAP_SUMMARY_PARSER__H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "ros/time.h"
#include "ros/serialization.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tf/tf.h"

#include <QFile>
#include <QDir>
#include <QXmlStreamWriter>

#include <room.h>
#include "metaroom.h"
#include "roomXMLparser.h"
#include "metaroomXMLparser.h"
#include "constants.h"


template <class PointType>
class SemanticMapSummaryParser {

public:
    struct EntityStruct{
        std::string roomXmlFile;
        std::string roomLogName;
        boost::posix_time::ptime roomLogStartTime, roomLogEndTime;
        Eigen::Vector4f centroid;
        bool    hasCentroid;
        bool    isMetaRoom;
        SEMANTIC_MAP_ENTITY_TYPE entityType;

        EntityStruct()
        {
            roomXmlFile="";
            roomLogName="";
            hasCentroid = false;
            centroid = Eigen::Vector4f::Zero();
            isMetaRoom = false;
        }
    };

private:
    QString                                 m_XMLFile;
    std::vector<EntityStruct>               m_vAllRooms;
    bool                                    m_bIsInitialized;

public:

    SemanticMapSummaryParser(std::string xmlFile="")
    {
        if (xmlFile == "")
        {
            m_XMLFile =(QDir::homePath() + QString("/.semanticMap/index.xml"));
        } else {
            m_XMLFile = QString(xmlFile.c_str());
        }

        m_bIsInitialized = false;
    }

    ~SemanticMapSummaryParser()
    {

    }

    void refresh()
    {
        m_vAllRooms = parseSummaryXML();

    }

    std::vector<EntityStruct> getMetaRooms()
    {
        std::vector<EntityStruct> toRet;

        if (!m_bIsInitialized)
        {
            refresh();
        }

        for (size_t i=0; i<m_vAllRooms.size();i++)
        {
            if (m_vAllRooms[i].entityType == SEMANTIC_MAP_METAROOM)
            {
                toRet.push_back(m_vAllRooms[i]);
            }
        }

        return toRet;
    }

    std::vector<EntityStruct> getRooms()
    {
        std::vector<EntityStruct> toRet;

        if (!m_bIsInitialized)
        {
            refresh();
        }

        for (size_t i=0; i<m_vAllRooms.size();i++)
        {
            if (m_vAllRooms[i].entityType == SEMANTIC_MAP_ROOM)
            {
                toRet.push_back(m_vAllRooms[i]);
            }
        }
    }


    std::vector<EntityStruct> parseSummaryXML()
    {

        std::vector<EntityStruct> toRet;
        QFile file(m_XMLFile);

        if (!file.exists())
        {
            std::cerr<<"Could not open file "<<m_XMLFile.toStdString()<<" to summary data."<<std::endl;
            return toRet;
        }

        file.open(QIODevice::ReadOnly);

        QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);

        while (!xmlReader->atEnd() && !xmlReader->hasError())
        {
            QXmlStreamReader::TokenType token = xmlReader->readNext();
            if (token == QXmlStreamReader::StartDocument)
                continue;

            if (xmlReader->hasError())
            {
                std::cout << "XML error: " << xmlReader->errorString().toStdString() << std::endl;
                return toRet;
            }

            QString elementName = xmlReader->name().toString();

            if (token == QXmlStreamReader::StartElement)
            {
                if (xmlReader->name() == "SemanticRoom")
                { // parse semantic room data
                    EntityStruct aEntityStruct;
                    aEntityStruct.entityType = SEMANTIC_MAP_ROOM;
                    aEntityStruct.hasCentroid = false;
                    token = xmlReader->readNext();
                    while(!((token == QXmlStreamReader::EndElement) && (xmlReader->name() == "SemanticRoom")) )
                    {
                        if (xmlReader->name() == "RoomLogName")
                        {
                            aEntityStruct.roomLogName = xmlReader->readElementText().toStdString();
                        }
                        if (xmlReader->name() == "RoomLogStartTime")
                        {
                            QString roomLogStartTime = xmlReader->readElementText();
                            boost::posix_time::ptime roomStartTime = boost::posix_time::time_from_string(roomLogStartTime.toStdString());
                            aEntityStruct.roomLogStartTime = roomStartTime;
                        }

                        if (xmlReader->name() == "RoomLogEndTime")
                        {
                            QString roomLogEndTime = xmlReader->readElementText();
                            boost::posix_time::ptime roomEndTime = boost::posix_time::time_from_string(roomLogEndTime.toStdString());
                            aEntityStruct.roomLogEndTime = roomEndTime;
                        }
                        if (xmlReader->name() == "RoomXMLFile")
                        {
                            aEntityStruct.roomXmlFile = xmlReader->readElementText().toStdString();
                        }

                        if (xmlReader->name() == "RoomCentroid")
                        {
                            QString centroidS = xmlReader->readElementText();
                            Eigen::Vector4f centroid;
                            QStringList centroidSlist = centroidS.split(' ');
                            centroid(0) = centroidSlist[0].toDouble();centroid(1) = centroidSlist[1].toDouble();
                            centroid(2) = centroidSlist[2].toDouble();centroid(3) = centroidSlist[3].toDouble();
                            aEntityStruct.centroid = centroid;
                        }
                        token = xmlReader->readNext();
                    }
                    toRet.push_back(aEntityStruct);
                }

                if (xmlReader->name() == "MetaRoom")
                { // parse semantic room data
                    EntityStruct aEntityStruct;
                    aEntityStruct.entityType = SEMANTIC_MAP_METAROOM;
                    aEntityStruct.hasCentroid = false;
                    token = xmlReader->readNext();
                    while(!((token == QXmlStreamReader::EndElement) && (xmlReader->name() == "MetaRoom")) )
                    {
                        if (xmlReader->name() == "MetaRoomXMLFile")
                        {
                            aEntityStruct.roomXmlFile = xmlReader->readElementText().toStdString();
                        }

                        if (xmlReader->name() == "MetaRoomCentroid")
                        {
                            QString centroidS = xmlReader->readElementText();
                            Eigen::Vector4f centroid;
                            QStringList centroidSlist = centroidS.split(' ');
                            centroid(0) = centroidSlist[0].toDouble();centroid(1) = centroidSlist[1].toDouble();
                            centroid(2) = centroidSlist[2].toDouble();centroid(3) = centroidSlist[3].toDouble();
                            aEntityStruct.centroid = centroid;
                        }
                        token = xmlReader->readNext();
                    }
                    toRet.push_back(aEntityStruct);
                }
            }
        }

        m_bIsInitialized = true;
        return toRet;
    }

    bool createSummaryXML(std::string rootFolder="")
    {
        QString qrootFolder;
        if (rootFolder == "")
        {
            qrootFolder =(QDir::homePath() + QString("/.semanticMap/"));
        }

        if (!QDir(qrootFolder).exists())
        {
            ROS_INFO_STREAM("The root folder for the semantic map doesn't exist. "<<qrootFolder.toStdString()<<" Exitting.");
        }

        if (QFile(m_XMLFile).exists())
        {
            QFile(m_XMLFile).remove();
        }

        QFile file(m_XMLFile);

        if (!file.open(QIODevice::ReadWrite | QIODevice::Text))
        {
            std::cerr<<"Could not open file "<<m_XMLFile.toStdString()<<" to save room as XML"<<std::endl;
            return "";
        }

        QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
        xmlWriter->setDevice(&file);

        xmlWriter->writeStartDocument();
        xmlWriter->writeStartElement("SemanticMap");

        saveSemanticRooms(xmlWriter, qrootFolder);
        saveMetaRooms(xmlWriter,qrootFolder);


        xmlWriter->writeEndElement(); // SemanticMap
        xmlWriter->writeEndDocument();


        return true;
    }


private:

    void saveMetaRooms(QXmlStreamWriter* xmlWriter, QString qrootFolder)
    {
        xmlWriter->writeStartElement("MetaRooms");

        QStringList metaroomRootFolders = QDir(qrootFolder).entryList(QStringList("*"),
                                                        QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);

        for (size_t i=0; i<metaroomRootFolders.size(); i++)
        {
            if (metaroomRootFolders[i].indexOf("metarooms") == 1)
            {
                // skip this folder
                continue;
            } else {
                // this is the folder containing the metarooms
                QString folders = qrootFolder + metaroomRootFolders[i]+"/";
                QStringList metaroomFolders = QDir(folders).entryList(QStringList("*"),
                                                                QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);

                for (size_t j=0; j<metaroomFolders.size();j++)
                {
                    if (metaroomFolders[j].indexOf("metaroom") == -1)
                    {
                        // not a metaroom folder
//                        ROS_INFO_STREAM("Skipping folder "<<metaroomFolders[j].toStdString()<<". Not a proper metaroom folder.");
                        continue;
                    } else {
                        QString tempMetaRoomFolder = folders+metaroomFolders[j]+"/";
                        QStringList tempMetaRoomFolderFiles = QDir(tempMetaRoomFolder).entryList(QStringList("*.xml"));
                        if (tempMetaRoomFolderFiles.size() == 0)
                        {
                            // no .xml files in this folder
                            ROS_INFO_STREAM("No xml files in metaroom folder "<<tempMetaRoomFolder.toStdString()<<". Skipping.");
                            continue;
                        } else {
                            QString savedMetaRoomXMLFile = tempMetaRoomFolder+tempMetaRoomFolderFiles[0]; // TODO for now I am assuming there is only 1 xml file in the metaroom folder. Maybe later there will be more.
                            MetaRoom<PointType> savedMetaRoom = MetaRoomXMLParser<PointType>::loadMetaRoomFromXML(savedMetaRoomXMLFile.toStdString(),false);
                            xmlWriter->writeStartElement("MetaRoom");

                            xmlWriter->writeStartElement("MetaRoomXMLFile");
                            xmlWriter->writeCharacters(savedMetaRoomXMLFile);
                            xmlWriter->writeEndElement();

                            xmlWriter->writeStartElement("MetaRoomCentroid");
                            Eigen::Vector4f centroid = savedMetaRoom.getCentroid();
                            QString centroidS = QString::number(centroid(0))+" "+QString::number(centroid(1))+" "+QString::number(centroid(2))+" "+QString::number(centroid(3));
                            xmlWriter->writeCharacters(centroidS);
                            xmlWriter->writeEndElement();

                            xmlWriter->writeEndElement(); // MetaRoom

                        }
                    }
                }

            }
        }

        xmlWriter->writeEndElement();
    }

    void saveSemanticRooms(QXmlStreamWriter* xmlWriter, QString qrootFolder)
    {
        xmlWriter->writeStartElement("SemanticRooms");


        // parse folder structure and look for semantic objects
        QStringList dateFolders = QDir(qrootFolder).entryList(QStringList("*"),
                                                        QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);

        for (size_t i=0; i<dateFolders.size(); i++)
        {
            bool isValidDate = true;
            // check that this is indeed a data folder (there might be other folders)
            if (dateFolders[i].size() != 8 )
            {
                isValidDate = false;
            } else {
                for (size_t letter_count = 0; letter_count<dateFolders[i].size(); letter_count++)
                {
                    if (!dateFolders[i].at(letter_count).isDigit())
                    {
                        isValidDate = false;
                        break;
                    }
                }
            }

            if (!isValidDate)
            {
//                std::cout<<"Skipping folder "<<dateFolders[i].toStdString()<<" as it doesn't have the right format."<<std::endl;
                continue;
            }

            QString dateFolder = qrootFolder+dateFolders[i];
            QStringList patrolFolders = QDir(dateFolder).entryList(QStringList("*"),
                                                                    QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);
            for (size_t j=0; j<patrolFolders.size(); j++)
            {
                QString patrolFolder = dateFolder + "/" + patrolFolders[j];
                QStringList roomFolders = QDir(patrolFolder).entryList(QStringList("*"),
                                                                     QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);
                for (size_t k=0; k<roomFolders.size(); k++)
                {
                    // parse XML file and extract some important fields with which to populate the index.html file
                    QString roomXmlFile = patrolFolder+"/"+roomFolders[k] + "/room.xml";
                    SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(roomXmlFile.toStdString(), false);
                    xmlWriter->writeStartElement("SemanticRoom");

                    xmlWriter->writeStartElement("RoomLogName");
                    xmlWriter->writeCharacters(QString(aRoom.getRoomLogName().c_str()));
                    xmlWriter->writeEndElement();

                    xmlWriter->writeStartElement("RoomLogStartTime");
                    boost::posix_time::ptime roomLogStartTime = aRoom.getRoomLogStartTime();
                    xmlWriter->writeCharacters(boost::posix_time::to_simple_string(roomLogStartTime).c_str());
                    xmlWriter->writeEndElement();

                    xmlWriter->writeStartElement("RoomLogEndTime");
                    boost::posix_time::ptime roomLogEndTime = aRoom.getRoomLogEndTime();
                    xmlWriter->writeCharacters(boost::posix_time::to_simple_string(roomLogEndTime).c_str());
                    xmlWriter->writeEndElement();

                    xmlWriter->writeStartElement("RoomXMLFile");
                    xmlWriter->writeCharacters(roomXmlFile);
                    xmlWriter->writeEndElement();

                    xmlWriter->writeStartElement("RoomCentroid");
                    Eigen::Vector4f centroid = aRoom.getCentroid();
                    QString centroidS = QString::number(centroid(0))+" "+QString::number(centroid(1))+" "+QString::number(centroid(2))+" "+QString::number(centroid(3));
                    xmlWriter->writeCharacters(centroidS);
                    xmlWriter->writeEndElement();

                    xmlWriter->writeEndElement();
                    ROS_INFO_STREAM("Added room "<<roomXmlFile.toStdString());
                }
            }
        }

        xmlWriter->writeEndElement(); // SemanticRooms
    }

};


#endif
