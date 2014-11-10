#ifndef __SIMPLE_SUMMARY_PARSER__H
#define __SIMPLE_SUMMARY_PARSER__H

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

template <class PointType>
class SimpleSummaryParser {

public:
    struct EntityStruct{
        std::string roomXmlFile;

        EntityStruct()
        {
            roomXmlFile="";
        }
    };

private:
    QString                                 m_XMLFile;
    std::vector<EntityStruct>               m_vAllRooms;

public:

    SimpleSummaryParser(std::string xmlFile="")
    {
        if (xmlFile == "")
        {
            m_XMLFile =(QDir::homePath() + QString("/.semanticMap/index.xml"));
        } else {
            m_XMLFile = QString(xmlFile.c_str());
        }

    }

    ~SimpleSummaryParser()
    {

    }

    std::vector<EntityStruct> getRooms()
    {
        return m_vAllRooms;
    }


    bool createSummaryXML(std::string rootFolder="")
    {
        QString qrootFolder;
        if (rootFolder == "")
        {
            qrootFolder =(QDir::homePath() + QString("/.semanticMap/"));
        } else {
            qrootFolder = rootFolder.c_str();
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

        xmlWriter->writeEndElement(); // SemanticMap
        xmlWriter->writeEndDocument();

        ROS_INFO_STREAM("Created semantic map summary xml.");


        return true;
    }


private:

    void saveSemanticRooms(QXmlStreamWriter* xmlWriter, QString qrootFolder)
    {
        xmlWriter->writeStartElement("SemanticRooms");

        // parse folder structure and look for semantic objects
        QStringList dateFolders = QDir(qrootFolder).entryList(QStringList("*"),
                                                        QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot,QDir::Time);

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
                std::cout<<"Skipping folder "<<dateFolders[i].toStdString()<<" as it doesn't have the right format."<<std::endl;
                continue;
            }

            QString dateFolder = qrootFolder+dateFolders[i];
            QStringList patrolFolders = QDir(dateFolder).entryList(QStringList("*"),
                                                                    QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot,QDir::Time| QDir::Reversed);
            for (size_t j=0; j<patrolFolders.size(); j++)
            {
                QString patrolFolder = dateFolder + "/" + patrolFolders[j];
                QStringList roomFolders = QDir(patrolFolder).entryList(QStringList("*"),
                                                                     QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot,QDir::Time| QDir::Reversed);
                for (size_t k=0; k<roomFolders.size(); k++)
                {

                    // parse XML file and extract some important fields with which to populate the index.html file
                    QString roomXmlFile = patrolFolder+"/"+roomFolders[k] + "/room.xml";

                    xmlWriter->writeStartElement("RoomXMLFile");
                    xmlWriter->writeCharacters(roomXmlFile);
                    xmlWriter->writeEndElement();

                    xmlWriter->writeEndElement();
                    ROS_INFO_STREAM("Added room "<<roomXmlFile.toStdString());

                    EntityStruct aRoom;
                    aRoom.roomXmlFile = roomXmlFile.toStdString();
                    m_vAllRooms.push_back(aRoom);
                }
            }
        }

        xmlWriter->writeEndElement(); // SemanticRooms
    }

private:

};


#endif
