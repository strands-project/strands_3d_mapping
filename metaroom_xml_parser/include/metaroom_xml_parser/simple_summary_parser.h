#ifndef __SIMPLE_SUMMARY_PARSER__H
#define __SIMPLE_SUMMARY_PARSER__H

//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>

#include "ros/time.h"
#include "ros/serialization.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tf/tf.h"

#include <QFile>
#include <QDir>
#include <QXmlStreamWriter>

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
    int                                     m_maxFolderDepth;

public:

    SimpleSummaryParser(std::string xmlFile="");

    ~SimpleSummaryParser();

    std::vector<EntityStruct> getRooms();

    bool createSummaryXML(std::string rootFolder="", bool verbose = false);


private:

    void saveSemanticRooms(QXmlStreamWriter* xmlWriter, QString qrootFolder, bool verbose);

    std::vector<QString> listXmlInFolder(QString qrootFolder, int depth);


};


#endif
