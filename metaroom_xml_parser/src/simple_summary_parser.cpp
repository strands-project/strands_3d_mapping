#include "simple_summary_parser.h"

SimpleSummaryParser::SimpleSummaryParser(std::string xmlFile)
{
    if (xmlFile == "")
    {
        m_XMLFile =(QDir::homePath() + QString("/.semanticMap/index.xml"));
    } else {
        m_XMLFile = QString(xmlFile.c_str());
    }

    m_maxFolderDepth = 10; // expand to a maximum of 10 folders down while looking for room xml files.

}

SimpleSummaryParser::~SimpleSummaryParser()
{

}

std::vector<SimpleSummaryParser::EntityStruct> SimpleSummaryParser::getRooms()
{
    return m_vAllRooms;
}


bool SimpleSummaryParser::createSummaryXML(std::string rootFolder)
{
    rootFolder+=std::string("/"); // just to make sure, doesn't matter if there are two /

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

void SimpleSummaryParser::saveSemanticRooms(QXmlStreamWriter* xmlWriter, QString qrootFolder)
{
    xmlWriter->writeStartElement("SemanticRooms");

//    // parse folder structure and look for semantic objects
//    QStringList dateFolders = QDir(qrootFolder).entryList(QStringList("*"),
//                                                    QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot,QDir::Time);

//    for (size_t i=0; i<dateFolders.size(); i++)
//    {
//        bool isValidDate = true;
//        // check that this is indeed a data folder (there might be other folders)
//        if (dateFolders[i].size() != 8 )
//        {
//            isValidDate = false;
//        } else {
//            for (size_t letter_count = 0; letter_count<dateFolders[i].size(); letter_count++)
//            {
//                if (!dateFolders[i].at(letter_count).isDigit())
//                {
//                    isValidDate = false;
//                    break;
//                }
//            }
//        }

//        if (!isValidDate)
//        {
//            std::cout<<"Skipping folder "<<dateFolders[i].toStdString()<<" as it doesn't have the right format."<<std::endl;
//            continue;
//        }

//        QString dateFolder = qrootFolder+dateFolders[i];
//        QStringList patrolFolders = QDir(dateFolder).entryList(QStringList("*"),
//                                                                QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot,QDir::Time| QDir::Reversed);
//        for (size_t j=0; j<patrolFolders.size(); j++)
//        {
//            QString patrolFolder = dateFolder + "/" + patrolFolders[j];
//            QStringList roomFolders = QDir(patrolFolder).entryList(QStringList("*"),
//                                                                 QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot,QDir::Time| QDir::Reversed);
//            for (size_t k=0; k<roomFolders.size(); k++)
//            {

//                // parse XML file and extract some important fields with which to populate the index.html file
//                QString roomXmlFile = patrolFolder+"/"+roomFolders[k] + "/room.xml";

//                xmlWriter->writeStartElement("RoomXMLFile");
//                xmlWriter->writeCharacters(roomXmlFile);
//                xmlWriter->writeEndElement();

//                xmlWriter->writeEndElement();
//                ROS_INFO_STREAM("Added room "<<roomXmlFile.toStdString());

//                EntityStruct aRoom;
//                aRoom.roomXmlFile = roomXmlFile.toStdString();
//                m_vAllRooms.push_back(aRoom);
//            }
//        }
//    }

    std::vector<QString> allXmls = listXmlInFolder(qrootFolder,0);

    for(QString roomXmlFile : allXmls)
    {
        // test that this xml file actually contains a semantic room / sweep
        QFile file(roomXmlFile);
        file.open(QIODevice::ReadOnly);

        QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);
        QXmlStreamReader::TokenType token = xmlReader->readNext();
        int max_token = 15;
        int current_token=0;
        while ((token != QXmlStreamReader::StartElement) && (current_token<max_token))
        {
            current_token++;
            token = xmlReader->readNext();
        }

        if (current_token > max_token)
        {
            continue;
        }

        if (xmlReader->name() == "SemanticRoom")
        {
            xmlWriter->writeStartElement("RoomXMLFile");
            xmlWriter->writeCharacters(roomXmlFile);
            xmlWriter->writeEndElement();

            xmlWriter->writeEndElement();
            ROS_INFO_STREAM("Added room "<<roomXmlFile.toStdString());

            EntityStruct aRoom;
            aRoom.roomXmlFile = roomXmlFile.toStdString();
            m_vAllRooms.push_back(aRoom);
        }

        file.close();
    }


    xmlWriter->writeEndElement(); // SemanticRooms
}

std::vector<QString> SimpleSummaryParser::listXmlInFolder(QString qrootFolder, int depth)
{
    if (depth > m_maxFolderDepth)
    {
        return std::vector<QString>();
    }


    std::vector<QString> toRet;

    QStringList childFolders = QDir(qrootFolder).entryList(QStringList("*"),
                                                    QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot,QDir::Time);

    QStringList filters;
    filters << "*.xml";

    for ( QString file : QDir(qrootFolder).entryList(filters, QDir::Files) )
    {
        toRet.push_back(qrootFolder+file);
    }

    for(QString childFolder : childFolders)
    {
        std::vector<QString> childXmls = listXmlInFolder(qrootFolder+childFolder+"/", depth+1);
        toRet.insert(toRet.end(),childXmls.begin(), childXmls.end());
    }

    return toRet;
}
