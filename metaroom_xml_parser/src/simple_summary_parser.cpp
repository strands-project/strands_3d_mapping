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


bool SimpleSummaryParser::createSummaryXML(std::string rootFolder, bool verbose)
{
    ROS_INFO_STREAM("Reading observations from "<<rootFolder);
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

    saveSemanticRooms(xmlWriter, qrootFolder, verbose);

    xmlWriter->writeEndElement(); // SemanticMap
    xmlWriter->writeEndDocument();

    ROS_INFO_STREAM("Done looking for observations. Found "<<m_vAllRooms.size()<<" observations.");

    return true;
}

void SimpleSummaryParser::saveSemanticRooms(QXmlStreamWriter* xmlWriter, QString qrootFolder, bool verbose)
{
    xmlWriter->writeStartElement("SemanticRooms");

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
            if (verbose)
            {
                ROS_INFO_STREAM("Added room "<<roomXmlFile.toStdString());
            }

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
