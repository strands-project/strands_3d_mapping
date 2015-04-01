#include "dynamic_object_xml_parser.h"

#include <pcl/io/pcd_io.h>

using namespace std;

DynamicObjectXMLParser::DynamicObjectXMLParser(std::string rootFolder, bool verbose ) : m_rootFolderPath(rootFolder),  m_verbose(verbose)
{

}

DynamicObjectXMLParser::~DynamicObjectXMLParser()
{

}

std::string DynamicObjectXMLParser::saveAsXML(DynamicObject::Ptr object, std::string filename, std::string cloud_filename)
{
    if (filename == "")
    {
        filename = object->m_label+".xml";
    }
    if (cloud_filename == "")
    {
        cloud_filename = object->m_label+".pcd";
    }

   string path = m_rootFolderPath + "/" + filename;
   string cloud_path = m_rootFolderPath + "/" + cloud_filename;

   if (m_verbose)
   {
       cout<<"Saving dynamic object xml at "<<path<<"  and point cloud at "<<cloud_path<<". Point cloud size "<<object->m_points->points.size()<<endl;
   }

   QFile file(path.c_str());
   if (file.exists())
   {
       file.remove();
   }

   if (!file.open(QIODevice::ReadWrite | QIODevice::Text))
   {
       std::cerr<<"Could not open file "<<path<<" to save object as XML"<<std::endl;
       return "";
   }

   QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
   xmlWriter->setDevice(&file);

   xmlWriter->writeStartDocument();

   xmlWriter->writeStartElement("DynamicObject");
   xmlWriter->writeAttribute("label",object->m_label.c_str());
   xmlWriter->writeAttribute("roomLogString",object->m_roomLogString.c_str());
   xmlWriter->writeAttribute("roomStringId",object->m_roomStringId.c_str());
   xmlWriter->writeAttribute("filename",cloud_filename.c_str());
   pcl::io::savePCDFileBinary(cloud_path, *object->m_points);


   xmlWriter->writeStartElement("Centroid");
   Eigen::Vector4f centroid = object->m_centroid;
   QString centroidS = QString::number(centroid(0))+" "+QString::number(centroid(1))+" "+QString::number(centroid(2))+" "+QString::number(centroid(3));
   xmlWriter->writeCharacters(centroidS);
   xmlWriter->writeEndElement(); // Centroid

   xmlWriter->writeStartElement("LogTime");
   boost::posix_time::ptime logTime = object->m_time;
   xmlWriter->writeCharacters(boost::posix_time::to_simple_string(logTime).c_str());
   xmlWriter->writeEndElement();// LogTime

   xmlWriter->writeEndElement(); // DynamicObject

   xmlWriter->writeEndDocument();

   delete xmlWriter;

   if (m_verbose)
   {
      cout<<"Saved object at: "<<path<<endl;
   }

   return path;
}

DynamicObject::Ptr DynamicObjectXMLParser::loadFromXML(string filename)
{
   DynamicObject::Ptr object(new DynamicObject());
   QFile file(filename.c_str());

   if (!file.exists())
   {
       std::cerr<<"Could not open file "<<filename<<" to load object."<<std::endl;
       return object;
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
           return object;
       }

       if (token == QXmlStreamReader::StartElement)
       {
           if (xmlReader->name() == "DynamicObject")
           {
              QXmlStreamAttributes attributes = xmlReader->attributes();
              if (attributes.hasAttribute("label"))
              {
                  QString labelS = attributes.value("label").toString();
                  object->m_label = labelS.toStdString();
              } else {
                  std::cerr<<"Object xml node does not have label attribute. Aborting."<<std::endl;
                  return object;
              }
              if (attributes.hasAttribute("roomLogString"))
              {
                  QString labelS = attributes.value("roomLogString").toString();
                  object->m_roomLogString = labelS.toStdString();
              } else {
                  std::cerr<<"Object xml node does not have roomLogString attribute."<<std::endl; // leaving blank
              }
              if (attributes.hasAttribute("roomStringId"))
              {
                  QString labelS = attributes.value("roomStringId").toString();
                  object->m_roomStringId = labelS.toStdString();
              } else {
                  std::cerr<<"Object xml node does not have roomStringId attribute."<<std::endl; // leaving blank
              }


              if (attributes.hasAttribute("filename"))
              {
                  QString fileS = objectFolder + "/" + attributes.value("filename").toString();

                  pcl::PCDReader reader;
                  CloudPtr cloud (new Cloud);
                  reader.read (fileS.toStdString(), *cloud);
                  object->setCloud(cloud);

              } else {
                  std::cerr<<"Object xml node does not have filename attribute. Aborting."<<std::endl;
                  return object;
              }
           }

           if (xmlReader->name() == "Centroid")
           {
               QString centroidS = xmlReader->readElementText();
               Eigen::Vector4f centroid;
               QStringList centroidSlist = centroidS.split(' ');
               centroid(0) = centroidSlist[0].toDouble();centroid(1) = centroidSlist[1].toDouble();
               centroid(2) = centroidSlist[2].toDouble();centroid(3) = centroidSlist[3].toDouble();
               object->m_centroid = centroid;
           }

           if (xmlReader->name() == "LogTime")
           {
               QString logTime = xmlReader->readElementText();
               boost::posix_time::ptime objectTime = boost::posix_time::time_from_string(logTime.toStdString());
               object->m_time = objectTime;
           }
       }
   }

   if (m_verbose)
   {
      cout<<"Loaded object from: "<<filename<<endl;
   }

   return object;
}
