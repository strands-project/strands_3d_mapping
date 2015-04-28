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
    xmlWriter->writeAttribute("roomRunNumber",QString::number(object->m_roomRunNumber));
    xmlWriter->writeAttribute("filename",cloud_filename.c_str());
    xmlWriter->writeAttribute("additionalViews",QString::number(object->m_noAdditionalViews));
    xmlWriter->writeAttribute("objectTracks",QString::number(object->m_vObjectTracks.size()));
    pcl::io::savePCDFileBinary(cloud_path, *object->m_points);

    // save additional views
    if (object->m_vAdditionalViews.size() > 0)
    {
        if (object->m_vAdditionalViews.size() != object->m_noAdditionalViews)
        {
            std::cerr<<"Number of additional views is set to "<<object->m_noAdditionalViews<<" however only "<<object->m_vAdditionalViews.size()<<" loaded in memory."<<std::endl;
        }

        for (size_t i=0; i<object->m_vAdditionalViews.size(); i++)
        {
            stringstream ss;ss<<i;
            string view_path = m_rootFolderPath + "/" + object->m_label + "_additional_view_"+ss.str()+".pcd";
            pcl::io::savePCDFileBinary(view_path, *object->m_vAdditionalViews[i]);
        }
    }

    // save object tracks
    xmlWriter->writeStartElement("ObjectTracks");
    for (size_t i=0; i<object->m_vObjectTracks.size(); i++)
    {
        stringstream ss; ss<<i;
        string track_cloud = m_rootFolderPath + "/" + object->m_label + "_object_track_"+ss.str()+".pcd";
        saveObjectTrackToXml(object->m_vObjectTracks[i].pose, object->m_vObjectTracks[i].cloud,xmlWriter, "ObjectTrack",track_cloud);
    }
    xmlWriter->writeEndElement(); // ObjectTracks


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

DynamicObject::Ptr DynamicObjectXMLParser::loadFromXML(string filename, bool load_cloud)
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
                if (attributes.hasAttribute("roomRunNumber"))
                {
                    object->m_roomRunNumber = attributes.value("roomRunNumber").toString().toInt();
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
                        object->setCloud(cloud);
                    }

                } else {
                    std::cerr<<"Object xml node does not have filename attribute. Aborting."<<std::endl;
                    return object;
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
                            string view_path = objectFolder.toStdString() + "/" + object->m_label + "_additional_view_"+ss.str()+".pcd";
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (view_path, *cloud);
                            if (cloud->points.size() != 0)
                            {
                                object->addAdditionalView(cloud);
                                if (m_verbose)
                                {
                                    std::cout<<"Loaded additional view cloud "<<view_path<<std::endl;
                                }
                            } else {
                                std::cerr<<"Could not load additional view cloud "<<view_path<<std::endl;
                            }
                        }
                    } else {
                        object->m_noAdditionalViews = additionalViews; // set this value but don't load anything
                    }
                } else {
                    // additional views not set
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

            if (xmlReader->name() == "ObjectTrack")
            {
                bool errorReading = false;
                DynamicObject::ObjectTrack track = readObjectTrackFromXml(xmlReader, "ObjectTrack",objectFolder.toStdString(), errorReading);
                if (errorReading)
                {
                    std::cerr<<"Error while loading object track. Not adding to object."<<std::endl;
                } else {
                    object->addObjectTrack(track.pose, track.cloud);
                }
            }
        }
    }

    if (m_verbose)
    {
        cout<<"Loaded object from: "<<filename<<endl;
    }

    return object;
}


void DynamicObjectXMLParser::saveObjectTrackToXml(tf::Transform pose, CloudPtr cloud, QXmlStreamWriter* xmlWriter, std::string nodeName, std::string cloudFilename)
{
    geometry_msgs::Transform msg_reg;
    tf::transformTFToMsg(pose, msg_reg);

    xmlWriter->writeStartElement(nodeName.c_str());
    xmlWriter->writeAttribute("Trans_x",QString::number(msg_reg.translation.x));
    xmlWriter->writeAttribute("Trans_y",QString::number(msg_reg.translation.y));
    xmlWriter->writeAttribute("Trans_z",QString::number(msg_reg.translation.z));
    xmlWriter->writeAttribute("Rot_w",QString::number(msg_reg.rotation.w));
    xmlWriter->writeAttribute("Rot_x",QString::number(msg_reg.rotation.x));
    xmlWriter->writeAttribute("Rot_y",QString::number(msg_reg.rotation.y));
    xmlWriter->writeAttribute("Rot_z",QString::number(msg_reg.rotation.z));
    xmlWriter->writeAttribute("Cloud_filename", cloudFilename.c_str());
    pcl::io::savePCDFileBinary(cloudFilename, *cloud);

    xmlWriter->writeEndElement();
}

DynamicObject::ObjectTrack DynamicObjectXMLParser::readObjectTrackFromXml(QXmlStreamReader* xmlReader, std::string nodeName, std::string objectFolder, bool& errorReading)
{
    DynamicObject::ObjectTrack toRet;

    errorReading = false;
    geometry_msgs::Transform tfmsg;
    tf::Transform transform;
    std::string cloud_name = "";

    if (xmlReader->name() == nodeName.c_str())
    {
        errorReading = false;
        QXmlStreamAttributes paramAttributes = xmlReader->attributes();

        if (paramAttributes.hasAttribute("Cloud_filename"))
        {
            QString val = paramAttributes.value("Cloud_filename").toString();
            cloud_name = val.toStdString();
            std::string cloud_path = objectFolder + "/" + cloud_name;
            pcl::PCDReader reader;
            CloudPtr cloud (new Cloud);
            reader.read (cloud_path, *cloud);
            if (cloud->points.size() != 0)
            {
                errorReading = true;
                ROS_ERROR("%s dynamic object track point cloud has no points. ", cloud_path.c_str());
            } else {
                toRet.cloud = cloud;
            }
        } else {
            ROS_ERROR("%s xml node does not have the Stamp_sec attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Trans_x"))
        {
            QString val = paramAttributes.value("Trans_x").toString();
            tfmsg.translation.x = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Trans_x attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Trans_y"))
        {
            QString val = paramAttributes.value("Trans_y").toString();
            tfmsg.translation.y = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Trans_y attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Trans_z"))
        {
            QString val = paramAttributes.value("Trans_z").toString();
            tfmsg.translation.z = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Trans_z attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Rot_w"))
        {
            QString val = paramAttributes.value("Rot_w").toString();
            tfmsg.rotation.w = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Rot_w attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Rot_x"))
        {
            QString val = paramAttributes.value("Rot_x").toString();
            tfmsg.rotation.x = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Rot_x attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Rot_y"))
        {
            QString val = paramAttributes.value("Rot_y").toString();
            tfmsg.rotation.y = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Rot_y attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Rot_z"))
        {
            QString val = paramAttributes.value("Rot_z").toString();
            tfmsg.rotation.z = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Rot_z attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }

    } else {
        ROS_ERROR("Error while attempting to construct tf::StampedTransform from node name %s  Node expected %s",xmlReader->name().toString().toStdString().c_str(), nodeName.c_str());
        errorReading = true;
    }

    if (!errorReading)
    {
        //            ROS_INFO_STREAM("No error while parsing node "<<xmlReader->name().toString().toStdString()<<"  constructing tf object ");
        tf::transformMsgToTF(tfmsg, transform);
        toRet.pose = transform;
    }

    return toRet;
}
