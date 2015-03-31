#include "semantic_map/metaroom.h"
#include "semantic_map/metaroom_xml_parser.h"

template <class PointType>
MetaRoomXMLParser<PointType>::MetaRoomXMLParser(std::string rootFolder)
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

template <class PointType>
MetaRoomXMLParser<PointType>::~MetaRoomXMLParser()
{

}

template <class PointType>
std::string MetaRoomXMLParser<PointType>::saveMetaRoomAsXML(MetaRoom<PointType>& aMetaRoom, std::string xmlFile)
{
    // metarooms are saved in the "home_folder_of_the_semantic_map"/metarooms/metaroom_#
    // they are uniquely identified by their centroids

    // first check that the metaroom contains some data
    if (!aMetaRoom.getCompleteRoomCloudLoaded() && aMetaRoom.getCompleteRoomCloudFilename() == "")
    {
        ROS_INFO_STREAM("Cannot save metaroom as it doesn't contain any data (no pointcloud loaded and the pointcloud filename hasn't been set).");
        return "";
    }

    QString metaRoomLocation = findMetaRoomLocation(&aMetaRoom);
    ROS_INFO_STREAM("Metaroom will be saved in folder "<<metaRoomLocation.toStdString());

    // save metaroom
    QString metaRoomXMLFile = metaRoomLocation+QString(xmlFile.c_str());
    QFile file(metaRoomXMLFile);
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
    xmlWriter->writeStartElement("MetaRoom");

    // RoomCompleteCloud
    xmlWriter->writeStartElement("RoomCompleteCloud");
    QString completeCloudFilenameLocal("complete_cloud.pcd");
    QString completeCloudFilename = metaRoomLocation + completeCloudFilenameLocal; // add the folder prefix
    xmlWriter->writeAttribute("filename",completeCloudFilenameLocal);
    xmlWriter->writeEndElement();
    if (aMetaRoom.getCompleteRoomCloudLoaded()) // only save the cloud file if it's been loaded
    {
        pcl::io::savePCDFileBinary(completeCloudFilename.toStdString(), *aMetaRoom.getCompleteRoomCloud());
        ROS_INFO_STREAM("Saving complete cloud file name "<<completeCloudFilename.toStdString());
    }

    // RoomInteriorCloud
    xmlWriter->writeStartElement("RoomInteriorCloud");
    QString interiorCloudFilenameLocal("interior_cloud.pcd");
    QString interiorCloudFilename = metaRoomLocation + interiorCloudFilenameLocal; // add the folder prefix
    xmlWriter->writeAttribute("filename",interiorCloudFilenameLocal);
    xmlWriter->writeEndElement();
    if (aMetaRoom.getInteriorRoomCloudLoaded()) // only save the cloud file if it's been loaded
    {
        pcl::io::savePCDFileBinary(interiorCloudFilename.toStdString(), *aMetaRoom.getInteriorRoomCloud());
        ROS_INFO_STREAM("Saving interior cloud file name "<<interiorCloudFilename.toStdString());
    }

    // RoomDeNoisedCloud
    xmlWriter->writeStartElement("RoomDeNoisedCloud");
    QString denoisedCloudFilenameLocal("denoised_cloud.pcd");
    QString denoisedCloudFilename = metaRoomLocation + denoisedCloudFilenameLocal; // add the folder prefix
    xmlWriter->writeAttribute("filename",denoisedCloudFilenameLocal);
    xmlWriter->writeEndElement();
    if (aMetaRoom.getDeNoisedRoomCloudLoaded()) // only save the cloud file if it's been loaded
    {
        pcl::io::savePCDFileBinary(denoisedCloudFilename.toStdString(), *aMetaRoom.getDeNoisedRoomCloud());
        ROS_INFO_STREAM("Saving denoised cloud file name "<<denoisedCloudFilename.toStdString());
    }

    // ConsistencyUpdateCloud
    xmlWriter->writeStartElement("ConsistencyUpdateCloud");
    QString consistencyUpdateCloudFilenameLocal("consistency_update_cloud.pcd");
    QString consistencyUpdateCloudFilename = metaRoomLocation + consistencyUpdateCloudFilenameLocal; // add the folder prefix
    xmlWriter->writeAttribute("filename",consistencyUpdateCloudFilenameLocal);
    xmlWriter->writeEndElement();
    if (aMetaRoom.getConsistencyUpdateCloudLoaded()) // only save the cloud file if it's been loaded
    {
        pcl::io::savePCDFileBinary(consistencyUpdateCloudFilename.toStdString(), *aMetaRoom.getConsistencyUpdateCloud());
        ROS_INFO_STREAM("Saving consistency update cloud file name "<<consistencyUpdateCloudFilename.toStdString());
    }

    // MetaroomStringId
    xmlWriter->writeStartElement("MetaRoomStringId");
    xmlWriter->writeCharacters(aMetaRoom.m_sMetaroomStringId.c_str());
    xmlWriter->writeEndElement();

    // RoomCentroid
    xmlWriter->writeStartElement("Centroid");
    Eigen::Vector4f centroid = aMetaRoom.getCentroid();
    QString centroidS = QString::number(centroid(0))+" "+QString::number(centroid(1))+" "+QString::number(centroid(2))+" "+QString::number(centroid(3));
    xmlWriter->writeCharacters(centroidS);
    xmlWriter->writeEndElement();

    // RoomSensorOrigin
    xmlWriter->writeStartElement("SensorOrigin");
    tf::Vector3 sensorOrigin = aMetaRoom.getSensorOrigin();
    QString soS = QString::number(sensorOrigin.x())+" "+QString::number(sensorOrigin.y())+" "+QString::number(sensorOrigin.z());
    xmlWriter->writeCharacters(soS);
    xmlWriter->writeEndElement();

    // Room Transform
    xmlWriter->writeStartElement("RoomTransform");
    Eigen::Matrix4f transform = aMetaRoom.getRoomTransform();
    QString transformS;
    for (size_t i=0; i<16;i++)
    {
        transformS+=QString::number(transform(i))+" ";
    }
    xmlWriter->writeCharacters(transformS);
    xmlWriter->writeEndElement();

    // update iterations
    xmlWriter->writeStartElement("UpdateIterations");
    std::vector<MetaRoomUpdateIteration<PointType>> updateIterations = aMetaRoom.getUpdateIterations();
    for (size_t i=0; i<updateIterations.size();i++)
    {
        xmlWriter->writeStartElement("UpdateIteration");

        xmlWriter->writeStartElement("RoomLogName");
        xmlWriter->writeCharacters(QString(updateIterations[i].roomLogName.c_str()));
        xmlWriter->writeEndElement();

        xmlWriter->writeStartElement("RoomRunNumber");
        xmlWriter->writeCharacters(QString::number(updateIterations[i].roomRunNumber));
        xmlWriter->writeEndElement();

        xmlWriter->writeStartElement("DifferenceMetaRoomToRoom");
        {
            std::stringstream ss;
            ss << "differenceMetaRoomToRoomIteration";
            ss <<i;
            ss <<".pcd";
            QString completeFileNameLocal = QString(ss.str().c_str());
            QString completeFileName = metaRoomLocation + completeFileNameLocal;
            if (updateIterations[i].differenceMetaRoomToRoomLoaded) // only save if it's been loaded
            {
                ROS_INFO_STREAM("Saving difference metaroom to room "<<completeFileName.toStdString());
                pcl::io::savePCDFile (completeFileName.toStdString(), *updateIterations[i].getDifferenceMetaRoomToRoom(), true);
                xmlWriter->writeCharacters(completeFileNameLocal);
            }

        }
        xmlWriter->writeEndElement();


        xmlWriter->writeStartElement("DifferenceRoomToMetaRoom");
        {
            std::stringstream ss;
            ss << "differenceRoomToMetaRoomIteration";
            ss <<i;
            ss <<".pcd";
            QString completeFileNameLocal = QString(ss.str().c_str());
            QString completeFileName = metaRoomLocation + completeFileNameLocal;
            if (updateIterations[i].differenceRoomToMetaRoomLoaded) // only save if it's been loaded
            {
                std::cout<<"Saving difference room to metaroom"<<completeFileName.toStdString()<<std::endl;
                pcl::io::savePCDFile (completeFileName.toStdString(), *updateIterations[i].getDifferenceRoomToMetaRoom(), true);
                xmlWriter->writeCharacters(completeFileNameLocal);
            }
        }
        xmlWriter->writeEndElement();

        if (updateIterations[i].clustersToBeAddedLoaded && updateIterations[i].getClustersToBeAdded()->points.size() > 0)
        {
            xmlWriter->writeStartElement("ClustersToBeAdded");
            {
                std::stringstream ss;
                ss << "clustersToBeAdded";
                ss <<i;
                ss <<".pcd";
                QString completeFileNameLocal = QString(ss.str().c_str());
                QString completeFileName = metaRoomLocation + completeFileNameLocal;

                if (updateIterations[i].clustersToBeAddedLoaded) // only save if it's been loaded
                {
                    std::cout<<"Saving clustersToBeAdded "<<completeFileName.toStdString()<<std::endl;
                    pcl::io::savePCDFile (completeFileName.toStdString(), *updateIterations[i].getClustersToBeAdded(), true);
                    xmlWriter->writeCharacters(completeFileNameLocal);
                }
            }
            xmlWriter->writeEndElement();
        }

        if (updateIterations[i].clustersToBeRemovedLoaded && updateIterations[i].getClustersToBeRemoved()->points.size() > 0)
        {
            xmlWriter->writeStartElement("ClustersToBeRemoved");
            {
                std::stringstream ss;
                ss << "clustersToBeRemoved";
                ss <<i;
                ss <<".pcd";
                QString completeFileNameLocal = QString(ss.str().c_str());
                QString completeFileName = metaRoomLocation + completeFileNameLocal;
                if (updateIterations[i].clustersToBeRemovedLoaded) // only save if it's been loaded
                {
                    std::cout<<"Saving clustersToBeAdded "<<completeFileName.toStdString()<<std::endl;
                    pcl::io::savePCDFile (completeFileName.toStdString(), *updateIterations[i].getClustersToBeRemoved(), true);
                    xmlWriter->writeCharacters(completeFileNameLocal);
                }
            }
            xmlWriter->writeEndElement();
        }

        xmlWriter->writeStartElement("MetaRoomInteriorCloud");
        {
            std::stringstream ss;
            ss << "metaRoomInteriorCloud";
            ss <<i;
            ss <<".pcd";
            QString completeFileNameLocal = QString(ss.str().c_str());
            QString completeFileName = metaRoomLocation + completeFileNameLocal;
            if (updateIterations[i].metaRoomInteriorCloudLoaded) // only save if it's been loaded
            {
                std::cout<<"Saving metaRoomInteriorCloud "<<completeFileName.toStdString()<<std::endl;
                pcl::io::savePCDFile (completeFileName.toStdString(), *updateIterations[i].getMetaRoomInteriorCloud(), true);
                xmlWriter->writeCharacters(completeFileNameLocal);
            }
        }
        xmlWriter->writeEndElement();

        xmlWriter->writeEndElement(); //UpdateIteration
    }

    xmlWriter->writeEndElement(); // UpdateIterations

    // Bounding primitives
    xmlWriter->writeStartElement("BoundingPrimitives");

    xmlWriter->writeStartElement("CeilingPrimitive");
    std::pair<pcl::ModelCoefficients::Ptr,bool> ceilingPrimitive = aMetaRoom.getCeilingPrimitive();
    QString ceilingS = QString::number(ceilingPrimitive.first->values[0])+" "+QString::number(ceilingPrimitive.first->values[1])+" "+QString::number(ceilingPrimitive.first->values[2])+" "+QString::number(ceilingPrimitive.first->values[3]);
    xmlWriter->writeAttribute("direction", QString::number(ceilingPrimitive.second));
//        ROS_INFO_STREAM("Saving ceiling plane with direction "<<ceilingPrimitive.second);
    xmlWriter->writeCharacters(ceilingS);
    xmlWriter->writeEndElement(); // Ceiling Plane

    xmlWriter->writeStartElement("FloorPrimitive");
    std::pair<pcl::ModelCoefficients::Ptr,bool> floorPrimitive = aMetaRoom.getFloorPrimitive();
    QString floorS = QString::number(floorPrimitive.first->values[0])+" "+QString::number(floorPrimitive.first->values[1])+" "+QString::number(floorPrimitive.first->values[2])+" "+QString::number(floorPrimitive.first->values[3]);
    xmlWriter->writeAttribute("direction", QString::number(floorPrimitive.second));
//        ROS_INFO_STREAM("Saving floor plane with direction "<<floorPrimitive.second);
    xmlWriter->writeCharacters(floorS);
    xmlWriter->writeEndElement(); // Ceiling Plane


    // wall primitives
    std::pair<std::vector<pcl::ModelCoefficients::Ptr>,std::vector<bool> > wallPrimitives = aMetaRoom.getWallPrimitives();
    for (size_t i=0; i<wallPrimitives.first.size();i++)
    {
        xmlWriter->writeStartElement("WallPrimitive");
        QString wallS = QString::number(wallPrimitives.first[i]->values[0])+" "+QString::number(wallPrimitives.first[i]->values[1])+" "+QString::number(wallPrimitives.first[i]->values[2])+" "+QString::number(wallPrimitives.first[i]->values[3]);
        xmlWriter->writeAttribute("direction", QString::number(wallPrimitives.second[i]));
        xmlWriter->writeCharacters(wallS);
//            ROS_INFO_STREAM("Saving wall plane with direction "<<wallPrimitives.second[i]);
        xmlWriter->writeEndElement();
    }



    xmlWriter->writeEndElement(); // Update Iterations
    xmlWriter->writeEndElement(); // Meta Room
    xmlWriter->writeEndDocument();

    delete xmlWriter;

    return metaRoomXMLFile.toStdString();

}

template <class PointType>
MetaRoom<PointType> MetaRoomXMLParser<PointType>::loadMetaRoomFromXML(const std::string& xmlFile, bool deepLoad)
{
    MetaRoom<PointType> aMetaRoom;

    QFile file(xmlFile.c_str());

    if (!file.exists())
    {
        std::cerr<<"Could not open file "<<xmlFile<<" to load metaroom."<<std::endl;
        return aMetaRoom;
    }

    QString xmlFileQS(xmlFile.c_str());
    int index = xmlFileQS.lastIndexOf('/');
    QString metaroomFolder = xmlFileQS.left(index);

    file.open(QIODevice::ReadOnly);

    QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);
    Eigen::Vector4f centroid(0.0,0.0,0.0,0.0);

    std::vector<pcl::ModelCoefficients::Ptr> wallPv;
    std::vector<bool> wallDirections;

    while (!xmlReader->atEnd() && !xmlReader->hasError())
    {
        QXmlStreamReader::TokenType token = xmlReader->readNext();
        if (token == QXmlStreamReader::StartDocument)
            continue;

        if (xmlReader->hasError())
        {
            std::cout << "XML error: " << xmlReader->errorString().toStdString() << std::endl;
            return aMetaRoom;
        }

        QString elementName = xmlReader->name().toString();

        if (token == QXmlStreamReader::StartElement)
        {
            if (xmlReader->name() == "RoomCompleteCloud")
            {
                QXmlStreamAttributes attributes = xmlReader->attributes();
                if (attributes.hasAttribute("filename"))
                {
                    QString roomCompleteCloudFile = attributes.value("filename").toString();
                    int sl_index = roomCompleteCloudFile.indexOf('/');
                    if (sl_index == -1)
                    {
                        roomCompleteCloudFile=metaroomFolder + "/" + roomCompleteCloudFile;
                    }
                    std::ifstream file(roomCompleteCloudFile.toStdString().c_str());

                    if (deepLoad && file)
                    {
                        std::cout<<"Loading complete cloud file name "<<roomCompleteCloudFile.toStdString()<<std::endl;
                        pcl::PCDReader reader;
                        CloudPtr cloud (new Cloud);
                        reader.read (roomCompleteCloudFile.toStdString(), *cloud);
                        aMetaRoom.setCompleteRoomCloud(cloud);
                    } else {
                        aMetaRoom.setCompleteRoomCloud(roomCompleteCloudFile.toStdString());
                    }
                } else {
                    std::cerr<<"RoomCompleteCloud xml node does not have filename attribute. Aborting."<<std::endl;
                    return aMetaRoom;
                }
            }

            if (xmlReader->name() == "RoomInteriorCloud")
            {
                QXmlStreamAttributes attributes = xmlReader->attributes();
                if (attributes.hasAttribute("filename"))
                {
                    QString roomInteriorCloudFile = attributes.value("filename").toString();
                    int sl_index = roomInteriorCloudFile.indexOf('/');
                    if (sl_index == -1)
                    {
                        roomInteriorCloudFile=metaroomFolder + "/" + roomInteriorCloudFile;
                    }
                    std::ifstream file(roomInteriorCloudFile.toStdString().c_str());

                    if (deepLoad && file)
                    {
                        std::cout<<"Loading interior cloud file name "<<roomInteriorCloudFile.toStdString()<<std::endl;
                        pcl::PCDReader reader;
                        CloudPtr cloud (new Cloud);
                        reader.read (roomInteriorCloudFile.toStdString(), *cloud);
                        aMetaRoom.setInteriorRoomCloud(cloud);
                    } else {
                        aMetaRoom.setInteriorRoomCloud(roomInteriorCloudFile.toStdString());
                    }
                } else {
                    std::cerr<<"RoomInteriorCloud xml node does not have filename attribute. Aborting."<<std::endl;
                    return aMetaRoom;
                }
            }

            if (xmlReader->name() == "RoomDeNoisedCloud")
            {
                QXmlStreamAttributes attributes = xmlReader->attributes();
                if (attributes.hasAttribute("filename"))
                {
                    QString roomDenoisedCloudFile = attributes.value("filename").toString();
                    int sl_index = roomDenoisedCloudFile.indexOf('/');
                    if (sl_index == -1)
                    {
                        roomDenoisedCloudFile=metaroomFolder + "/" + roomDenoisedCloudFile;
                    }
                    std::ifstream file(roomDenoisedCloudFile.toStdString().c_str());

                    if (deepLoad && file)
                    {
                        std::cout<<"Loading denoised cloud file name "<<roomDenoisedCloudFile.toStdString()<<std::endl;
                        pcl::PCDReader reader;
                        CloudPtr cloud (new Cloud);
                        reader.read (roomDenoisedCloudFile.toStdString(), *cloud);
                        aMetaRoom.setDeNoisedRoomCloud(cloud);
                    } else {
                        aMetaRoom.setDeNoisedRoomCloud(roomDenoisedCloudFile.toStdString());
                    }
                } else {
                    std::cerr<<"RoomDeNoisedCloud xml node does not have filename attribute. Aborting."<<std::endl;
                    return aMetaRoom;
                }
            }

            if (xmlReader->name() == "ConsistencyUpdateCloud")
            {
                QXmlStreamAttributes attributes = xmlReader->attributes();
                if (attributes.hasAttribute("filename"))
                {
                    QString consistencyUpdateCloudFile = attributes.value("filename").toString();
                    int sl_index = consistencyUpdateCloudFile.indexOf('/');
                    if (sl_index == -1)
                    {
                        consistencyUpdateCloudFile=metaroomFolder + "/" + consistencyUpdateCloudFile;
                    }
                    std::ifstream file(consistencyUpdateCloudFile.toStdString().c_str());

                    if (deepLoad && file)
                    {
                        std::cout<<"Loading consistency update cloud file name "<<consistencyUpdateCloudFile.toStdString()<<std::endl;
                        pcl::PCDReader reader;
                        CloudPtr cloud (new Cloud);
                        reader.read (consistencyUpdateCloudFile.toStdString(), *cloud);
                        aMetaRoom.setConsistencyUpdateCloud(cloud);
                    } else {
                        aMetaRoom.setConsistencyUpdateCloud(consistencyUpdateCloudFile.toStdString());
                    }
                } else {
                    std::cerr<<"ConsistencyUpdateCloud xml node does not have filename attribute. Aborting."<<std::endl;
                    return aMetaRoom;
                }
            }

            if (xmlReader->name() == "CeilingPrimitive")
            {
                QXmlStreamAttributes attributes = xmlReader->attributes();
                QString ceilingS = xmlReader->readElementText();
                pcl::ModelCoefficients::Ptr ceilingP  = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
                QStringList ceilingSlist = ceilingS.split(' ');
                ceilingP->values.resize(4);
                ceilingP->values[0] = ceilingSlist[0].toDouble();ceilingP->values[1] = ceilingSlist[1].toDouble();
                ceilingP->values[2] = ceilingSlist[2].toDouble();ceilingP->values[3] = ceilingSlist[3].toDouble();
                bool direction;
                if (attributes.value("direction").toString() == "0")
                {
                    direction = false;
                } else {
                    direction = true;
                }
                aMetaRoom.setCeilingPrimitive(ceilingP,direction);
            }

            if (xmlReader->name() == "FloorPrimitive")
            {
                QXmlStreamAttributes attributes = xmlReader->attributes();
                QString floorS = xmlReader->readElementText();
                pcl::ModelCoefficients::Ptr floorP  = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
                floorP->values.resize(4);
                QStringList floorSlist = floorS.split(' ');
                floorP->values[0] = floorSlist[0].toDouble();floorP->values[1] = floorSlist[1].toDouble();
                floorP->values[2] = floorSlist[2].toDouble();floorP->values[3] = floorSlist[3].toDouble();
                bool direction;
                if (attributes.value("direction").toString()== "0")
                {
                    direction = false;
                } else {
                    direction = true;
                }
                aMetaRoom.setFloorPrimitive(floorP,direction);
            }

            if (xmlReader->name() == "WallPrimitive")
            {
                QXmlStreamAttributes attributes = xmlReader->attributes();
                QString wallS = xmlReader->readElementText();
                pcl::ModelCoefficients::Ptr wallP = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
                wallP->values.resize(4);
                QStringList wallSlist = wallS.split(' ');
                wallP->values[0] = wallSlist[0].toDouble();wallP->values[1] = wallSlist[1].toDouble();
                wallP->values[2] = wallSlist[2].toDouble();wallP->values[3] = wallSlist[3].toDouble();
                bool direction;
                if (attributes.value("direction").toString() == "0")
                {
                    direction = false;
                } else {
                    direction = true;
                }
                wallPv.push_back(wallP);
                wallDirections.push_back(direction);
            }

            if (xmlReader->name() == "Centroid")
            {
                QString centroidS = xmlReader->readElementText();
                Eigen::Vector4f centroid;
                QStringList centroidSlist = centroidS.split(' ');
                centroid(0) = centroidSlist[0].toDouble();centroid(1) = centroidSlist[1].toDouble();
                centroid(2) = centroidSlist[2].toDouble();centroid(3) = centroidSlist[3].toDouble();
                aMetaRoom.setCentroid(centroid);
            }

            if (xmlReader->name() == "MetaRoomStringId")
            {
                QString metaroomStringId = xmlReader->readElementText();
                aMetaRoom.m_sMetaroomStringId = (metaroomStringId.toStdString());
            }

            if (xmlReader->name() == "SensorOrigin")
            {
                QString soS = xmlReader->readElementText();
                tf::Vector3 so;
                QStringList soSlist = soS.split(' ');
                so.setX(soSlist[0].toDouble()); so.setY(soSlist[1].toDouble());
                so.setZ(soSlist[2].toDouble());
                aMetaRoom.setSensorOrigin(so);
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
                aMetaRoom.setRoomTransform(transform);
            }

            // Update iterations
            if (xmlReader->name() == "UpdateIteration")
            {
                MetaRoomUpdateIteration<PointType> updateIteration;
                bool completelyUpdated = true;
                while(!((token == QXmlStreamReader::EndElement) && (xmlReader->name() == "UpdateIteration")) )
                {
                    if (xmlReader->name() == "RoomLogName")
                    {
                        QString roomLogName = xmlReader->readElementText();
                        updateIteration.roomLogName = roomLogName.toStdString();
                    }

                    if (xmlReader->name() == "RoomRunNumber")
                    {
                         int roomRunNumber = xmlReader->readElementText().toInt();
                         updateIteration.roomRunNumber = roomRunNumber;
                    }

                    if (xmlReader->name() == "DifferenceMetaRoomToRoom")
                    {
                        QXmlStreamAttributes update_attributes = xmlReader->attributes();
                        QString differenceMetaRoomToRoomFile = xmlReader->readElementText();
                        int sl_index = differenceMetaRoomToRoomFile.indexOf('/');
                        if (sl_index == -1)
                        {
                            differenceMetaRoomToRoomFile=metaroomFolder + "/" + differenceMetaRoomToRoomFile;
                        }
                        std::ifstream file(differenceMetaRoomToRoomFile.toStdString().c_str());

                        if (deepLoad && file)
                        {
                            std::cout<<"Loading DifferenceMetaRoomToRoom cloud file name "<<differenceMetaRoomToRoomFile.toStdString()<<std::endl;
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (differenceMetaRoomToRoomFile.toStdString(), *cloud);
                            updateIteration.setDifferenceMetaRoomToRoom(cloud);
                        } else {
                            updateIteration.setDifferenceMetaRoomToRoom(differenceMetaRoomToRoomFile.toStdString());
                        }
                    }

                    if (xmlReader->name() == "DifferenceRoomToMetaRoom")
                    {
                        QString differenceRoomToMetaRoomFile = xmlReader->readElementText();
                        int sl_index = differenceRoomToMetaRoomFile.indexOf('/');
                        if (sl_index == -1)
                        {
                            differenceRoomToMetaRoomFile=metaroomFolder + "/" + differenceRoomToMetaRoomFile;
                        }
                        std::ifstream file(differenceRoomToMetaRoomFile.toStdString().c_str());

                        if (deepLoad && file)
                        {
                            std::cout<<"Loading DifferenceRoomToMetaRoom cloud file name "<<differenceRoomToMetaRoomFile.toStdString()<<std::endl;
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (differenceRoomToMetaRoomFile.toStdString(), *cloud);
                            updateIteration.setDifferenceRoomToMetaRoom(cloud);
                        } else {
                            updateIteration.setDifferenceRoomToMetaRoom(differenceRoomToMetaRoomFile.toStdString());
                        }
                    }

                    if (xmlReader->name() == "ClustersToBeAdded")
                    {
                        QString clustersToBeAdded = xmlReader->readElementText();
                        int sl_index = clustersToBeAdded.indexOf('/');
                        if (sl_index == -1)
                        {
                            clustersToBeAdded=metaroomFolder + "/" + clustersToBeAdded;
                        }
                        std::ifstream file(clustersToBeAdded.toStdString().c_str());

                        if (deepLoad && file)
                        {
                            std::cout<<"Loading ClustersToBeAdded cloud file name "<<clustersToBeAdded.toStdString()<<std::endl;
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (clustersToBeAdded.toStdString(), *cloud);
                            updateIteration.setClustersToBeAdded(cloud);
                        } else {
                            updateIteration.setClustersToBeAdded(clustersToBeAdded.toStdString());
                        }
                    }

                    if (xmlReader->name() == "ClustersToBeRemoved")
                    {
                        QString clustersToBeRemoved = xmlReader->readElementText();
                        int sl_index = clustersToBeRemoved.indexOf('/');
                        if (sl_index == -1)
                        {
                            clustersToBeRemoved=metaroomFolder + "/" + clustersToBeRemoved;
                        }
                        std::ifstream file(clustersToBeRemoved.toStdString().c_str());

                        if (deepLoad && file)
                        {
                            std::cout<<"Loading ClustersToBeRemoved cloud file name "<<clustersToBeRemoved.toStdString()<<std::endl;
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (clustersToBeRemoved.toStdString(), *cloud);
                            updateIteration.setClustersToBeRemoved(cloud);
                        } else {
                            updateIteration.setClustersToBeRemoved(clustersToBeRemoved.toStdString());
                        }
                    }

                    if (xmlReader->name() == "MetaRoomInteriorCloud")
                    {
                        QString metaRoomInteriorCloud = xmlReader->readElementText();
                        int sl_index = metaRoomInteriorCloud.indexOf('/');
                        if (sl_index == -1)
                        {
                            metaRoomInteriorCloud=metaroomFolder + "/" + metaRoomInteriorCloud;
                        }
                        std::ifstream file(metaRoomInteriorCloud.toStdString().c_str());

                        if (deepLoad && file)
                        {
                            std::cout<<"Loading metaRoomInteriorCloud cloud file name "<<metaRoomInteriorCloud.toStdString()<<std::endl;
                            pcl::PCDReader reader;
                            CloudPtr cloud (new Cloud);
                            reader.read (metaRoomInteriorCloud.toStdString(), *cloud);
                            updateIteration.setMetaRoomInteriorCloud(cloud);
                        } else {
                            updateIteration.setMetaRoomInteriorCloud(metaRoomInteriorCloud.toStdString());
                        }
                    }

                    token = xmlReader->readNext();
                }

                if (completelyUpdated)
                {
                    aMetaRoom.addUpdateIteration(updateIteration);
                }
            }

        }
    }

    aMetaRoom.setWallPrimitives(wallPv, wallDirections);

    delete xmlReader;


    return aMetaRoom;

}

template <class PointType>
QString MetaRoomXMLParser<PointType>::findMetaRoomLocation(MetaRoom<PointType>* aMetaRoom)
{
    // list the folders in the home folder of the semantic map and look for the metarooms folder
    QStringList rootFolders = QDir(m_RootFolder).entryList(QStringList("*"),
                                                    QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);

    bool metaRoomsFolderFound = false;

    for (size_t i=0; i<rootFolders.size(); i++)
    {
        if (rootFolders[i] == "metarooms")
        {
            metaRoomsFolderFound = true;
        }
    }

    QString metaroomsRootFolder = m_RootFolder+"metarooms" + "/";
    if (!metaRoomsFolderFound)
    {
        // first time a metaroom is created
        bool folderCreated = QDir().mkdir(metaroomsRootFolder);
//            ROS_INFO_STREAM("Creating the metaroom root folder returned "<<folderCreated<<" folder name "<<metaroomsRootFolder.toStdString());
    }

    QStringList metaroomFolders = QDir(metaroomsRootFolder).entryList(QStringList("*"),
                                                    QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);

    QString metaRoomFolder;
    if (metaroomFolders.size() == 0)
    {
        // no metarooms so far -> create a folder and return it
        metaRoomFolder = metaroomsRootFolder + "metaroom_0" + "/";
        bool folderCreated = QDir().mkdir(metaRoomFolder);
//            ROS_INFO_STREAM("Creating the metaroom root folder returned "<<folderCreated<<" folder name "<<metaRoomFolder.toStdString());
    } else {
        // there are some metarooms -> compare centroids and see which one we are trying to update
        // alternatively, if one of the metaroom folders doesn't contain an .xml file, we take that one
        bool metaRoomFolderFound = false;
        int metarooms = 0;
        for (size_t i=0;i<metaroomFolders.size(); i++)
        {
            // first check that this folder has the right format
            if (metaroomFolders[i].indexOf("metaroom") == -1)
            {
                // not the right format -> skip it
//                    ROS_INFO_STREAM("Skipping folder "<<metaroomFolders[i].toStdString()<<". It doesn't have the right format.");
                continue;
            }
            QString tempMetaRoomFolder = metaroomsRootFolder+metaroomFolders[i]+"/";
            QStringList tempMetaRoomFolderFiles = QDir(tempMetaRoomFolder).entryList(QStringList("*.xml"));
            if (tempMetaRoomFolderFiles.size() == 0)
            {
                // no .xml files in this folder
                metaRoomFolder = tempMetaRoomFolder;
                metaRoomFolderFound = true;
                ROS_INFO_STREAM("No xml files in metaroom folder "<<tempMetaRoomFolder.toStdString()<<". It will be used for saving the current metaroom.");
            } else {
                QString savedMetaRoomXMLFile = tempMetaRoomFolder+tempMetaRoomFolderFiles[0]; // TODO for now I am assuming there is only 1 xml file in the metaroom folder. Maybe later there will be more.
                MetaRoom<PointType> savedMetaRoom = MetaRoomXMLParser::loadMetaRoomFromXML(savedMetaRoomXMLFile.toStdString(),false);
                double centroidDistance = pcl::distances::l2(savedMetaRoom.getCentroid(),aMetaRoom->getCentroid());
                ROS_INFO_STREAM("Comparing with saved metaroom. Centroid distance "<<centroidDistance);
                if (centroidDistance < ROOM_CENTROID_DISTANCE)
                {
                    // found a matching metaroom
                    metaRoomFolder = tempMetaRoomFolder;
                    metaRoomFolderFound = true;
                    break;
                }

                metarooms++;
            }
        }

        if (!metaRoomFolderFound)
        {
            metaRoomFolder = metaroomsRootFolder + "metaroom_"+QString::number(metarooms) + "/";
            bool folderCreated = QDir().mkdir(metaRoomFolder);
//                ROS_INFO_STREAM("Creating the metaroom root folder returned "<<folderCreated<<" folder name "<<metaRoomFolder.toStdString());


        }
    }


    return metaRoomFolder;
}
