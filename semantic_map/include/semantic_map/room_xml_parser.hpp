#include "semantic_map/room_xml_parser.h"

template <class PointType>
SemanticRoomXMLParser<PointType>::SemanticRoomXMLParser(std::string rootFolder)
{
    if (rootFolder == "home")
    {
        m_RootFolder =(QDir::homePath() + QString("/.semanticMap/"));
    } else {
        m_RootFolder = rootFolder.c_str();
    }

    // create root folder
    if (!QDir(m_RootFolder).exists())
    {
        bool folderCreated = QDir().mkdir(m_RootFolder);
        ROS_INFO_STREAM("Creating the root folder returned "<<folderCreated<<" folder name "<<m_RootFolder.toStdString());
    }

}

template <class PointType>
SemanticRoomXMLParser<PointType>::~SemanticRoomXMLParser()
{

}

/// Assumes that the roomXml file follows the structure /path_to_root_folder/date_folder/patrol_run_#/room_#/room.xml
template <class PointType>
bool SemanticRoomXMLParser<PointType>::setRootFolderFromRoomXml(std::string roomXml)
{
    QString qRoomXml(roomXml.c_str());
    int slash_index = qRoomXml.lastIndexOf('/');
    qRoomXml = qRoomXml.left(slash_index); // room folder
    slash_index = qRoomXml.lastIndexOf('/');
    qRoomXml = qRoomXml.left(slash_index); // patrol folder
    slash_index = qRoomXml.lastIndexOf('/');
    qRoomXml = qRoomXml.left(slash_index); // date folder
    slash_index = qRoomXml.lastIndexOf('/');
    qRoomXml = qRoomXml.left(slash_index+1); // root folder

    ROS_INFO_STREAM("Setting root folder to "<<qRoomXml.toStdString());
    m_RootFolder = qRoomXml;
    // create root folder
    if (!QDir(m_RootFolder).exists())
    {
        bool folderCreated = QDir().mkdir(m_RootFolder);
        ROS_INFO_STREAM("Creating the root folder returned "<<folderCreated<<" folder name "<<m_RootFolder.toStdString());
    }

}

template <class PointType>
std::string SemanticRoomXMLParser<PointType>::saveRoomAsXML(SemanticRoom<PointType>& aRoom, std::string xmlFile, bool verbose )
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
    ROS_INFO_STREAM("Saving room at "<<xmlFile);

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
    QString cloudFilename("complete_cloud.pcd");
    QString completeCloudFilename = roomFolder + cloudFilename; // add the folder prefix
    xmlWriter->writeAttribute("filename",cloudFilename);
    xmlWriter->writeEndElement();
    if (aRoom.getCompleteRoomCloudLoaded()) // only save the cloud file if it's been loaded
    {
        QFile file(completeCloudFilename);
//        if (!file.exists())
        {
            if (aRoom.getCompleteRoomCloud()->points.size()>0)
            {
                pcl::io::savePCDFileBinary(completeCloudFilename.toStdString(), *aRoom.getCompleteRoomCloud());
                if (verbose)
                {
                    ROS_INFO_STREAM("Saving complete cloud file name "<<completeCloudFilename.toStdString());
                }
            }
        }
    }

    // RoomInteriorCloud
    xmlWriter->writeStartElement("RoomInteriorCloud");
    QString interiorCloudFilenameLocal("interior_cloud.pcd");
    QString interiorCloudFilename = roomFolder + interiorCloudFilenameLocal; // add the folder prefix
    xmlWriter->writeAttribute("filename",interiorCloudFilenameLocal);
    xmlWriter->writeEndElement();
    if (aRoom.getInteriorRoomCloudLoaded()) // only save the cloud file if it's been loaded
    {
        QFile file(interiorCloudFilename);
//        if (!file.exists())
        {
            if (aRoom.getInteriorRoomCloud()->points.size()>0)
            {
                pcl::io::savePCDFileBinary(interiorCloudFilename.toStdString(), *aRoom.getInteriorRoomCloud());
                if (verbose)
                {
                    ROS_INFO_STREAM("Saving interior cloud file name "<<interiorCloudFilename.toStdString());
                }
            }
        }
        aRoom.setInteriorRoomCloud(interiorCloudFilename.toStdString());
    }

    // RoomDeNoisedCloud
    xmlWriter->writeStartElement("RoomDeNoisedCloud");
    QString denoisedCloudFilenameLocal("denoised_cloud.pcd");
    QString denoisedCloudFilename = roomFolder + denoisedCloudFilenameLocal; // add the folder prefix
    xmlWriter->writeAttribute("filename",denoisedCloudFilenameLocal);
    xmlWriter->writeEndElement();
    if (aRoom.getDeNoisedRoomCloudLoaded()) // only save the cloud file if it's been loaded
    {
        QFile file(denoisedCloudFilename);
        if (!file.exists())
        {
            if (aRoom.getDeNoisedRoomCloud()->points.size())
            {
                pcl::io::savePCDFileBinary(denoisedCloudFilename.toStdString(), *aRoom.getDeNoisedRoomCloud());
                if (verbose)
                {
                    ROS_INFO_STREAM("Saving denoised cloud file name "<<denoisedCloudFilename.toStdString());
                }
            }
        }
    }

    // RoomDynamicClusters
    QString dynamicClustersFilenameLocal("dynamic_clusters.pcd");
    QString dynamicClustersFilename = roomFolder + dynamicClustersFilenameLocal; // add the folder prefix
    QFile dynamicClustersFile(dynamicClustersFilename);

    if (aRoom.getDynamicClustersCloudLoaded() && aRoom.getDynamicClustersCloud()->points.size()) // only save the cloud file if it's been loaded
    {
        //            if (!dynamicClustersFile.exists())
        //            {
        pcl::io::savePCDFileBinary(dynamicClustersFilename.toStdString(), *aRoom.getDynamicClustersCloud());

        if (verbose)
        {
            ROS_INFO_STREAM("Saving dynamic clusters cloud file name "<<dynamicClustersFilename.toStdString());
        }
        //            }

        xmlWriter->writeStartElement("RoomDynamicClusters");
        xmlWriter->writeAttribute("filename",dynamicClustersFilenameLocal);
        xmlWriter->writeEndElement();
    } else {
        if (dynamicClustersFile.exists())
        {
            xmlWriter->writeStartElement("RoomDynamicClusters");
            xmlWriter->writeAttribute("filename",dynamicClustersFilenameLocal);
            xmlWriter->writeEndElement();
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
    xmlWriter->writeAttribute("pan_start",QString::number(aRoom.pan_start));
    xmlWriter->writeAttribute("pan_step",QString::number(aRoom.pan_step));
    xmlWriter->writeAttribute("pan_end",QString::number(aRoom.pan_end));
    xmlWriter->writeAttribute("tilt_start",QString::number(aRoom.tilt_start));
    xmlWriter->writeAttribute("tilt_step",QString::number(aRoom.tilt_step));
    xmlWriter->writeAttribute("tilt_end",QString::number(aRoom.tilt_end));

    std::vector<CloudPtr> roomIntermediateClouds = aRoom.getIntermediateClouds();
    std::vector<tf::StampedTransform> roomIntermediateCloudTransforms = aRoom.getIntermediateCloudTransforms();    
    std::vector<image_geometry::PinholeCameraModel> roomIntermediateCloudCameraParameters = aRoom.getIntermediateCloudCameraParameters();
    std::vector<bool>   roomIntermediateCloudsLoaded = aRoom.getIntermediateCloudsLoaded();
    for (size_t i=0; i<roomIntermediateCloudTransforms.size(); i++)
    {
        // RoomIntermediateCloud
        xmlWriter->writeStartElement("RoomIntermediateCloud");
        std::stringstream ss;
        QString intermediateCloudLocalPath = "";
        QString intermediateCloudPath = "";
        if (aRoom.getSaveIntermediateClouds())
        {
            ss << "intermediate_cloud"<<std::setfill('0')<<std::setw(4)<<i<<".pcd";
            intermediateCloudLocalPath = ss.str().c_str();
            intermediateCloudPath = roomFolder + intermediateCloudLocalPath;
        }
        xmlWriter->writeAttribute("filename",intermediateCloudLocalPath);

        if(roomIntermediateCloudsLoaded[i] && aRoom.getSaveIntermediateClouds())
        {
            QFile file(intermediateCloudPath);
            if (!file.exists())
            {
                pcl::io::savePCDFileBinary(intermediateCloudPath.toStdString(), *roomIntermediateClouds[i]);
                if (verbose)
                {
                    ROS_INFO_STREAM("Saving intermediate cloud file name "<<intermediateCloudPath.toStdString());
                }
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

        // RoomIntermediateCloudRegisteredTransform
        std::vector<tf::StampedTransform> roomIntermediateCloudTransformsRegistered = aRoom.getIntermediateCloudTransformsRegistered();
        // TODO: this is not the case as I might be registering only the lower sweep.
//        if (roomIntermediateCloudTransformsRegistered.size() == roomIntermediateCloudTransforms.size())
        if (roomIntermediateCloudTransformsRegistered.size() > i)
        {
            saveTfStampedTransfromToXml(roomIntermediateCloudTransformsRegistered[i], xmlWriter, "RoomIntermediateCloudTransformRegistered");
        }

        // RoomIntermediateRoomCloudsCamParamsCorrected
        std::vector<image_geometry::PinholeCameraModel> roomIntermediateCloudCameraParametersCorrected = aRoom.getIntermediateCloudCameraParametersCorrected();
//        if (roomIntermediateCloudCameraParametersCorrected.size() == roomIntermediateCloudCameraParameters.size()) // consistency check
        if (roomIntermediateCloudCameraParametersCorrected.size() > i) // consistency check
        {
           saveCameraParametersToXML(roomIntermediateCloudCameraParametersCorrected[i], xmlWriter, "RoomIntermediateRoomCameraParametersCorrected");
        }

        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", "");
        xmlWriter->writeStartElement("RoomIntermediateCameraParameters");

        image_geometry::PinholeCameraModel aCameraModel = roomIntermediateCloudCameraParameters[i];
        const sensor_msgs::CameraInfo & camInfo = aCameraModel.cameraInfo();

        // size
        cv::Size imageSize = aCameraModel.fullResolution();
        xmlWriter->writeAttribute("width",QString::number(camInfo.width));
        xmlWriter->writeAttribute("height",QString::number(camInfo.height));
        xmlWriter->writeAttribute("distortion_model",QString(camInfo.distortion_model.c_str()));
        xmlWriter->writeAttribute("frame_id",QString(camInfo.header.frame_id.c_str()));

        // K matrix
        QString KString;
        for (size_t j=0; j<9;j++)
        {
            KString+=QString::number(camInfo.K[j]);
            KString+=",";
        }
        xmlWriter->writeAttribute("K",KString);
        //            ROS_INFO_STREAM("K matrix "<<KString.toStdString());

        // D matrix
        QString DString;
        for (size_t j=0; j<5;j++)
        {
            DString+=QString::number(camInfo.D[j]);
            DString+=",";
        }
        xmlWriter->writeAttribute("D",DString);
        //            ROS_INFO_STREAM("D matrix "<<DString.toStdString());

        // R matrix
        QString RString;
        for (size_t j=0; j<9;j++)
        {
            RString+=QString::number(camInfo.R[j]);
            RString+=",";
        }
        xmlWriter->writeAttribute("R",RString);
        //            ROS_INFO_STREAM("R matrix "<<RString.toStdString());

        // P matrix
        QString PString;
        for (size_t j=0; j<12;j++)
        {
            PString+=QString::number(camInfo.P[j]);
            PString+=",";
        }
        xmlWriter->writeAttribute("P",PString);
        //            ROS_INFO_STREAM("P matrix "<<PString.toStdString());


        xmlWriter->writeEndElement(); // RoomIntermediateCameraParameters

        xmlWriter->writeEndElement(); // RoomIntermediateCloud
    }
    //        }
    xmlWriter->writeEndElement(); // RoomIntermediateClouds

    // Intermediate cloud images
    saveIntermediateImagesToXML(aRoom,xmlWriter, roomFolder.toStdString());


    xmlWriter->writeEndElement(); // Semantic Room

    xmlWriter->writeEndDocument();
    delete xmlWriter;

    ROS_INFO_STREAM("... done saving.");
    return xmlFile;

}

template <class PointType>
void SemanticRoomXMLParser<PointType>::saveIntermediateImagesToXML(SemanticRoom<PointType>& aRoom, QXmlStreamWriter* xmlWriter, std::string roomFolder)
{

    auto v_intermdiate_images = aRoom.getIntermdiatePositionImages();

    if (v_intermdiate_images.size() > 0)
    {
        xmlWriter->writeStartElement("RoomIntermediateImages");

        size_t i, j;

        auto save_image = [&](cv::Mat image, std::string base_name)
        {
            std::stringstream ss; ss<<base_name; ss<<"_"<<std::setfill('0')<<std::setw(4)<<i<<"_"<<std::setfill('0')<<std::setw(4)<<j<<".png";
            cv::imwrite(ss.str().c_str(),image);
            ROS_INFO_STREAM("Saving intermediate image: "<<ss.str().c_str());
        };

        for (i=0; i<v_intermdiate_images.size();i++)
        {
            xmlWriter->writeStartElement("IntermediatePosition");

            xmlWriter->writeAttribute("RGB_Images", QString::number(v_intermdiate_images[i].numRGBImages));
            xmlWriter->writeAttribute("Depth_Images", QString::number(v_intermdiate_images[i].numDepthImages));

            // ------------------------------  Camera Parameters ----------------------------------------------------
            saveCameraParametersToXML(v_intermdiate_images[i].intermediateRGBCamParams, xmlWriter, "RGBCameraParameters");
            saveCameraParametersToXML(v_intermdiate_images[i].intermediateDepthCamParams, xmlWriter, "DepthCameraParameters");
            // ------------------------------  Camera Parameters ----------------------------------------------------

            // ------------------------------   RGB & Depth Transform  ----------------------------------------------------
            saveTfStampedTransfromToXml(v_intermdiate_images[i].intermediateRGBTransform, xmlWriter, "RGBTransform");
            saveTfStampedTransfromToXml(v_intermdiate_images[i].intermediateDepthTransform, xmlWriter, "DepthTransform");
            // ------------------------------   RGB & Depth Transform  ----------------------------------------------------


            xmlWriter->writeEndElement(); // IntermediatePosition

            if (v_intermdiate_images[i].images_loaded)
            {
                for (j=0; j<v_intermdiate_images[i].vIntermediateDepthImages.size();j++)
                {
                    std::string image_root_name = roomFolder + "/depth_image";
                    save_image(v_intermdiate_images[i].vIntermediateDepthImages[j],image_root_name);
                }
                for (j=0; j<v_intermdiate_images[i].vIntermediateRGBImages.size();j++)
                {
                    std::string image_root_name = roomFolder+ "/rgb_image";
                    save_image(v_intermdiate_images[i].vIntermediateRGBImages[j],image_root_name);
                }
            }
        }

        xmlWriter->writeEndElement(); // RoomIntermediateImages
    }

}


template <class PointType>
SemanticRoom<PointType> SemanticRoomXMLParser<PointType>::loadRoomFromXML(const std::string& xmlFile, bool deepLoad, bool verbose)
{
    ROS_INFO_STREAM("Loading room from "<<xmlFile);
    SemanticRoom<PointType> aRoom;


    QFile file(xmlFile.c_str());

    if (!file.exists())
    {
        std::cerr<<"Could not open file "<<xmlFile<<" to load room."<<std::endl;
        return aRoom;
    }

    // room folder
    QString xmlFileQS(xmlFile.c_str());
    int index = xmlFileQS.lastIndexOf('/');
    QString roomFolder = xmlFileQS.left(index);

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
                    int sl_index = roomCompleteCloudFile.indexOf('/');
                    if (sl_index == -1)
                    {
                        roomCompleteCloudFile=roomFolder + "/" + roomCompleteCloudFile;
                    }
                    std::ifstream file(roomCompleteCloudFile.toStdString().c_str());

                    if (deepLoad && file)
                    {
                        if (verbose)
                        {
                            std::cout<<"Loading complete cloud file name "<<roomCompleteCloudFile.toStdString()<<std::endl;
                        }
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
                    int sl_index = roomInteriorCloudFile.indexOf('/');
                    if (sl_index == -1)
                    {
                        roomInteriorCloudFile=roomFolder + "/" + roomInteriorCloudFile;
                    }
                    std::ifstream file(roomInteriorCloudFile.toStdString().c_str());
                    if (deepLoad && file)
                    {
                        if (verbose)
                        {
                            std::cout<<"Loading interior cloud file name "<<roomInteriorCloudFile.toStdString()<<std::endl;
                        }
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
                    int sl_index = roomDenoisedCloudFile.indexOf('/');
                    if (sl_index == -1)
                    {
                        roomDenoisedCloudFile=roomFolder + "/" + roomDenoisedCloudFile;
                    }

                    std::ifstream file(roomDenoisedCloudFile.toStdString().c_str());
                    if (deepLoad && file)
                    {
                        if (verbose)
                        {
                            std::cout<<"Loading denoised cloud file name "<<roomDenoisedCloudFile.toStdString()<<std::endl;
                        }
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
                    int sl_index = roomDynamicClustersFile.indexOf('/');
                    if (sl_index == -1)
                    {
                        roomDynamicClustersFile=roomFolder + "/" + roomDynamicClustersFile;
                    }

                    std::ifstream file(roomDynamicClustersFile.toStdString().c_str());
                    if (deepLoad && file)
                    {
                        if (verbose)
                        {
                            std::cout<<"Loading dynamic clusters cloud file name "<<roomDynamicClustersFile.toStdString()<<std::endl;
                        }
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

            if (xmlReader->name() == "RoomIntermediateClouds")
            {
                QXmlStreamAttributes attributes = xmlReader->attributes();
                if (attributes.hasAttribute("pan_start"))
                {
                    QString val = attributes.value("pan_start").toString();
                    aRoom.pan_start = val.toInt();
                }
                if (attributes.hasAttribute("pan_step"))
                {
                    QString val = attributes.value("pan_step").toString();
                    aRoom.pan_step = val.toInt();
                }
                if (attributes.hasAttribute("pan_end"))
                {
                    QString val = attributes.value("pan_end").toString();
                    aRoom.pan_end = val.toInt();
                }
                if (attributes.hasAttribute("tilt_start"))
                {
                    QString val = attributes.value("tilt_start").toString();
                    aRoom.tilt_start = val.toInt();
                }
                if (attributes.hasAttribute("tilt_step"))
                {
                    QString val = attributes.value("tilt_step").toString();
                    aRoom.tilt_step = val.toInt();
                }
                if (attributes.hasAttribute("tilt_end"))
                {
                    QString val = attributes.value("tilt_end").toString();
                    aRoom.tilt_end = val.toInt();
                }
            }


            if (xmlReader->name() == "RoomIntermediateCloud")
            {
                IntermediateCloudData intermediateCloudData = parseRoomIntermediateCloudNode(*xmlReader);
                image_geometry::PinholeCameraModel aCameraModel;
                aCameraModel.fromCameraInfo(intermediateCloudData.camInfo);

                QString intermediateCloudFilename(intermediateCloudData.filename.c_str());

                int sl_index = intermediateCloudFilename.indexOf('/');
                if (sl_index == -1)
                {
                    intermediateCloudData.filename=roomFolder.toStdString() + "/" + intermediateCloudData.filename;
                }

                std::ifstream file(intermediateCloudData.filename.c_str());
                if (deepLoad && file)
                {
                    if (verbose)
                    {
                        std::cout<<"Loading intermediate cloud file name "<<intermediateCloudData.filename<<std::endl;
                    }
                    pcl::PCDReader reader;
                    CloudPtr cloud (new Cloud);
                    reader.read (intermediateCloudData.filename, *cloud);
                    aRoom.addIntermediateRoomCloud(cloud, intermediateCloudData.transform,aCameraModel);
                } else {
                    aRoom.addIntermediateRoomCloud(intermediateCloudData.filename, intermediateCloudData.transform,aCameraModel);
                }

                if (intermediateCloudData.hasRegTransform)
                {
                    aRoom.addIntermediateRoomCloudRegisteredTransform(intermediateCloudData.regTransform);
                }

                if (intermediateCloudData.hasCorCamInfo)
                {
                   aRoom.addIntermediateCloudCameraParametersCorrected(intermediateCloudData.corCamInfo);
                }

            }

            if (xmlReader->name() == "IntermediatePosition")
            {
                auto positionImages = parseIntermediatePositionImages(xmlReader, roomFolder.toStdString(), deepLoad);

                aRoom.addIntermediateCloudImages(positionImages);
            }
        }
    }

    delete xmlReader;


    ROS_INFO_STREAM("... finished loading.");
    return aRoom;
}

/********************************************READING ************************************************************************/

template <class PointType>
typename SemanticRoomXMLParser<PointType>::IntermediateCloudData SemanticRoomXMLParser<PointType>::parseRoomIntermediateCloudNode(QXmlStreamReader& xmlReader)
{
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped tfmsg;
    tf::StampedTransform regTfmsg;
    sensor_msgs::CameraInfo camInfo;
    image_geometry::PinholeCameraModel corCamParam;
    bool camInfoError = false;
    bool regTfmsgError = true;
    bool corCamParamError = true;

    SemanticRoomXMLParser<PointType>::IntermediateCloudData       structToRet;
    structToRet.hasRegTransform = false;

    //        toRet.first = CloudPtr(new Cloud);
    QString intermediateParentNode("");

    if (xmlReader.name()!="RoomIntermediateCloud")
    {
        ROS_ERROR("Cannot parse RoomIntermediateCloud node, it has a different name: %s",xmlReader.name().toString().toStdString().c_str());
        return structToRet;
    }
    QXmlStreamAttributes attributes = xmlReader.attributes();
    if (attributes.hasAttribute("filename"))
    {
        QString roomIntermediateCloudFile = attributes.value("filename").toString();
        structToRet.filename = roomIntermediateCloudFile.toStdString();

    } else {
        ROS_ERROR("RoomIntermediateCloud xml node does not have filename attribute. Aborting.");
        return structToRet;
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
            if (xmlReader.name() == "FrameId")
            {
                QString val = xmlReader.readElementText();
                tfmsg.header.frame_id = val.toStdString();
            }
            if (xmlReader.name() == "ChildFrameId")
            {
                QString val = xmlReader.readElementText();
                tfmsg.child_frame_id = val.toStdString();
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

            if (xmlReader.name() == "RoomIntermediateCloudTransformRegistered")
            {
                regTfmsg = readTfStampedTransformFromXml(&xmlReader, "RoomIntermediateCloudTransformRegistered", regTfmsgError);
            }

            if (xmlReader.name() == "RoomIntermediateRoomCameraParametersCorrected")
            {
                corCamParam = readCamParamsFromXml(&xmlReader, "RoomIntermediateRoomCameraParametersCorrected", corCamParamError);
            }



            // camera parameters
            if (xmlReader.name() == "RoomIntermediateCameraParameters")
            {
                QXmlStreamAttributes paramAttributes = xmlReader.attributes();

                if (paramAttributes.hasAttribute("height"))
                {
                    QString val = paramAttributes.value("height").toString();
                    camInfo.height = val.toInt();

                } else {
                    ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the height attribute. Cannot construct camera parameters object.");
                    camInfoError = true;
                }

                if (paramAttributes.hasAttribute("width"))
                {
                    QString val = paramAttributes.value("width").toString();
                    camInfo.width = val.toInt();

                } else {
                    ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the width attribute. Cannot construct camera parameters object.");
                    camInfoError = true;
                }

                if (paramAttributes.hasAttribute("distortion_model"))
                {
                    QString val = paramAttributes.value("distortion_model").toString();
                    camInfo.distortion_model = val.toStdString();

                } else {
                    ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the distortion_model attribute. Cannot construct camera parameters object.");
                    camInfoError = true;
                }

                if (paramAttributes.hasAttribute("frame_id"))
                {
                    QString val = paramAttributes.value("frame_id").toString();
                    camInfo.header.frame_id = val.toStdString();

                } else {
                    ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the frame_id attribute. Cannot construct camera parameters object.");
                    camInfoError = true;
                }

                if (paramAttributes.hasAttribute("K"))
                {
                    QString val = paramAttributes.value("K").toString();
                    QStringList val_list = val.split(",",QString::SkipEmptyParts);
                    if (val_list.size() != 9) // wrong number of parameters
                    {
                        ROS_ERROR("RoomIntermediateCameraParameters K attribute has more than 9 parameters. Cannot construct camera parameters object.");
                        camInfoError = true;
                    } else {
                        for (size_t j=0; j< val_list.size();j++)
                        {
                            camInfo.K[j] = val_list[j].toDouble();
                        }
                    }
                } else {
                    ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the K attribute. Cannot construct camera parameters object.");
                    camInfoError = true;
                }

                if (paramAttributes.hasAttribute("D"))
                {
                    QString val = paramAttributes.value("D").toString();
                    QStringList val_list = val.split(",",QString::SkipEmptyParts);
                    if (val_list.size() != 5) // wrong number of parameters
                    {
                        ROS_ERROR("RoomIntermediateCameraParameters D attribute has more than 5 parameters. Cannot construct camera parameters object.");
                        camInfoError = true;
                    } else {
                        for (size_t j=0; j< val_list.size();j++)
                        {
                            camInfo.D.push_back(val_list[j].toDouble());
                        }
                    }
                } else {
                    ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the D attribute. Cannot construct camera parameters object.");
                    camInfoError = true;
                }

                if (paramAttributes.hasAttribute("R"))
                {
                    QString val = paramAttributes.value("R").toString();
                    QStringList val_list = val.split(",",QString::SkipEmptyParts);
                    if (val_list.size() != 9) // wrong number of parameters
                    {
                        ROS_ERROR("RoomIntermediateCameraParameters R attribute has more than 9 parameters. Cannot construct camera parameters object.");
                        camInfoError = true;
                    } else {
                        for (size_t j=0; j< val_list.size();j++)
                        {
                            camInfo.R[j] = val_list[j].toDouble();
                        }
                    }
                } else {
                    ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the R attribute. Cannot construct camera parameters object.");
                    camInfoError = true;
                }

                if (paramAttributes.hasAttribute("P"))
                {
                    QString val = paramAttributes.value("P").toString();
                    QStringList val_list = val.split(",",QString::SkipEmptyParts);
                    if (val_list.size() != 12) // wrong number of parameters
                    {
                        ROS_ERROR("RoomIntermediateCameraParameters P attribute has more than 12 parameters. Cannot construct camera parameters object.");
                        camInfoError = true;
                    } else {
                        for (size_t j=0; j< val_list.size();j++)
                        {
                            camInfo.P[j] = val_list[j].toDouble();
                        }
                    }
                } else {
                    ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the P attribute. Cannot construct camera parameters object.");
                    camInfoError = true;
                }


            }

        }
        token = xmlReader.readNext();
    }

    if (!camInfoError)
    {
        structToRet.camInfo = camInfo;
    }
    if (!regTfmsgError)
    {
        structToRet.regTransform = regTfmsg;
        structToRet.hasRegTransform = true;
    } else {
        structToRet.hasRegTransform = false;
    }

    if (!corCamParamError)
    {
      structToRet.corCamInfo = corCamParam;
      structToRet.hasCorCamInfo = true;
    } else {
      structToRet.hasCorCamInfo = false;
    }

    tf::transformStampedMsgToTF(tfmsg, transform);
    structToRet.transform = transform;
    return structToRet;
}

template <class PointType>
typename SemanticRoomXMLParser<PointType>::IntermediatePositionImages
SemanticRoomXMLParser<PointType>::parseIntermediatePositionImages(QXmlStreamReader* xmlReader, std::string roomFolder, bool deepLoad)
{

    static int intermediatePoisitionCounter = 0;

    SemanticRoomXMLParser<PointType>::IntermediatePositionImages toRet;
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped tfmsg;
    sensor_msgs::CameraInfo camInfo;
    bool camInfoError = false;

    if (xmlReader->name()!="IntermediatePosition")
    {
        ROS_ERROR("Cannot parse IntermediatePosition node, it has a different name: %s",xmlReader->name().toString().toStdString().c_str());
        return toRet;
    }
    QXmlStreamAttributes attributes = xmlReader->attributes();

    // read in number of depth and rgb images
    int numRGB, numDepth;
    if (attributes.hasAttribute("RGB_Images"))
    {
        numRGB = attributes.value("RGB_Images").toString().toInt();
    }

    if (attributes.hasAttribute("Depth_Images"))
    {
        numDepth = attributes.value("Depth_Images").toString().toInt();
    }

    // read in the images

    if (deepLoad)
    {
        for (int i=0; i<numRGB;i++)
        {
            std::stringstream ss; ss<<roomFolder;ss<<"/rgb_image"; ss<<"_"<<std::setfill('0')<<std::setw(4)<<intermediatePoisitionCounter<<"_"<<std::setfill('0')<<std::setw(4)<<i<<".png";
            cv::Mat image = cv::imread(ss.str().c_str(), CV_LOAD_IMAGE_COLOR);
            ROS_INFO_STREAM("Loading intermediate image: "<<ss.str().c_str());
            toRet.vIntermediateRGBImages.push_back(image);
        };

        for (int i=0; i<numDepth;i++)
        {
            std::stringstream ss; ss<<roomFolder;ss<<"/depth_image"; ss<<"_"<<std::setfill('0')<<std::setw(4)<<intermediatePoisitionCounter<<"_"<<std::setfill('0')<<std::setw(4)<<i<<".png";
            cv::Mat image = cv::imread(ss.str().c_str(), CV_LOAD_IMAGE_ANYDEPTH);
            ROS_INFO_STREAM("Loading intermediate image: "<<ss.str().c_str());
            toRet.vIntermediateDepthImages.push_back(image);
        };
    }
    toRet.images_loaded = deepLoad;
    toRet.numRGBImages = numRGB;
    toRet.numDepthImages = numDepth;


    QXmlStreamReader::TokenType token = xmlReader->readNext();

    while(!((token == QXmlStreamReader::EndElement) && (xmlReader->name() == "IntermediatePosition")) )
    {

        if ((xmlReader->name() == "RGBTransform") && !(token == QXmlStreamReader::EndElement))
        {
            bool errorReading = false;
            toRet.intermediateRGBTransform = readTfStampedTransformFromXml(xmlReader, "RGBTransform", errorReading);
        }

        if ((xmlReader->name() == "DepthTransform") && !(token == QXmlStreamReader::EndElement))
        {
            bool errorReading = false;
            toRet.intermediateDepthTransform = readTfStampedTransformFromXml(xmlReader, "DepthTransform", errorReading);
        }

        if ((xmlReader->name() == "RGBCameraParameters") && !(token == QXmlStreamReader::EndElement))
        {
            bool errorReading = false;
            toRet.intermediateRGBCamParams = readCamParamsFromXml(xmlReader, "RGBCameraParameters", errorReading);
        }

        if ((xmlReader->name() == "DepthCameraParameters") && !(token == QXmlStreamReader::EndElement))
        {
            bool errorReading = false;
            toRet.intermediateDepthCamParams = readCamParamsFromXml(xmlReader, "DepthCameraParameters", errorReading);
        }

        token = xmlReader->readNext();
    }
    intermediatePoisitionCounter++;

    return toRet;

}

// Reads in a image_geometry::PinholeCameraModel from the current xml node
// Does not advance the xml reader!!
template <class PointType>
typename image_geometry::PinholeCameraModel SemanticRoomXMLParser<PointType>::readCamParamsFromXml(QXmlStreamReader* xmlReader, std::string nodeName, bool& errorReading)
{
    image_geometry::PinholeCameraModel toRet;
    sensor_msgs::CameraInfo camInfo;
    bool camInfoError = false;

    // camera parameters
    if (xmlReader->name() == nodeName.c_str())
    {
        QXmlStreamAttributes paramAttributes = xmlReader->attributes();

        if (paramAttributes.hasAttribute("height"))
        {
            QString val = paramAttributes.value("height").toString();
            camInfo.height = val.toInt();

        } else {
            ROS_ERROR("%s xml node does not have the height attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
            camInfoError = true;
        }

        if (paramAttributes.hasAttribute("width"))
        {
            QString val = paramAttributes.value("width").toString();
            camInfo.width = val.toInt();

        } else {
            ROS_ERROR("%s xml node does not have the width attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
            camInfoError = true;
        }

        if (paramAttributes.hasAttribute("distortion_model"))
        {
            QString val = paramAttributes.value("distortion_model").toString();
            camInfo.distortion_model = val.toStdString();

        } else {
            ROS_ERROR("%s xml node does not have the distortion_model attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
            camInfoError = true;
        }

        if (paramAttributes.hasAttribute("frame_id"))
        {
            QString val = paramAttributes.value("frame_id").toString();
            camInfo.header.frame_id = val.toStdString();

        } else {
            ROS_ERROR("%s xml node does not have the frame_id attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
            camInfoError = true;
        }

        if (paramAttributes.hasAttribute("K"))
        {
            QString val = paramAttributes.value("K").toString();
            QStringList val_list = val.split(",",QString::SkipEmptyParts);
            if (val_list.size() != 9) // wrong number of parameters
            {
                ROS_ERROR("%s K attribute has more than 9 parameters. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            } else {
                for (size_t j=0; j< val_list.size();j++)
                {
                    camInfo.K[j] = val_list[j].toDouble();
                }
            }
        } else {
            ROS_ERROR("%s xml node does not have the K attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
            camInfoError = true;
        }

        if (paramAttributes.hasAttribute("D"))
        {
            QString val = paramAttributes.value("D").toString();
            QStringList val_list = val.split(",",QString::SkipEmptyParts);
            if (val_list.size() != 5) // wrong number of parameters
            {
                ROS_ERROR("%s D attribute has more than 5 parameters. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            } else {
                for (size_t j=0; j< val_list.size();j++)
                {
                    camInfo.D.push_back(val_list[j].toDouble());
                }
            }
        } else {
            ROS_ERROR("%s xml node does not have the D attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
            camInfoError = true;
        }

        if (paramAttributes.hasAttribute("R"))
        {
            QString val = paramAttributes.value("R").toString();
            QStringList val_list = val.split(",",QString::SkipEmptyParts);
            if (val_list.size() != 9) // wrong number of parameters
            {
                ROS_ERROR("%s R attribute has more than 9 parameters. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            } else {
                for (size_t j=0; j< val_list.size();j++)
                {
                    camInfo.R[j] = val_list[j].toDouble();
                }
            }
        } else {
            ROS_ERROR("%s xml node does not have the R attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
            camInfoError = true;
        }

        if (paramAttributes.hasAttribute("P"))
        {
            QString val = paramAttributes.value("P").toString();
            QStringList val_list = val.split(",",QString::SkipEmptyParts);
            if (val_list.size() != 12) // wrong number of parameters
            {
                ROS_ERROR("%s P attribute has more than 12 parameters. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            } else {
                for (size_t j=0; j< val_list.size();j++)
                {
                    camInfo.P[j] = val_list[j].toDouble();
                }
            }
        } else {
            ROS_ERROR("%s xml node does not have the P attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
            camInfoError = true;
        }
    } else {
        ROS_ERROR("Error while attempting to construct camera parameters object from node name %s  Node expected %s",xmlReader->name().toString().toStdString().c_str(), nodeName.c_str());
        camInfoError = true;
    }

    if (!camInfoError)
    {
        toRet.fromCameraInfo(camInfo);
    }

    errorReading = camInfoError;

    return toRet;


}

// Reads in a TfStampedTransform from the current xml node
// Does not advance the xml reader!!

template <class PointType>
tf::StampedTransform SemanticRoomXMLParser<PointType>::readTfStampedTransformFromXml(QXmlStreamReader* xmlReader, std::string nodeName, bool& errorReading)
{

    errorReading = false;
    geometry_msgs::TransformStamped tfmsg;
    tf::StampedTransform transform;

    if (xmlReader->name() == nodeName.c_str())
    {
        errorReading = false;
        QXmlStreamAttributes paramAttributes = xmlReader->attributes();

        if (paramAttributes.hasAttribute("Stamp_sec"))
        {
            QString val = paramAttributes.value("Stamp_sec").toString();
            tfmsg.header.stamp.sec = val.toInt();

        } else {
            ROS_ERROR("%s xml node does not have the Stamp_sec attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Stamp_nsec"))
        {
            QString val = paramAttributes.value("Stamp_nsec").toString();
            tfmsg.header.stamp.nsec = val.toInt();

        } else {
            ROS_ERROR("%s xml node does not have the Stamp_nsec attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("FrameId"))
        {
            QString val = paramAttributes.value("FrameId").toString();
            tfmsg.header.frame_id = val.toStdString();

        } else {
            ROS_ERROR("%s xml node does not have the FrameId attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("ChildFrameId"))
        {
            QString val = paramAttributes.value("ChildFrameId").toString();
            tfmsg.child_frame_id = val.toStdString();

        } else {
            ROS_ERROR("%s xml node does not have the ChildFrameId attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Trans_x"))
        {
            QString val = paramAttributes.value("Trans_x").toString();
            tfmsg.transform.translation.x = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Trans_x attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Trans_y"))
        {
            QString val = paramAttributes.value("Trans_y").toString();
            tfmsg.transform.translation.y = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Trans_y attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Trans_z"))
        {
            QString val = paramAttributes.value("Trans_z").toString();
            tfmsg.transform.translation.z = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Trans_z attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Rot_w"))
        {
            QString val = paramAttributes.value("Rot_w").toString();
            tfmsg.transform.rotation.w = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Rot_w attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Rot_x"))
        {
            QString val = paramAttributes.value("Rot_x").toString();
            tfmsg.transform.rotation.x = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Rot_x attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Rot_y"))
        {
            QString val = paramAttributes.value("Rot_y").toString();
            tfmsg.transform.rotation.y = val.toDouble();

        } else {
            ROS_ERROR("%s xml node does not have the Rot_y attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
            errorReading = true;
        }
        if (paramAttributes.hasAttribute("Rot_z"))
        {
            QString val = paramAttributes.value("Rot_z").toString();
            tfmsg.transform.rotation.z = val.toDouble();

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
        tf::transformStampedMsgToTF(tfmsg, transform);
    }

    return transform;

}

/******************************************** WRITING  ************************************************************************/
template <class PointType>
void SemanticRoomXMLParser<PointType>::saveTfStampedTransfromToXml(tf::StampedTransform transform, QXmlStreamWriter* xmlWriter, std::string nodeName)
{
    geometry_msgs::TransformStamped msg_reg;
    tf::transformStampedTFToMsg(transform, msg_reg);

    xmlWriter->writeStartElement(nodeName.c_str());
    xmlWriter->writeAttribute("Stamp_sec",QString::number(msg_reg.header.stamp.sec));
    xmlWriter->writeAttribute("Stamp_nsec",QString::number(msg_reg.header.stamp.nsec));
    xmlWriter->writeAttribute("FrameId",QString(msg_reg.header.frame_id.c_str()));
    xmlWriter->writeAttribute("ChildFrameId",QString(msg_reg.child_frame_id.c_str()));
    xmlWriter->writeAttribute("Trans_x",QString::number(msg_reg.transform.translation.x));
    xmlWriter->writeAttribute("Trans_y",QString::number(msg_reg.transform.translation.y));
    xmlWriter->writeAttribute("Trans_z",QString::number(msg_reg.transform.translation.z));
    xmlWriter->writeAttribute("Rot_w",QString::number(msg_reg.transform.rotation.w));
    xmlWriter->writeAttribute("Rot_x",QString::number(msg_reg.transform.rotation.x));
    xmlWriter->writeAttribute("Rot_y",QString::number(msg_reg.transform.rotation.y));
    xmlWriter->writeAttribute("Rot_z",QString::number(msg_reg.transform.rotation.z));

    xmlWriter->writeEndElement();

}

template <class PointType>
void SemanticRoomXMLParser<PointType>::saveCameraParametersToXML(image_geometry::PinholeCameraModel cam_model, QXmlStreamWriter* xmlWriter, std::string nodeName)
{
    xmlWriter->writeStartElement(nodeName.c_str());

    const sensor_msgs::CameraInfo & camInfo = cam_model.cameraInfo();

    // size
    cv::Size imageSize = cam_model.fullResolution();
    xmlWriter->writeAttribute("width",QString::number(camInfo.width));
    xmlWriter->writeAttribute("height",QString::number(camInfo.height));
    xmlWriter->writeAttribute("distortion_model",QString(camInfo.distortion_model.c_str()));
    xmlWriter->writeAttribute("frame_id",QString(camInfo.header.frame_id.c_str()));

    // K matrix
    QString KString;
    for (size_t j=0; j<9;j++)
    {
        KString+=QString::number(camInfo.K[j]);
        KString+=",";
    }
    xmlWriter->writeAttribute("K",KString);

    // D matrix
    QString DString;
    for (size_t j=0; j<5;j++)
    {
        DString+=QString::number(camInfo.D[j]);
        DString+=",";
    }
    xmlWriter->writeAttribute("D",DString);

    // R matrix
    QString RString;
    for (size_t j=0; j<9;j++)
    {
        RString+=QString::number(camInfo.R[j]);
        RString+=",";
    }
    xmlWriter->writeAttribute("R",RString);

    // P matrix
    QString PString;
    for (size_t j=0; j<12;j++)
    {
        PString+=QString::number(camInfo.P[j]);
        PString+=",";
    }
    xmlWriter->writeAttribute("P",PString);

    xmlWriter->writeEndElement();
}
