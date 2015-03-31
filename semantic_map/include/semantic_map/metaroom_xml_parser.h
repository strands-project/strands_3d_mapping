#ifndef __META_ROOM_XML_PARSER__H
#define __META_ROOM_XML_PARSER__H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/distances.h>

#include "ros/time.h"
#include "ros/serialization.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tf/tf.h"

#include "constants.h"
#include "metaroom_update_iteration.h"

// QT
#include <QFile>
#include <QDir>
#include <QXmlStreamWriter>


template <class PointType>
class MetaRoom;

template <class PointType>
class MetaRoomXMLParser {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;


    MetaRoomXMLParser(std::string rootFolder="home");
    ~MetaRoomXMLParser();

    std::string saveMetaRoomAsXML(MetaRoom<PointType>& aMetaRoom, std::string xmlFile="metaroom.xml");

    static MetaRoom<PointType> loadMetaRoomFromXML(const std::string& xmlFile, bool deepLoad=true);

    QString findMetaRoomLocation(MetaRoom<PointType>* aMetaRoom);
private:

    QString                                 m_RootFolder;
};

#include "metaroom_xml_parser.hpp"

#endif
