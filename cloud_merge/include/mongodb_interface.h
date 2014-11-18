#ifndef __MONGODB_INTERFACE
#define __MONGODB_INTERFACE


#include <mongodb_store/message_store.h>

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <QString>

#include <semantic_map/room.h>
#include <semantic_map/semantic_map_summary_parser.h>

class MongodbInterface {
public:


    MongodbInterface(ros::NodeHandle nh);
    ~MongodbInterface();

    template <class PointType>
    bool logRoomToDB(SemanticRoom<PointType> aRoom, std::string roomXMLPath)
    {

        typename pcl::PointCloud<PointType> Cloud;
        typename pcl::PointCloud<PointType>::Ptr CloudPtr;

        // first add xml file
        std::ifstream file(roomXMLPath);
        std::stringstream buffer;
        buffer << file.rdbuf();

        std_msgs::String room_name_ros;
        room_name_ros.data = buffer.str();

        mongo::BSONObjBuilder builder;
        builder.append("name", "metric_map_room_xml");
        mongo::BSONObj obj = builder.obj();

        std::string id(m_messageStoreSummary.insert(room_name_ros, "metric_maps" ,"summary" ,obj));
        ROS_INFO_STREAM("Room \""<<roomXMLPath<<"\" inserted with id "<<id);

        QString room_name = aRoom.getRoomLogName().c_str() + QString("___") + QString::number(aRoom.getRoomRunNumber());

        // save complete cloud
        {
            std::stringstream ss;
            ss<<room_name.toStdString();ss<<"___";
            ss<<"complete_cloud";

            mongo::BSONObjBuilder builder;
            builder.append("name", ss.str());
            mongo::BSONObj obj = builder.obj();

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*aRoom.getCompleteRoomCloud(), msg_cloud);

            std::string id(m_messageStoreData.insert(msg_cloud, "metric_maps" ,"data" ,obj));
            ROS_INFO_STREAM("Complete cloud \""<<ss.str()<<"\" inserted with id "<<id);
        }

        // save intermediate clouds

        for (int j=0; j<aRoom.getIntermediateClouds().size();j++)
        {
            std::stringstream ss;
            ss<<room_name.toStdString();ss<<"___";
            ss<<"intermediate_cloud_";ss<<j;

            mongo::BSONObjBuilder builder;
            builder.append("name", ss.str());
            mongo::BSONObj obj = builder.obj();

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*aRoom.getIntermediateClouds()[j], msg_cloud);

            std::string id(m_messageStoreData.insert(msg_cloud, "metric_maps" ,"data" ,obj));
            ROS_INFO_STREAM("Intermediate cloud \""<<ss.str()<<"\" inserted with id "<<id);

        }


        return true;
    }

    template <class PointType>
    std::vector<typename SemanticMapSummaryParser<PointType>::EntityStruct> getMongodbRooms()
    {
        typedef typename SemanticMapSummaryParser<PointType>::EntityStruct Entities;

        std::vector<Entities> toRet;

        std::vector< boost::shared_ptr<std_msgs::String> > results;
        if(m_messageStoreSummary.queryNamed<std_msgs::String>("metric_map_room_xml", results,false)) {
            ROS_INFO_STREAM("Found "<<results.size()<<" results.");

            BOOST_FOREACH(boost::shared_ptr<std_msgs::String> p,  results)
            {
                Entities new_room;
                 // Test room log name, run number and centroid
                QString room_file(p->data.c_str());

                int index1 = room_file.indexOf("<RoomLogName>");
                int index2 = room_file.indexOf("</RoomLogName>");
                int start = index1 + QString("<RoomLogName>").length();
                QString room_log_name_from_xml = room_file.mid(start,index2-start);

                index1 = room_file.indexOf("<RoomRunNumber>");
                index2 = room_file.indexOf("</RoomRunNumber>");
                start = index1 + QString("<RoomRunNumber>").length();
                QString room_run_number_from_xml = room_file.mid(start,index2-start);

                index1 = room_file.indexOf("<Centroid>");
                index2 = room_file.indexOf("</Centroid>");
                start = index1 + QString("<Centroid>").length();
                QString centroidS = room_file.mid(start,index2-start);

                ROS_INFO_STREAM("Room saved in mongodb: "<<room_log_name_from_xml.toStdString()<<"  "<<room_run_number_from_xml.toStdString()<<"  "<<centroidS.toStdString());

                new_room.entityType = SEMANTIC_MAP_ROOM;
                QString room_xml_file = room_log_name_from_xml + "/room_" + room_run_number_from_xml;
                Eigen::Vector4f centroid;
                QStringList centroidSlist = centroidS.split(' ');
                centroid(0) = centroidSlist[0].toDouble();centroid(1) = centroidSlist[1].toDouble();
                centroid(2) = centroidSlist[2].toDouble();centroid(3) = centroidSlist[3].toDouble();

                new_room.roomXmlFile = room_xml_file.toStdString();
                new_room.roomLogName = room_log_name_from_xml.toStdString();
                new_room.centroid = centroid;
                new_room.isMetaRoom = false;

                toRet.push_back(new_room);
            }

        }

        return toRet;
    }



private:
    ros::NodeHandle                                                             m_NodeHandle;
    mongodb_store::MessageStoreProxy                                           m_messageStoreData;
    mongodb_store::MessageStoreProxy                                           m_messageStoreSummary;

};




#endif // __MONGODB_INTERFACE
