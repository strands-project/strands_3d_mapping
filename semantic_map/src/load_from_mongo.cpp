#include <mongodb_store/message_store.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <semantic_map/room_xml_parser.h>

#include <QFile>
#include <QDir>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

using namespace std;

int main(int argc, char** argv)
{

    if (argc < 2)
    {
        cout<<"Please provide folder where to save the data."<<endl;
        return -1;
    }

    string folderPath = argv[1] + string("/");

    if (!QDir(folderPath.c_str()).exists())
    {
        QDir().mkdir(folderPath.c_str());
    }

    ros::init(argc, argv, "load_from_mongo_node");
    ros::NodeHandle n;

    mongodb_store::MessageStoreProxy message_store_summary(n,"summary","metric_maps");
    mongodb_store::MessageStoreProxy message_store_data(n,"data","metric_maps");

    std::vector< boost::shared_ptr<std_msgs::String> > results;
    if(message_store_summary.queryNamed<std_msgs::String>("metric_map_room_xml", results,false)) {
        cout<<"Found "<<results.size()<<" results."<<endl;

        for (int i=0; i<results.size();i++)
        {
            stringstream temp_room_ss;
            temp_room_ss<<folderPath;temp_room_ss<<"/";
            temp_room_ss<<"room";temp_room_ss<<i;temp_room_ss<<".xml";
            ofstream file;
            file.open(temp_room_ss.str());
            if (!file.is_open())
            {
                ROS_WARN_STREAM("Could not save room xml "<<temp_room_ss.str());
                continue;
            }
            file<<results[i]->data;
            file.close();

            ROS_INFO_STREAM("Saving "<<temp_room_ss.str());

            SemanticRoomXMLParser<PointType> parser;
            SemanticRoom<PointType> aRoom = parser.loadRoomFromXML(temp_room_ss.str(),false);
            cout<<aRoom.getRoomLogName()<<"  "<<aRoom.getRoomRunNumber()<<endl;

            // create folder structure
            QString room_log_name(aRoom.getRoomLogName().c_str());
            int index = room_log_name.indexOf('_');
            QString date = room_log_name.left(index);
            QString patrol = room_log_name.right(room_log_name.length() - index-1);

            QString full_date = QString(folderPath.c_str()) + "/" + date;
            QString full_patrol = full_date + "/" + patrol;
            QString full_room = full_patrol + "/room_" + QString::number(aRoom.getRoomRunNumber());

            if (!QDir(full_date).exists())
            {
                QDir().mkdir(full_date);
            }

            if (!QDir(full_patrol).exists())
            {
                QDir().mkdir(full_patrol);
            }

            if (!QDir(full_room).exists())
            {
                QDir().mkdir(full_room);
            }

            // save intermediate and complete point clouds
//            QString room_name = "room_sweep_";
            QString room_name = QString(aRoom.getRoomLogName().c_str()) + QString("___") + QString::number(aRoom.getRoomRunNumber());

            for (int j=0; j<aRoom.getIntermediateCloudTransforms().size();j++)
            {
//                stringstream ss;ss<<room_name.toStdString()<<i<<"___intermediate_cloud_"<<j;
                stringstream ss;ss<<room_name.toStdString()<<"___intermediate_cloud_"<<j;
                sensor_msgs::PointCloud2 msg_cloud;
                std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > results;

                if(message_store_data.queryNamed<sensor_msgs::PointCloud2>(ss.str(), results,false)) {
                    if (results.size() == 1)
                    {
                        ROS_INFO_STREAM("Found "<<ss.str());
                        CloudPtr databaseCloud(new Cloud());
                        pcl::fromROSMsg(*results[0],*databaseCloud);

                        stringstream cloud_ss; cloud_ss << "intermediate_cloud"<<std::setfill('0')<<std::setw(4)<<j<<".pcd";
                        QString cloud_name = full_room + "/"+QString(cloud_ss.str().c_str());

                        if (databaseCloud->points.size()>0)
                        {
                            pcl::io::savePCDFileBinary(cloud_name.toStdString(), *databaseCloud);
                        }
                    } else {
                      ROS_WARN_STREAM("Found multiple matches for element named "<<ss.str());
                    }

                } else {
                    ROS_WARN_STREAM("Cannot find element named "<<ss.str());
                }
            }

            // complete cloud
            {
                stringstream ss;ss<<room_name.toStdString()<<"___complete_cloud";
                sensor_msgs::PointCloud2 msg_cloud;
                std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > results;

                if(message_store_data.queryNamed<sensor_msgs::PointCloud2>(ss.str(), results,false)) {
                    if (results.size() == 1)
                    {
                        ROS_INFO_STREAM("Found "<<ss.str());
                        CloudPtr databaseCloud(new Cloud());
                        pcl::fromROSMsg(*results[0],*databaseCloud);
                        QString cloud_name = full_room + "/"+QString("complete_cloud.pcd");

                        try {
                        if (databaseCloud->points.size()>0)
                        {
                            pcl::io::savePCDFileBinary(cloud_name.toStdString(), *databaseCloud);
                        }
                        } catch (...)
                        {

                        }

                    } else {
                        ROS_WARN_STREAM("Found multiple matches for element named "<<ss.str());
                    }

                } else {
                    ROS_WARN_STREAM("Cannot find element named "<<ss.str());
                }
            }

            QString full_xml = full_room+"/room.xml";

            file.open(full_xml.toStdString());
            if (!file.is_open())
            {
                ROS_WARN_STREAM("Could not save room xml "<<full_xml.toStdString());
                continue;
            }
            file<<results[i]->data;
            file.close();

            QFile temp_room_file(temp_room_ss.str().c_str());
            if (!temp_room_file.remove())
            {
                ROS_WARN_STREAM("Could not delete temporary room xml file "<<temp_room_ss.str());
            }

        }

    }

}
