#include <mongodb_store/message_store.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "dynamic_object_xml_parser.h"

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

    ros::init(argc, argv, "load_objects_from_mongo_node");
    ros::NodeHandle n;

    mongodb_store::MessageStoreProxy message_store_summary(n,"summary","metric_maps");
    mongodb_store::MessageStoreProxy message_store_data(n,"data","metric_maps");

    std::vector< boost::shared_ptr<std_msgs::String> > results;
    if(message_store_summary.queryNamed<std_msgs::String>("dynamic_object_xml", results,false)) {
        cout<<"Found "<<results.size()<<" results."<<endl;


        for (int i=0; i<results.size();i++)
        {
            stringstream temp_ss;
            temp_ss<<folderPath;temp_ss<<"/";
            temp_ss<<"object";temp_ss<<i;temp_ss<<".xml";
            ofstream file;
            file.open(temp_ss.str());
            if (!file.is_open())
            {
                ROS_WARN_STREAM("Could not save room xml "<<temp_ss.str());
                continue;
            }
            file<<results[i]->data;
            file.close();

            ROS_INFO_STREAM("Saving "<<temp_ss.str());

            DynamicObjectXMLParser parser;
            DynamicObject::Ptr parsed = parser.loadFromXML(temp_ss.str(), false); // don't load the cloud. not there anyway

            // create folder structure
            QString room_log_name(parsed->m_roomLogString.c_str());
            int index = room_log_name.indexOf('_');
            QString date = room_log_name.left(index);
            QString patrol = room_log_name.right(room_log_name.length() - index-1);

            QString full_date = QString(folderPath.c_str()) + "/" + date;
            QString full_patrol = full_date + "/" + patrol;
            QString full_room = full_patrol + "/room_" + QString::number(parsed->m_roomRunNumber);

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

            // save object cloud
//            QString room_name = "room_sweep_";

            QString object_name = QString(parsed->m_label.c_str());

            // complete cloud
            {
                stringstream ss;ss<<object_name.toStdString()<<"___object_cloud";
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
                            parsed->setCloud(databaseCloud);
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

            // additonal views cloud
            {
                for (size_t i=0; i<parsed->m_noAdditionalViews; i++)
                {
                    stringstream ss;ss<<object_name.toStdString();ss<<"___additional_view_";ss<<i;

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
                                    parsed->m_vAdditionalViews.push_back(databaseCloud);
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
            }

            DynamicObjectXMLParser save_parser(full_room.toStdString());
            string saved_xml = save_parser.saveAsXML(parsed);

            QFile temp_room_file(temp_ss.str().c_str());
            if (!temp_room_file.remove())
            {
                ROS_WARN_STREAM("Could not delete temporary room xml file "<<temp_ss.str());
            }

        }



    }

}
