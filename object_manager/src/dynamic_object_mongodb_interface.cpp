#include "dynamic_object_mongodb_interface.h"
#include <boost/filesystem.hpp>

using namespace std;

bool DynamicObjectMongodbInterface::logToDB(DynamicObject::Ptr object, std::string objectXMLPath)
{

//    typename pcl::PointCloud<PointType> Cloud;
//    typename pcl::PointCloud<PointType>::Ptr CloudPtr;

    if ( ! boost::filesystem::exists( objectXMLPath ) )
    {
        ROS_ERROR_STREAM("Supplied dynamic object xml path does not exist. Cannot save to mongo db. "+objectXMLPath);
        return false;
    }

    bool update = false;
    // save object xml
    {

        {
            std::stringstream ss;
            ss<<object->m_label;ss<<"___";ss<<"object_cloud";
            std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > results;
            if(m_messageStoreData.queryNamed<sensor_msgs::PointCloud2>(ss.str(), results,false)) {
                if (results.size()>0) // already inserted, update
                {
                    update = true;
                }
            }
        }

        std::ifstream file(objectXMLPath);
        std::stringstream buffer;
        buffer << file.rdbuf();

        std_msgs::String object_name_ros;
        object_name_ros.data = buffer.str();

        mongo::BSONObjBuilder builder;
        builder.append("name", "dynamic_object_xml");
        mongo::BSONObj obj = builder.obj();

        // check if it's already been inserted. If yes, update
        std::string id("");
        if (!update)
        {
            id = (m_messageStoreSummary.insert(object_name_ros, "metric_maps" ,"summary" ,obj));
        } else {
            id = (m_messageStoreSummary.update(object_name_ros));
        }

        if (m_verbose)
        {
            ROS_INFO_STREAM("object \""<<objectXMLPath<<"\" inserted with id "<<id);
        }
    }

    string object_label = object->m_label;
    // save object cloud
    {
        std::stringstream ss;
        ss<<object_label;ss<<"___";ss<<"object_cloud";

        mongo::BSONObjBuilder builder;
        builder.append("name", ss.str());
        mongo::BSONObj obj = builder.obj();

        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*object->m_points, msg_cloud);

        std::string id("");
        id = (m_messageStoreData.updateNamed(ss.str(),msg_cloud,true));

        if (m_verbose)
        {
            ROS_INFO_STREAM("Object cloud \""<<ss.str()<<"\" inserted with id "<<id);
        }
    }

    // save additional views
    if (object->m_noAdditionalViews > 0)
    {
        for (size_t i=0;i <object->m_vAdditionalViews.size(); i++)
        {
            std::stringstream ss;
            ss<<object_label;ss<<"___";ss<<"additional_view_";ss<<i;

            mongo::BSONObjBuilder builder;
            builder.append("name", ss.str());
            mongo::BSONObj obj = builder.obj();

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*object->m_vAdditionalViews[i], msg_cloud);

            std::string id("");
            std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > results;
            id = (m_messageStoreData.updateNamed(ss.str(),msg_cloud,true));

            if (m_verbose)
            {
                ROS_INFO_STREAM("Object additional view cloud \""<<ss.str()<<"\" inserted with id "<<id);
            }
        }
    }

    return true;
}
