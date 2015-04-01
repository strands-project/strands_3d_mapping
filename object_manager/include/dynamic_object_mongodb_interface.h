#ifndef __DYNAMIC_OBJECT_MONGODB_INTERFACE
#define __DYNAMIC_OBJECT_MONGODB_INTERFACE


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

#include "dynamic_object.h"

class DynamicObjectMongodbInterface {
public:


    DynamicObjectMongodbInterface(ros::NodeHandle nh, bool verbose = false)  : m_messageStoreData(nh,"data","metric_maps"),
        m_messageStoreSummary(nh,"summary","metric_maps"),m_verbose(verbose)
    {
        m_NodeHandle = nh;
    }

    ~DynamicObjectMongodbInterface()
    {

    }

    bool logToDB(DynamicObject::Ptr object, std::string objectXMLPath);


private:
    ros::NodeHandle                                                            m_NodeHandle;
    mongodb_store::MessageStoreProxy                                           m_messageStoreData;
    mongodb_store::MessageStoreProxy                                           m_messageStoreSummary;
    bool                                                                       m_verbose;

};




#endif // __DYNAMIC_OBJECT_MONGODB_INTERFACE
