#include "mongodb_interface.h"

MongodbInterface::MongodbInterface(ros::NodeHandle nh) : m_messageStoreData(nh,"data","metric_maps"), m_messageStoreSummary(nh,"summary","metric_maps")
{
    m_NodeHandle = nh;
}

MongodbInterface::~MongodbInterface()
{

}



