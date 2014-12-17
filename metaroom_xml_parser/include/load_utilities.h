#include "simple_summary_parser.h"
#include "simple_xml_parser.h"

namespace semantic_map_load_utilties
{


    template <class PointType>
    boost::shared_ptr<pcl::PointCloud<PointType>> loadMergedCloudFromSingleSweep(std::string sweepXmlPath, bool verbose=false)
    {
        boost::shared_ptr<pcl::PointCloud<PointType>> toRet;

        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, verbose);


        return sweep.completeRoomCloud;
    }


}
