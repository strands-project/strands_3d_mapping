#include "load_utilities.h"

typedef pcl::PointXYZRGB PointType;

int main(int argc, char** argv)
{

    std::string pathToSweepXml = "";

    boost::shared_ptr<pcl::PointCloud<PointType>> mergedCloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointType>(pathToSweepXml);

}
