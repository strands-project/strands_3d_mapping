#include "vlad/common.h"

#include <pcl/io/pcd_io.h>

using namespace std;

pair<vector<CloudT::Ptr>, vector<boost::filesystem::path> > load_vlad_clouds(const vector<pair<float, string> >& matches)
{
    pair<vector<CloudT::Ptr>, vector<boost::filesystem::path> > results;
    for (auto tup : matches) {
        results.first.push_back(CloudT::Ptr(new CloudT));
        results.second.push_back(boost::filesystem::path(tup.second).parent_path().parent_path() / "room.xml");
        pcl::io::loadPCDFile(tup.second, *results.first.back());
    }
    return results;
}
