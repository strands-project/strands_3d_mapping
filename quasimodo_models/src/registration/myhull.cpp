#include "myhull.h"
#include <pcl/surface/convex_hull.h>
//#include "ICP.h"
namespace reglib
{
myhull::myhull(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
/*
    cloud->points.resize(all_starts[ax].size());
    for(unsigned int i = 0; i < all_starts[ax].size(); i++){
        cloud->points[i].x = all_starts[ax].at(i)(0);
        cloud->points[i].y = all_starts[ax].at(i)(1);
        cloud->points[i].z = all_starts[ax].at(i)(2);
    }
*/
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud);
    chull.reconstruct (*cloud_hull);

}
myhull::~myhull(){}
}
