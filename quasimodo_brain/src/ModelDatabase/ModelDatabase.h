#ifndef ModelDatabase_H
#define ModelDatabase_H

#include <vector>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ModelDatabase{
	public:
		
	//Add pointcloud to database, return index number in database, weight is the bias of the system to perfer this object when searching
	virtual int add(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, double weight = 1);
		
	// return true if successfull
	// return false if fail
	virtual bool remove(int index);
		
	//Find the number_of_matches closest matches in dabase to the pointcloud for index 
	virtual std::vector<int> search(int index, int number_of_matches);
		
	ModelDatabase();
	~ModelDatabase();
};

#include "ModelDatabaseBasic.h"
#endif // ModelDatabase_H
