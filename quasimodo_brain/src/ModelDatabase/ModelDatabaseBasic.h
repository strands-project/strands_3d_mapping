#ifndef ModelDatabaseBasic_H
#define ModelDatabaseBasic_H

#include "ModelDatabase.h"


class ModelDatabaseBasic: public ModelDatabase{
	public:
	virtual int add(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, double weight = 1);
		
	// return true if successfull
	// return false if fail
	virtual bool remove(int index);
		
	//Find the number_of_matches closest matches in dabase to the pointcloud for index 
	virtual std::vector<int> search(int index, int number_of_matches);
		
	ModelDatabaseBasic();
	~ModelDatabaseBasic();
};



#endif // ModelDatabaseBasic_H
