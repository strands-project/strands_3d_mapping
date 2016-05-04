#ifndef ModelDatabase_H
#define ModelDatabase_H

#include <vector>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "model/Model.h"

class ModelDatabase{
	public:
	std::vector<reglib::Model * > models;

	//Add pointcloud to database, return index number in database, weight is the bias of the system to perfer this object when searching
	virtual void add(reglib::Model * model);
		
	// return true if successfull
	// return false if fail
	virtual bool remove(reglib::Model * model);
		
	//Find the number_of_matches closest matches in dabase to the pointcloud for index 
	virtual std::vector<reglib::Model * > search(reglib::Model * model, int number_of_matches);
		
	ModelDatabase();
	~ModelDatabase();
};

#include "ModelDatabaseBasic.h"
#include "ModelDatabaseRGBHistogram.h"
//#include "ModelDatabaseRetrieval.h"
#endif // ModelDatabase_H
