#include "ModelDatabaseBasic.h"

ModelDatabaseBasic::ModelDatabaseBasic(){}
ModelDatabaseBasic::~ModelDatabaseBasic(){}

//Add pointcloud to database, return index number in database, weight is the bias of the system to perfer this object when searching
int ModelDatabaseBasic::add(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, double weight){
	return true;
}
		
// return true if successfull
// return false if fail
bool ModelDatabaseBasic::remove(int index){
	return false;
}
		
//Find the number_of_matches closest matches in dabase to the pointcloud for index 
std::vector<int> ModelDatabaseBasic::search(int index, int number_of_matches){
	return std::vector<int>();
}
