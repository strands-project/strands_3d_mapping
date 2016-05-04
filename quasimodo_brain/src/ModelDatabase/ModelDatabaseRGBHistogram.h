#ifndef ModelDatabaseRGBHistogram_H
#define ModelDatabaseRGBHistogram_H

#include "ModelDatabase.h"


class ModelDatabaseRGBHistogram: public ModelDatabase{
	public:

	int res;
	std::vector< std::vector< double > > descriptors;

	
	virtual void add(reglib::Model * model);
	virtual bool remove(reglib::Model * model);
	virtual std::vector<reglib::Model *> search(reglib::Model * model, int number_of_matches);
		
	ModelDatabaseRGBHistogram(int res_);
	~ModelDatabaseRGBHistogram();
};



#endif // ModelDatabaseRGBHistogram_H
