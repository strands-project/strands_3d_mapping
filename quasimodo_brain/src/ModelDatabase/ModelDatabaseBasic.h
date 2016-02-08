#ifndef ModelDatabaseBasic_H
#define ModelDatabaseBasic_H

#include "ModelDatabase.h"


class ModelDatabaseBasic: public ModelDatabase{
	public:
	
	virtual void add(reglib::Model * model);
	virtual bool remove(reglib::Model * model);
	virtual std::vector<reglib::Model *> search(reglib::Model * model, int number_of_matches);
		
	ModelDatabaseBasic();
	~ModelDatabaseBasic();
};



#endif // ModelDatabaseBasic_H
