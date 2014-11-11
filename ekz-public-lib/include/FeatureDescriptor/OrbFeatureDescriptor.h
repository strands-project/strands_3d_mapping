#ifndef OrbFeatureDescriptor_H_
#define OrbFeatureDescriptor_H_
#include "FeatureDescriptor.h"
using namespace std;
class OrbFeatureDescriptor : public FeatureDescriptor
{
	public:
	int * descriptor;
	
	OrbFeatureDescriptor();
	OrbFeatureDescriptor(string path);
	OrbFeatureDescriptor(int * feature_descriptor);
	double distance(OrbFeatureDescriptor * other_descriptor);
	void print();
	void store(string path);
	OrbFeatureDescriptor * clone();
	void update(vector<FeatureDescriptor * > * input);
	~OrbFeatureDescriptor();
};
#endif
