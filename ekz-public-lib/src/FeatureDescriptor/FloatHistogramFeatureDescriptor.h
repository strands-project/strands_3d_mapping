#ifndef FloatHistogramFeatureDescriptor_H_
#define FloatHistogramFeatureDescriptor_H_
#include "FeatureDescriptor.h"
using namespace std;
class FloatHistogramFeatureDescriptor : public FeatureDescriptor
{
	public:
	float * descriptor;
	int length;
	
	FloatHistogramFeatureDescriptor();
	FloatHistogramFeatureDescriptor(string path);
	FloatHistogramFeatureDescriptor(float * feature_descriptor, int feature_length);
	double distance(FeatureDescriptor * other_descriptor);
	void print();
	void store(string path);
	FloatHistogramFeatureDescriptor * clone();
	void update(vector<FeatureDescriptor * > * input);
	~FloatHistogramFeatureDescriptor();
};
#endif
