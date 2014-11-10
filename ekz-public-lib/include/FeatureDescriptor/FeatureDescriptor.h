#ifndef FeatureDescriptor_H_
#define FeatureDescriptor_H_
//#include <pcl/point_types.h>
#include <vector>
#include <string>

#include <stdio.h>
using namespace std;

enum DescriptorType { unitiated, base, surf128, surf64, rgb, orb, FPFH, IntegerHistogram, FloatHistogram };

class FeatureDescriptor {
	public:
		DescriptorType type;
		virtual double distance(FeatureDescriptor * other_descriptor);
		FeatureDescriptor();
		virtual ~FeatureDescriptor();
		virtual void print();
		virtual void normalize();
		virtual FeatureDescriptor * add(FeatureDescriptor * feature);
		virtual FeatureDescriptor * mul(float val);
		virtual FeatureDescriptor * clone();
		virtual void update(vector<FeatureDescriptor * > * input);
		virtual void store(string path);
};
#include "OrbFeatureDescriptor.h"
#include "SurfFeatureDescriptor64.h"
#include "SurfFeatureDescriptor128.h"
#endif
