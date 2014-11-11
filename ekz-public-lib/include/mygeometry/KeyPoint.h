#ifndef KeyPoint_H_
#define KeyPoint_H_
#include "FeatureDescriptor.h"
#include <Eigen/Core>
#include <algorithm>
#include <vector>
#include "Point.h"
#include "KeyPointChain.h"


class KeyPointChain;
class KeyPoint {
	public:
		int index_number;
		FeatureDescriptor * descriptor;
		Point * point;
		bool valid;
		int r;
		int g;
		int b;

		float w;
		float h;

		float stabilety;
		vector< pair <int , float > > cluster_distance_pairs;
		KeyPointChain * chain;
		int frame_id;
		KeyPoint();
		~KeyPoint();
		void print();
		void sortDistances();
};
#endif
