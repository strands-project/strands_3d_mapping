#ifndef KeyPointSet_H_
#define KeyPointSet_H_
#include "KeyPoint.h"
#include <vector>

using namespace std;

class KeyPointSet {
	public:
		double stabilety_threshold;
		vector<KeyPoint * > valid_key_points;
		vector<KeyPoint * > invalid_key_points;
 	
		KeyPointSet();
		~KeyPointSet();

		void sortKeyPoints();
};


#endif
