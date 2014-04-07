#ifndef KeyPointChain_H_
#define KeyPointChain_H_
#include "KeyPoint.h"
#include <vector>

using namespace std;

class KeyPoint;
class KeyPointChain {
	public:
		vector<KeyPoint * > key_points;
 		int id;

		KeyPointChain(KeyPoint * kp);
		~KeyPointChain();

		void merge(KeyPointChain * chain);
};


#endif
