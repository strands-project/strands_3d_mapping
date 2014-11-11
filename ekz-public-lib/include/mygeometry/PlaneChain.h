#ifndef PlaneChain_H_
#define PlaneChain_H_
#include "Plane.h"
#include <vector>

using namespace std;

class Plane;
class PlaneChain {
	public:
		int id;
		vector<Plane * > planes;
 		int r;
		int g;
		int b;
		PlaneChain(Plane * kp);
		~PlaneChain();

		void merge(PlaneChain * chain);
};


#endif
