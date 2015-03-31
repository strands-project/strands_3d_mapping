#ifndef Sweep_H_
#define Sweep_H_
#include <vector>
#include <string>
#include "Frame.h"

class Sweep
{
	public:
	std::string idtag;
	std::string xmlpath;

	int width;
	int height;
	Frame *** 			frames;
	Eigen::Matrix4f ** 	poses;
	
	Sweep(int width_, int height_, std::vector<Frame *> framesvec);
	Sweep();
	virtual ~Sweep();
	
    Eigen::Matrix4f align(Sweep * sweep, float threshold = 0.02, int ransac_iter = 20000, int nr_points = 3);
	std::vector<Eigen::Matrix4f> getPoseVector();
};
#endif
