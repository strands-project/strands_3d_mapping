#ifndef reglibCamera_H
#define reglibCamera_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>

namespace reglib
{
	class Camera{
		public:
		double idepth_scale;

		double fx;
		double fy;
		double cx;
		double cy;

		double bias;

		double * pixel_weights;
		double * pixel_sums;
		unsigned int width;
		unsigned int height;

		Camera();
		~Camera();
	};

}

#endif // reglibCamera_H
