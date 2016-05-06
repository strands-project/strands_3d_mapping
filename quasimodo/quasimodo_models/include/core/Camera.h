#ifndef reglibCamera_H
#define reglibCamera_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>


#include <iostream>
#include <fstream>

namespace reglib
{
	class Camera{
		public:
		int id;

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

		void save(std::string path = "");
		void print();
		static Camera * load(std::string path);
	};

}

#endif // reglibCamera_H
