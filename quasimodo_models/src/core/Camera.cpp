#include "Camera.h"

namespace reglib
{

Camera::Camera(){
	width = 640;
	height = 480;

	fx = 540.0;
	fy = 540.0;
	cx = float(width-1)/2;
	cy = float(height-1)/2;

	idepth_scale = 0.001;

	bias = 500;

	pixel_weights = new double[width*height];
	pixel_sums = new double[width*height];
	for(unsigned int i = 0; i < width*height; i++){
		pixel_weights[i]	= bias;
		pixel_sums[i]		= bias;
	}

}

Camera::~Camera(){
	delete[] pixel_weights;
	delete[] pixel_sums;
}

}
