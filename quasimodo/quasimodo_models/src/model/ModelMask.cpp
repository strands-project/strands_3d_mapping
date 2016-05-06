#include "model/ModelMask.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"

namespace reglib
{

int ModelMask_id = 0;

ModelMask::ModelMask(cv::Mat mask_){
	mask = mask_;
	sweepid = -1;

	id = ModelMask_id++;
	using namespace cv;

	unsigned char * maskdata = (unsigned char *)mask.data;

	width = 640;
	height = 480;
	maskvec = new bool[width*height];
    for(int i = 0; i < width*height; i++){maskvec[i] = maskdata[i] != 0;}



	for(unsigned int w = 1; w < 639; w++){
		for(unsigned int h = 1; h < 479; h++){
			if(maskdata[h*640+w] != 0){
				testw.push_back(w);
				testh.push_back(h);
			}
		}
	}

	for(unsigned int w = 1; w < 639; w++){
		for(unsigned int h = 1; h < 479; h++){
			if(maskdata[h*640+w] != 0){
				testw.push_back(w);
				testh.push_back(h);
			}
		}
	}

	for(unsigned int i = 0; i < testw.size(); i++){
		int ind = rand() % testw.size();
		int tw = testw[i];
		int th = testh[i];
		testw[i] = testw[ind];
		testh[i] = testh[ind];
		testw[ind] = tw;
		testh[ind] = th;
	}
}

cv::Mat ModelMask::getMask(){
	cv::Mat fullmask;
	fullmask.create(height,width,CV_8UC1);
	unsigned char * maskdata = (unsigned char *)fullmask.data;
    for(int j = 0; j < width*height; j++){maskdata[j] = 255*maskvec[j];}
	return fullmask;
}

ModelMask::~ModelMask(){delete[] maskvec;}


}

