#include "ModelMask.h"

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
	printf("ModelMask: %i\n",id);

	using namespace cv;
/*
	int erosion_size = 1;
	int erosion_type = MORPH_RECT;
	Mat element = getStructuringElement( erosion_type, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );

	cv::Mat erosion_mask;
	erode( mask, erosion_mask, element );


	unsigned char * erosion_maskdata = (unsigned char *)erosion_mask.data;
	for(unsigned int w = 1; w < 639; w++){
		for(unsigned int h = 1; h < 479; h++){
			if(erosion_maskdata[h*640+w] != 0){
				testw.push_back(w);
				testh.push_back(h);
			}
		}
	}

*/
	unsigned char * maskdata = (unsigned char *)mask.data;

	width = 640;
	height = 480;
	maskvec = new bool[width*height];
	for(unsigned int i = 0; i < width*height; i++){maskvec[i] = maskdata[i] != 0;}



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
	//cv::imshow("ModelMask",	mask);
	//cv::imshow("erosion_mask",	erosion_mask);
	//cv::waitKey(0);
	//printf("ModelMask\n");
}

cv::Mat ModelMask::getMask(){
//	printf("cv::Mat ModelMask::getMask()\n");
//	printf("%i %i\n",width,height);

	cv::Mat fullmask;
	fullmask.create(height,width,CV_8UC1);

	unsigned char * maskdata = (unsigned char *)fullmask.data;
	for(unsigned int j = 0; j < width*height; j++){
		maskdata[j] = 255*maskvec[j];
	}
	return fullmask;
}
//cv::Mat ModelMask::getMask(){

//}

}

