#include "ModelMask.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"

namespace reglib
{

ModelMask::ModelMask(cv::Mat mask_){
	mask = mask_;
	sweepid = -1;

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

}

