#include "Line2D.h"
#include <iostream>
#include <vector>
#include <algorithm>
//#include "cv.h"
//#include "highgui.h"
//#include <opencv.hpp>
#include <Eigen/LU>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;
int Line2D_counter = 0;

Line2D::Line2D(vector<float> * p_width, vector<float> * p_height, vector<float> * p_weight){
	id = Line2D_counter++;
	float wmean = 0;
	float hmean = 0;
	float weight_sum = 0;
	unsigned int size = p_width->size();

	float w[size];
	float h[size];

	for(unsigned int i = 0; i < size; i++){
		float weight = p_weight->at(i);
		wmean+=p_width->at(i)*weight;
		hmean+=p_height->at(i)*weight;
		weight_sum += weight;

		w[i] = p_width->at(i);
		h[i] = p_height->at(i);
	}
	wmean/=weight_sum;
	hmean/=weight_sum;

	for(unsigned int i = 0; i < size; i++){
		w[i] -= wmean;
		h[i] -= hmean;
	}

	vector<float *> data;
	data.push_back(w);
	data.push_back(h);
	
	MatrixXf covMat(data.size(),data.size());

	for(unsigned int i = 0; i < data.size(); i++){
		for(unsigned int j = i; j < data.size(); j++){
			float * col1 	= data.at(i);
			float * col2 	= data.at(j);
			float sum = 0;
			for(int k = 0; k < size; k++){
				sum+=col1[k]*col2[k];
			}
			covMat(i,j)=sum/float(size-1);
			covMat(j,i)=covMat(i,j);
		}
	}		

	JacobiSVD<MatrixXf> svd(covMat, ComputeThinU | ComputeThinV);

	VectorXf S = svd.singularValues();

	weight = 0.001*(S(0)+S(1))/(S(1)*S(1));
	if(isnan(weight)){weight = -1;}

	MatrixXf U = svd.matrixU();
	
	point_w = wmean;
	point_h = hmean;

	normal_w 			= U(0,0);
	normal_h 			= U(1,0);

	normal2_w 			= U(0,1);
	normal2_h 			= U(1,1);
}
Line2D::~Line2D(){}

float Line2D::distance(float w, float h){
	return normal2_w*(point_w-w) + normal2_h*(point_h-h);
}

