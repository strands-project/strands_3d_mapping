#include "strands_sweep_registration/PixelFunction.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
PixelFunction::PixelFunction(){
	r_mul = 1;
	g_mul = 1;
	b_mul = 1;
	d_mul = 1;
};
PixelFunction::~PixelFunction(){};

void PixelFunction::getValues(float * pixel){//Go through pixel and calculate new values
	pixel[0] *= r_mul;
	pixel[1] *= g_mul;
	pixel[2] *= b_mul;
	pixel[3] *= d_mul;
}

void PixelFunction::update(std::vector<float*> data, float bias, float random, float step){
	float meanrel = bias;
	int nr = data.size();
	for(int i = 0; i < nr; i++){
		meanrel += step*((data.at(i)[7]/data.at(i)[3])-1)+1;
	}

	d_mul *= meanrel / float(bias+nr);
	d_mul += random * (2.0f*float(rand()%10000)/float(10000) - 1.0f);
}

void PixelFunction::addOutput(	std::vector<char *> & pixeldata_char,	std::vector<int> & pixeldata_counter){
	int length = 4*sizeof(float);
	char * buffer = new char[length];
	float * buffer_float = (float *)buffer;
	buffer_float[0] = r_mul;
	buffer_float[1] = g_mul;
	buffer_float[2] = b_mul;
	buffer_float[3] = d_mul;
	pixeldata_char.push_back(buffer);
	pixeldata_counter.push_back(length);
}

void PixelFunction::load(char * buffer,int & pos){
	float * buffer_float = (float *)buffer;
	r_mul = buffer_float[0];
	g_mul = buffer_float[1];
	b_mul = buffer_float[2];
	d_mul = buffer_float[3];
	pos += 4*sizeof(float);
}
