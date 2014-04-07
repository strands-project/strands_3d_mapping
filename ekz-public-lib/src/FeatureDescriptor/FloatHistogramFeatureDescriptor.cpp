#include "FloatHistogramFeatureDescriptor.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
FloatHistogramFeatureDescriptor::FloatHistogramFeatureDescriptor(float * desc, int feature_length){
	descriptor = desc;
	type = FloatHistogram;
	length = feature_length;
};

FloatHistogramFeatureDescriptor::FloatHistogramFeatureDescriptor(){
	type = IntegerHistogram;
	length = 0;
	descriptor = 0;
};

FloatHistogramFeatureDescriptor::FloatHistogramFeatureDescriptor(string path){
	type = IntegerHistogram;

	cout<< "loading FH: " << path << endl;
	ifstream file (path.c_str());
	if (file.is_open()){
		file.seekg(0,ifstream::end);
		long size=file.tellg();
		char * buffer_char 		= new char [size];
		//int * buffer_int 		= (int *) buffer_char;
		file.seekg(0);
		file.read (buffer_char,size);
		file.close();
		descriptor = (float *) buffer_char;
	}else{cout<<"File not opened"<<endl;}
	print();
};

FloatHistogramFeatureDescriptor::~FloatHistogramFeatureDescriptor(){
	delete[] descriptor;
	descriptor = 0;
};

void FloatHistogramFeatureDescriptor::print(){
	cout<<"FH: ";
	for(int i = 0; i < length; i++){if(descriptor[i] > 0){cout << descriptor[i] << " ";}}
	cout<<endl;
}

inline FloatHistogramFeatureDescriptor * FloatHistogramFeatureDescriptor::clone(){
	FloatHistogramFeatureDescriptor * orb = new FloatHistogramFeatureDescriptor();
	orb->descriptor = new float[length];
	for(int i = 0; i < length; i++){orb->descriptor[i] = descriptor[i];}
	return orb;
}

inline void FloatHistogramFeatureDescriptor::store(string path){
	cout<<"storing FloatHistogram\n";
	ifstream file (path.c_str());
	if (!file.is_open()){
		long size 				= sizeof(int)*(length);
		ofstream outfile (path.c_str(),ofstream::binary);
		outfile.write ((char *)descriptor,size);
		outfile.close();
	}else{file.close();}
}

inline void FloatHistogramFeatureDescriptor::update(vector<FeatureDescriptor * > * input){
	for(int j = 0; j < length; j++){descriptor[j] = 0;} 
	for(unsigned int i = 0; i < input->size(); i++){
		FloatHistogramFeatureDescriptor * current = (FloatHistogramFeatureDescriptor *)input->at(i);
		for(int j = 0; j < length; j++){descriptor[j] += current->descriptor[j];} 
	}
	for(int j = 0; j < length; j++){descriptor[j] /= input->size();};
}

double FloatHistogramFeatureDescriptor::distance(FeatureDescriptor * other_descriptor)
{
	if(other_descriptor->type == type){
		float * disc 	= ((FloatHistogramFeatureDescriptor*)other_descriptor)->descriptor;
		double sum = 0;
		for(int i = 0; i < length; i++){sum += (disc[i]-descriptor[i])*(disc[i]-descriptor[i]);}
		return sum;
	}else{
		return -1;
	}
}
