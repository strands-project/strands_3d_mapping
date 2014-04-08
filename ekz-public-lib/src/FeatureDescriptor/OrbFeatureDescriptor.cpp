#include "OrbFeatureDescriptor.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
OrbFeatureDescriptor::OrbFeatureDescriptor(int * desc){
	descriptor = desc;
	type = orb;
	
};

OrbFeatureDescriptor::OrbFeatureDescriptor(){
	type = orb;
};

OrbFeatureDescriptor::OrbFeatureDescriptor(string path){
	type = orb;
	cout<< "loading orb: " << path << endl;
	ifstream file (path.c_str());
	if (file.is_open()){
		file.seekg(0,ifstream::end);
		long size=file.tellg();
		char * buffer_char 		= new char [size];
		//int * buffer_int 		= (int *) buffer_char;
		file.seekg(0);
		file.read (buffer_char,size);
		file.close();
		descriptor = (int *) buffer_char;
	}else{cout<<"File not opened"<<endl;}
	print();
};

OrbFeatureDescriptor::~OrbFeatureDescriptor(){
	delete[] descriptor;
	descriptor = 0;
};

void OrbFeatureDescriptor::print(){
	cout<<"orb: ";
	for(int i = 0; i < 32; i++){cout << descriptor[i] << " ";}
	cout<<endl;
}

inline OrbFeatureDescriptor * OrbFeatureDescriptor::clone(){
	OrbFeatureDescriptor * orb = new OrbFeatureDescriptor();
	orb->descriptor = new int[32];
	for(int i = 0; i < 32; i++){orb->descriptor[i] = descriptor[i];}
	return orb;
}

inline void OrbFeatureDescriptor::store(string path){
	cout<<"storing orb\n";
	//for(int i = 0; i < 32; i++){orb->descriptor[i] = descriptor[i];}
	path += ".orb";
	ifstream file (path.c_str());
	if (!file.is_open()){
		long size 				= sizeof(int)*(32);
		//char * buffer_char 		= new char[size];
		//int * buffer_int 		= (int *)buffer_char;
		//for(int i = 0; i < 32; i++){buffer_int[i] = descriptor[i];}
		ofstream outfile (path.c_str(),ofstream::binary);
		outfile.write ((char *)descriptor,size);
		outfile.close();
		//delete buffer_char;
	}else{file.close();}
}

inline void OrbFeatureDescriptor::update(vector<FeatureDescriptor * > * input){
	//printf("input->size() = %i\n",input->size());
	for(int j = 0; j < 32; j++){descriptor[j] = 0;} 
	for(unsigned int i = 0; i < input->size(); i++){
		OrbFeatureDescriptor * current = (OrbFeatureDescriptor *)input->at(i);
		for(int j = 0; j < 32; j++){descriptor[j] += current->descriptor[j];} 
	}
	for(int j = 0; j < 32; j++){descriptor[j] /= input->size();};
}

double OrbFeatureDescriptor::distance(OrbFeatureDescriptor * other_descriptor)
{
	//printf("OrbFeatureDescriptor::distance\n");
	int * disc 	= other_descriptor->descriptor;
	int tmp0 	= descriptor[0] - disc[0];
	int tmp1 	= descriptor[1] - disc[1];
	int tmp2 	= descriptor[2] - disc[2];
	int tmp3 	= descriptor[3] - disc[3];
	int tmp4 	= descriptor[4] - disc[4];
	int tmp5 	= descriptor[5] - disc[5];
	int tmp6 	= descriptor[6] - disc[6];
	int tmp7 	= descriptor[7] - disc[7];
	int tmp8 	= descriptor[8] - disc[8];
	int tmp9 	= descriptor[9] - disc[9];
		
	int tmp10 	= descriptor[10] - disc[10];
	int tmp11 	= descriptor[11] - disc[11];
	int tmp12 	= descriptor[12] - disc[12];
	int tmp13 	= descriptor[13] - disc[13];
	int tmp14 	= descriptor[14] - disc[14];
	int tmp15 	= descriptor[15] - disc[15];
	int tmp16 	= descriptor[16] - disc[16];
	int tmp17 	= descriptor[17] - disc[17];
	int tmp18 	= descriptor[18] - disc[18];
	int tmp19 	= descriptor[19] - disc[19];
			
	int tmp20 	= descriptor[20] - disc[20];
	int tmp21 	= descriptor[21] - disc[21];
	int tmp22 	= descriptor[22] - disc[22];
	int tmp23 	= descriptor[23] - disc[23];
	int tmp24 	= descriptor[24] - disc[24];
	int tmp25 	= descriptor[25] - disc[25];
	int tmp26 	= descriptor[26] - disc[26];
	int tmp27 	= descriptor[27] - disc[27];
	int tmp28 	= descriptor[28] - disc[28];
	int tmp29 	= descriptor[29] - disc[29];
			
	int tmp30 	= descriptor[30] - disc[30];
	int tmp31 	= descriptor[31] - disc[31];
	
	return 0.0003*sqrt(double(tmp0*tmp0 + tmp1*tmp1 + tmp2*tmp2 + tmp3*tmp3 + tmp4*tmp4 + tmp5*tmp5 + tmp6*tmp6 + tmp7*tmp7 + tmp8*tmp8 + tmp9*tmp9 + tmp10*tmp10 + tmp11*tmp11 + tmp12*tmp12 + tmp13*tmp13 + tmp14*tmp14 + tmp15*tmp15 + tmp16*tmp16 + tmp17*tmp17 + tmp18*tmp18 + tmp19*tmp19 + tmp20*tmp20 + tmp21*tmp21 + tmp22*tmp22 + tmp23*tmp23 + tmp24*tmp24 + tmp25*tmp25 + tmp26*tmp26 + tmp27*tmp27 + tmp28*tmp28 + tmp29*tmp29 + tmp30*tmp30 + tmp31*tmp31));
}
