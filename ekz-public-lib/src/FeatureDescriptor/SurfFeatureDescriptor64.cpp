#include "SurfFeatureDescriptor64.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
SurfFeatureDescriptor64::SurfFeatureDescriptor64(float * feature_descriptor){
	descriptor = feature_descriptor;
	descriptor_length = 64;
	type = surf64;
	
};

SurfFeatureDescriptor64::SurfFeatureDescriptor64(){
	descriptor = new float[64];
	for(unsigned int i = 0; i < 64;i++){descriptor[i] = 0;}
	descriptor_length = 64;
	type = surf64;
	laplacian = -10;
};

SurfFeatureDescriptor64::SurfFeatureDescriptor64(float * feature_descriptor, int feature_laplacian){
	descriptor = feature_descriptor;
	descriptor_length = 64;
	type = surf64;
	laplacian = feature_laplacian;
};

SurfFeatureDescriptor64::~SurfFeatureDescriptor64(){
	delete[] descriptor;
	descriptor = 0;
	descriptor_length = 0;
	laplacian = -10;
};

SurfFeatureDescriptor64::SurfFeatureDescriptor64(string path){
	type = surf64;
	descriptor_length = 64;
	//cout<< "loading surf64: " << path << endl;
	ifstream file (path.c_str());
	if (file.is_open()){
		file.seekg(0,ifstream::end);
		long size=file.tellg();
		char * buffer_char 		= new char [size];
		int * buffer_int 		= (int *) buffer_char;
		file.seekg(0);
		file.read (buffer_char,size);
		file.close();
		descriptor = (float *) buffer_char;
		laplacian = buffer_int[64];
	}else{cout<<"File not opened"<<endl;}
	//print();
};

void SurfFeatureDescriptor64::print(){
	printf("laplacian: %i\n",laplacian);
	cout<<"descriptor: ";
	for(int i = 0; i < 64; i++){cout << descriptor[i] << " ";}
	cout<<endl;
}

inline SurfFeatureDescriptor64 * SurfFeatureDescriptor64::clone(){
	SurfFeatureDescriptor64 * surf = new SurfFeatureDescriptor64();
	for(int i = 0; i < 64; i++){surf->descriptor[i] = descriptor[i];}
	surf->laplacian = laplacian;
	//surf->print();
	return surf;
}

inline void SurfFeatureDescriptor64::store(string path){
	//cout<<"storing surf\n";
	//for(int i = 0; i < 32; i++){orb->descriptor[i] = descriptor[i];}
	path += ".surf64";
	ifstream file (path.c_str());
	if (!file.is_open()){
		long size 				= sizeof(float)*(64)+sizeof(int);
		char * buffer_char 		= new char[size];
		float * buffer_float 	= (float *)buffer_char;
		int * buffer_int 		= (int *)buffer_char;
		for(int i = 0; i < 64; i++){buffer_float[i] = descriptor[i];}
		buffer_int[64] = laplacian;
		ofstream outfile (path.c_str(),ofstream::binary);
		outfile.write ((char *)buffer_char,size);
		outfile.close();
		delete buffer_char;
	}else{file.close();}
}

inline void SurfFeatureDescriptor64::update(vector<FeatureDescriptor * > * input){
	//printf("input->size() = %i\n",input->size());
	for(int j = 0; j < 64; j++){descriptor[j] = 0;} 
	for(unsigned int i = 0; i < input->size(); i++){
		SurfFeatureDescriptor64 * current = (SurfFeatureDescriptor64 *)input->at(i);
		for(int j = 0; j < 64; j++){descriptor[j] += current->descriptor[j];} 
	}
	for(int j = 0; j < 64; j++){descriptor[j] /= float(input->size());};
}

double SurfFeatureDescriptor64::distance(SurfFeatureDescriptor64 * other_descriptor)
{
	//if(laplacian == other_descriptor->laplacian){
		float * disc = other_descriptor->descriptor;
		float tmp0 		= descriptor[0] - disc[0];
		float tmp1 		= descriptor[1] - disc[1];
		float tmp2 		= descriptor[2] - disc[2];
		float tmp3 		= descriptor[3] - disc[3];
		float tmp4 		= descriptor[4] - disc[4];
		float tmp5 		= descriptor[5] - disc[5];
		float tmp6 		= descriptor[6] - disc[6];
		float tmp7 		= descriptor[7] - disc[7];
		float tmp8 		= descriptor[8] - disc[8];
		float tmp9 		= descriptor[9] - disc[9];
		
		float tmp10 	= descriptor[10] - disc[10];
		float tmp11 	= descriptor[11] - disc[11];
		float tmp12 	= descriptor[12] - disc[12];
		float tmp13 	= descriptor[13] - disc[13];
		float tmp14 	= descriptor[14] - disc[14];
		float tmp15 	= descriptor[15] - disc[15];
		float tmp16 	= descriptor[16] - disc[16];
		float tmp17 	= descriptor[17] - disc[17];
		float tmp18 	= descriptor[18] - disc[18];
		float tmp19 	= descriptor[19] - disc[19];
			
		float tmp20 	= descriptor[20] - disc[20];
		float tmp21 	= descriptor[21] - disc[21];
		float tmp22 	= descriptor[22] - disc[22];
		float tmp23 	= descriptor[23] - disc[23];
		float tmp24 	= descriptor[24] - disc[24];
		float tmp25 	= descriptor[25] - disc[25];
		float tmp26 	= descriptor[26] - disc[26];
		float tmp27 	= descriptor[27] - disc[27];
		float tmp28 	= descriptor[28] - disc[28];
		float tmp29 	= descriptor[29] - disc[29];
			
		float tmp30 	= descriptor[30] - disc[30];
		float tmp31 	= descriptor[31] - disc[31];
		float tmp32 	= descriptor[32] - disc[32];
		float tmp33 	= descriptor[33] - disc[33];
		float tmp34 	= descriptor[34] - disc[34];
		float tmp35 	= descriptor[35] - disc[35];
		float tmp36 	= descriptor[36] - disc[36];
		float tmp37 	= descriptor[37] - disc[37];
		float tmp38 	= descriptor[38] - disc[38];
		float tmp39 	= descriptor[39] - disc[39];
		
		float tmp40 	= descriptor[40] - disc[40];
		float tmp41 	= descriptor[41] - disc[41];
		float tmp42 	= descriptor[42] - disc[42];
		float tmp43 	= descriptor[43] - disc[43];
		float tmp44 	= descriptor[44] - disc[44];
		float tmp45 	= descriptor[45] - disc[45];
		float tmp46 	= descriptor[46] - disc[46];
		float tmp47 	= descriptor[47] - disc[47];
		float tmp48 	= descriptor[48] - disc[48];
		float tmp49 	= descriptor[49] - disc[49];
		
		float tmp50 	= descriptor[50] - disc[50];
		float tmp51 	= descriptor[51] - disc[51];
		float tmp52 	= descriptor[52] - disc[52];
		float tmp53 	= descriptor[53] - disc[53];
		float tmp54 	= descriptor[54] - disc[54];
		float tmp55 	= descriptor[55] - disc[55];
		float tmp56 	= descriptor[56] - disc[56];
		float tmp57 	= descriptor[57] - disc[57];
		float tmp58 	= descriptor[58] - disc[58];
		float tmp59 	= descriptor[59] - disc[59];
		
		float tmp60 	= descriptor[60] - disc[60];
		float tmp61 	= descriptor[61] - disc[61];
		float tmp62 	= descriptor[62] - disc[62];
		float tmp63 	= descriptor[63] - disc[63];
		float dist 		= tmp0*tmp0 + tmp1*tmp1 + tmp2*tmp2 + tmp3*tmp3 + tmp4*tmp4 + tmp5*tmp5 + tmp6*tmp6 + tmp7*tmp7 + tmp8*tmp8 + tmp9*tmp9 + tmp10*tmp10 + tmp11*tmp11 + tmp12*tmp12 + tmp13*tmp13 + tmp14*tmp14 + tmp15*tmp15 + tmp16*tmp16 + tmp17*tmp17 + tmp18*tmp18 + tmp19*tmp19 + tmp20*tmp20 + tmp21*tmp21 + tmp22*tmp22 + tmp23*tmp23 + tmp24*tmp24 + tmp25*tmp25 + tmp26*tmp26 + tmp27*tmp27 + tmp28*tmp28 + tmp29*tmp29 + tmp30*tmp30 + tmp31*tmp31 + tmp32*tmp32 + tmp33*tmp33 + tmp34*tmp34 + tmp35*tmp35 + tmp36*tmp36 + tmp37*tmp37 + tmp38*tmp38 + tmp39*tmp39 + tmp40*tmp40 + tmp41*tmp41 + tmp42*tmp42 + tmp43*tmp43 + tmp44*tmp44 + tmp45*tmp45 + tmp46*tmp46 + tmp47*tmp47 + tmp48*tmp48 + tmp49*tmp49 + tmp50*tmp50 + tmp51*tmp51 + tmp52*tmp52 + tmp53*tmp53 + tmp54*tmp54 + tmp55*tmp55 + tmp56*tmp56 + tmp57*tmp57 + tmp58*tmp58 + tmp59*tmp59 + tmp60*tmp60 + tmp61*tmp61 + tmp62*tmp62 + tmp63*tmp63;
		return (dist);
	//}else{return 999999;}
}
