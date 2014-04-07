#include "SurfFeatureDescriptor128.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
SurfFeatureDescriptor128::SurfFeatureDescriptor128(float * feature_descriptor){
	descriptor = feature_descriptor;
	descriptor_length = 128;
	type = surf128;
	
};

SurfFeatureDescriptor128::SurfFeatureDescriptor128(){
	descriptor = new float[128];
	for(unsigned int i = 0; i < 128;i++){descriptor[i] = 0;}
	descriptor_length = 128;
	type = surf128;
	laplacian = -10;
};

SurfFeatureDescriptor128::SurfFeatureDescriptor128(float * feature_descriptor, int feature_laplacian){
	descriptor = feature_descriptor;
	descriptor_length = 128;
	type = surf128;
	laplacian = feature_laplacian;
};

SurfFeatureDescriptor128::~SurfFeatureDescriptor128(){
	delete[] descriptor;
	descriptor = 0;
	descriptor_length = 0;
	laplacian = -10;
};

SurfFeatureDescriptor128::SurfFeatureDescriptor128(string path){
	type = surf128;
	descriptor_length = 128;
	//cout<< "loading surf128: " << path << endl;
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
		laplacian = buffer_int[128];
	}else{cout<<"File not opened"<<endl;}
	//print();
};

void SurfFeatureDescriptor128::print(){
	printf("laplacian: %i\n",laplacian);
	cout<<"descriptor: ";
	for(int i = 0; i < 128; i++){cout << descriptor[i] << " ";}
	cout<<endl;
}

inline SurfFeatureDescriptor128 * SurfFeatureDescriptor128::clone(){
	SurfFeatureDescriptor128 * surf = new SurfFeatureDescriptor128();
	for(int i = 0; i < 128; i++){surf->descriptor[i] = descriptor[i];}
	surf->laplacian = laplacian;
	//surf->print();
	return surf;
}

inline void SurfFeatureDescriptor128::store(string path){
	//cout<<"storing surf\n";
	//for(int i = 0; i < 32; i++){orb->descriptor[i] = descriptor[i];}
	path += ".surf128";
	ifstream file (path.c_str());
	if (!file.is_open()){
		long size 				= sizeof(float)*(128)+sizeof(int);
		char * buffer_char 		= new char[size];
		float * buffer_float 	= (float *)buffer_char;
		int * buffer_int 		= (int *)buffer_char;
		for(int i = 0; i < 128; i++){buffer_float[i] = descriptor[i];}
		buffer_int[128] = laplacian;
		ofstream outfile (path.c_str(),ofstream::binary);
		outfile.write ((char *)buffer_char,size);
		outfile.close();
		delete buffer_char;
	}else{file.close();}
}

inline void SurfFeatureDescriptor128::update(vector<FeatureDescriptor * > * input){
	//printf("input->size() = %i\n",input->size());
	for(int j = 0; j < 128; j++){descriptor[j] = 0;} 
	for(unsigned int i = 0; i < input->size(); i++){
		SurfFeatureDescriptor128 * current = (SurfFeatureDescriptor128 *)input->at(i);
		for(int j = 0; j < 128; j++){descriptor[j] += current->descriptor[j];} 
	}
	for(int j = 0; j < 128; j++){descriptor[j] /= float(input->size());};
}

double SurfFeatureDescriptor128::distance(SurfFeatureDescriptor128 * other_descriptor)
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
		float tmp64 	= descriptor[64] - disc[64];
		float tmp65 	= descriptor[65] - disc[65];
		float tmp66 	= descriptor[66] - disc[66];
		float tmp67 	= descriptor[67] - disc[67];
		float tmp68 	= descriptor[68] - disc[68];
		float tmp69 	= descriptor[69] - disc[69];
		
		float tmp70 	= descriptor[70] - disc[70];
		float tmp71 	= descriptor[71] - disc[71];
		float tmp72 	= descriptor[72] - disc[72];
		float tmp73 	= descriptor[73] - disc[73];
		float tmp74 	= descriptor[74] - disc[74];
		float tmp75 	= descriptor[75] - disc[75];
		float tmp76 	= descriptor[76] - disc[76];
		float tmp77 	= descriptor[77] - disc[77];
		float tmp78 	= descriptor[78] - disc[78];
		float tmp79 	= descriptor[79] - disc[79];
		
		float tmp80 	= descriptor[80] - disc[80];
		float tmp81 	= descriptor[81] - disc[81];
		float tmp82 	= descriptor[82] - disc[82];
		float tmp83 	= descriptor[83] - disc[83];
		float tmp84 	= descriptor[84] - disc[84];
		float tmp85 	= descriptor[85] - disc[85];
		float tmp86 	= descriptor[86] - disc[86];
		float tmp87 	= descriptor[87] - disc[87];
		float tmp88 	= descriptor[88] - disc[88];
		float tmp89 	= descriptor[89] - disc[89];
			
		float tmp90 	= descriptor[90] - disc[90];
		float tmp91 	= descriptor[91] - disc[91];
		float tmp92 	= descriptor[92] - disc[92];
		float tmp93 	= descriptor[93] - disc[93];
		float tmp94 	= descriptor[94] - disc[94];
		float tmp95 	= descriptor[95] - disc[95];
		float tmp96 	= descriptor[96] - disc[96];
		float tmp97 	= descriptor[97] - disc[97];
		float tmp98 	= descriptor[98] - disc[98];
		float tmp99 	= descriptor[99] - disc[99];
			
		float tmp100 	= descriptor[100] - disc[100];
		float tmp101 	= descriptor[101] - disc[101];
		float tmp102 	= descriptor[102] - disc[102];
		float tmp103 	= descriptor[103] - disc[103];
		float tmp104 	= descriptor[104] - disc[104];
		float tmp105 	= descriptor[105] - disc[105];
		float tmp106 	= descriptor[106] - disc[106];
		float tmp107 	= descriptor[107] - disc[107];
		float tmp108 	= descriptor[108] - disc[108];
		float tmp109 	= descriptor[109] - disc[109];
			
		float tmp110 	= descriptor[110] - disc[110];
		float tmp111 	= descriptor[111] - disc[111];
		float tmp112 	= descriptor[112] - disc[112];
		float tmp113 	= descriptor[113] - disc[113];
		float tmp114 	= descriptor[114] - disc[114];
		float tmp115 	= descriptor[115] - disc[115];
		float tmp116 	= descriptor[116] - disc[116];
		float tmp117 	= descriptor[117] - disc[117];
		float tmp118 	= descriptor[118] - disc[118];
		float tmp119 	= descriptor[119] - disc[119];
			
		float tmp120 	= descriptor[120] - disc[120];
		float tmp121 	= descriptor[121] - disc[121];
		float tmp122 	= descriptor[122] - disc[122];
		float tmp123 	= descriptor[123] - disc[123];
		float tmp124 	= descriptor[124] - disc[124];
		float tmp125 	= descriptor[125] - disc[125];
		float tmp126 	= descriptor[126] - disc[126];
		float tmp127 	= descriptor[127] - disc[127];

		float dist = tmp0*tmp0 + tmp1*tmp1 + tmp2*tmp2 + tmp3*tmp3 + tmp4*tmp4 + tmp5*tmp5 + tmp6*tmp6 + tmp7*tmp7 + tmp8*tmp8 + tmp9*tmp9 + tmp10*tmp10 + tmp11*tmp11 + tmp12*tmp12 + tmp13*tmp13 + tmp14*tmp14 + tmp15*tmp15 + tmp16*tmp16 + tmp17*tmp17 + tmp18*tmp18 + tmp19*tmp19 + tmp20*tmp20 + tmp21*tmp21 + tmp22*tmp22 + tmp23*tmp23 + tmp24*tmp24 + tmp25*tmp25 + tmp26*tmp26 + tmp27*tmp27 + tmp28*tmp28 + tmp29*tmp29 + tmp30*tmp30 + tmp31*tmp31 + tmp32*tmp32 + tmp33*tmp33 + tmp34*tmp34 + tmp35*tmp35 + tmp36*tmp36 + tmp37*tmp37 + tmp38*tmp38 + tmp39*tmp39 + tmp40*tmp40 + tmp41*tmp41 + tmp42*tmp42 + tmp43*tmp43 + tmp44*tmp44 + tmp45*tmp45 + tmp46*tmp46 + tmp47*tmp47 + tmp48*tmp48 + tmp49*tmp49 + tmp50*tmp50 + tmp51*tmp51 + tmp52*tmp52 + tmp53*tmp53 + tmp54*tmp54 + tmp55*tmp55 + tmp56*tmp56 + tmp57*tmp57 + tmp58*tmp58 + tmp59*tmp59 + tmp60*tmp60 + tmp61*tmp61 + tmp62*tmp62 + tmp63*tmp63 + tmp64*tmp64 + tmp65*tmp65 + tmp66*tmp66 + tmp67*tmp67 + tmp68*tmp68 + tmp69*tmp69 + tmp70*tmp70 + tmp71*tmp71 + tmp72*tmp72 + tmp73*tmp73 + tmp74*tmp74 + tmp75*tmp75 + tmp76*tmp76 + tmp77*tmp77 + tmp78*tmp78 + tmp79*tmp79 + tmp80*tmp80 + tmp81*tmp81 + tmp82*tmp82 + tmp83*tmp83 + tmp84*tmp84 + tmp85*tmp85 + tmp86*tmp86 + tmp87*tmp87 + tmp88*tmp88 + tmp89*tmp89 + tmp90*tmp90 + tmp91*tmp91 + tmp92*tmp92 + tmp93*tmp93 + tmp94*tmp94 + tmp95*tmp95 + tmp96*tmp96 + tmp97*tmp97 + tmp98*tmp98 + tmp99*tmp99 + tmp100*tmp100 + tmp101*tmp101 + tmp102*tmp102 + tmp103*tmp103 + tmp104*tmp104 + tmp105*tmp105 + tmp106*tmp106 + tmp107*tmp107 + tmp108*tmp108 + tmp109*tmp109 + tmp110*tmp110 + tmp111*tmp111 + tmp112*tmp112 + tmp113*tmp113 + tmp114*tmp114 + tmp115*tmp115 + tmp116*tmp116 + tmp117*tmp117 + tmp118*tmp118 + tmp119*tmp119 + tmp120*tmp120 + tmp121*tmp121 + tmp122*tmp122 + tmp123*tmp123 + tmp124*tmp124 + tmp125*tmp125 + tmp126*tmp126 + tmp127*tmp127;
		return (dist);
	//}else{return 999999;}
}
