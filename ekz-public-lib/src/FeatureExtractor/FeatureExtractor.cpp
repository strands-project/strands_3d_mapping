#include "FeatureExtractor.h"

using namespace std;

FeatureExtractor::FeatureExtractor(){}

FeatureExtractor::~FeatureExtractor(){}

void FeatureExtractor::setVerbose(bool b){verbose = b;}
void FeatureExtractor::setDebugg(bool b){debugg = b;}

KeyPointSet * FeatureExtractor::getKeyPointSet(FrameInput * fi){
	//printf("Extracting unknown feature\n");
	return new KeyPointSet();
}
