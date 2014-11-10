#include "ekz.h"

using namespace std;

int main(int argc, char **argv){
	printf("starting testing software2\n");
	string input = argv[1];

	Calibration * cal = new Calibration();
	cal->fx			= 525.0;
	cal->fy			= 525.0;
	cal->cx			= 319.5;
	cal->cy			= 239.5;
	cal->ds			= 1;
	cal->scale		= 5000;

	FrameMatcher * matcher = new AICK();
	RGBDSegmentation * seg = new RGBDSegmentation();
	FeatureExtractor *  fe = new SurfExtractor();

	vector< RGBDFrame * > frames;
	for(int i = 25; i <= 95; i+=5){
		printf("adding a new frame\n");
		char rgbbuf[512];
		char depthbuf[512];
		sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i);
		sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i);
		FrameInput * fi = new FrameInput(cal, string(rgbbuf) , string(depthbuf));
		frames.push_back(new RGBDFrame(fi,fe,seg));
	}

	for(unsigned int i = 1; i < frames.size(); i++){
		Transformation * t = matcher->getTransformation(frames.at(i-1),frames.at(i));
		t->show(false);
	}

	return 0;
}
