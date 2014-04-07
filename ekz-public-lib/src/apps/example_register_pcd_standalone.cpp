#include "ekz.h" //Include the library

using namespace std;

int main(int argc, char **argv){
	printf("starting testing software for pcd files\n");
	printf("sequentally matches frames in form of pcd files given as input\n");

	Calibration * cal = new Calibration();	//Standard kinect parameters for the recorded pcd files
	cal->fx			= 525.0;				//Focal Length X
	cal->fy			= 525.0;				//Focal Length Y
	cal->cx			= 319.5;				//Center coordinate X
	cal->cy			= 239.5;				//Center coordinate X
	cal->ds			= 1;					//Depth scaling for camera
	cal->scale		= 5000;					//Depth scaling in file due to discretization.

	FrameMatcher * matcher = new AICK();				//The keypoint matcher to be used
	RGBDSegmentation * seg = new RGBDSegmentation();	//Base segmentation class, no segmentation to be used.
	FeatureExtractor *  fe = new SurfExtractor();		//Keypoint extractor, currently using Surf.

	vector< RGBDFrame * > frames;
	for(int i = 1; i < argc; i++){
		string input = string(argv[i]);
		printf("input: %s\n",input.c_str());

		//Read cloud from a the user defined input
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input.c_str(), *cloud) == -1){continue;}
		
		//Create a FrameInput object from calibration and from the loaded cloud, the last three parameters controll wether the FrameInput should be stored on disc aswell.
		FrameInput * fi = new FrameInput(cal, *cloud,false, i, "./");
		
		//Create a RGBD Frame using a FrameInput, a FeatureExtractor and a segmentation algorithm.
		frames.push_back(new RGBDFrame(fi,fe,seg));
	}
	
	//manually register sequential frames.
	for(unsigned int i = 1; i < frames.size(); i++){
		Transformation * t = matcher->getTransformation(frames.at(i-1),frames.at(i));	//Estimate transformation between frame i-1 and frame i
		t->show(false);																	//Vizualize the transformation, do not draw in 3d. Set true for 3d vizualizer. 
	}

	return 0;
}
