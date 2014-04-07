#include "ekz.h"

using namespace std;

int main(int argc, char **argv){
	printf("starting testing software2\n");
	printf("give path to files as input\n");
	string input = argv[1];

	Map3D * m = new Map3D();	//Create a standard map object
	m->setVerbose(true);		//Set the map to give text output	

	vector< RGBDFrame * > frames;
	for(int i = 5; i < 100; i+=5){
		printf("adding a new frame\n");
		
		//Get paths to image files
		char rgbbuf[512];
		char depthbuf[512];
		sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i+1);
		sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i+1);

		//Add frame to map
		m->addFrame(string(rgbbuf) , string(depthbuf));
	}
	
	vector<Matrix4f> poses = m->estimate();	//Estimate poses for the frames using the map object.
	m->savePCD("test.pcd");					//Saves a downsampled pointcloud with aligned data.
	
	//Print poses
	cout << "Poses:" << endl;
	for(unsigned int i = 0; i < poses.size(); i++){
		cout << poses.at(i) << endl << endl;
	}
	return 0;
}
