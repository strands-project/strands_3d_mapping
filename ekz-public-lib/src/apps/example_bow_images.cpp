#include "ekz.h"

using namespace std;

int main(int argc, char **argv){
	printf("starting testing software2\n");
	printf("give path to files as input\n");
	string input = argv[1];				//Folder to load data from
	string output = argv[2];			//path to save output and initial name
	int nr_files = atoi(argv[3]);		//max number of files to investigate
	int step = atoi(argv[4]);			//See how many files to step
	Map3D * m = new Map3Dbow(output);	//Create a bow map object
	m->setVerbose(true);				//Set the map to give text output

	vector< RGBDFrame * > frames;
	for(int i = step; i <= nr_files; i+=step){
		printf("adding a new frame\n");
		
		//Get paths to image files
		char rgbbuf[512];
		char depthbuf[512];
		sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i);
		sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i);
		
		printf("%i \n",i);
		//Add frame to map
		m->addFrame(string(rgbbuf) , string(depthbuf));
	}
	
	m->estimate();	//Estimate and save bag of words model.

	return 0;
}
