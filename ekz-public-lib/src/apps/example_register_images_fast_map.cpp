#include "ekz.h"

using namespace std;

int main(int argc, char **argv){
	printf("starting testing software2\n");
	printf("give path to files as input\n");
	string input = argv[1];
	
	string bow_path = argv[2];

	Map3D * m = new Map3D();	//Create a standard map object
	m->setVerbose(true);		//Set the map to give text output
	
	m->loadCalibrationWords(bow_path,"orb", 500);	//set bag of words to orb 500 orb features from bow_path
	m->setFeatureExtractor(new OrbExtractor());		//Use orb features
	
	int max_points = 300;							//Number of keypoints used by matcher
	int nr_iter = 10;								//Number of iterations the matcher will run
	float shrinking = 0.7;							//The rate of convergence for the matcher
	float bow_threshold = 0.15;						//Bag of words threshold to avoid investigating bad matches
	float distance_threshold = 0.015;				//Distance threshold to discard bad matches using euclidean information.
	float feature_threshold = 0.15;					//Feature threshold to discard bad matches using feature information.
	
	m->setMatcher(new BowAICK(max_points, nr_iter,shrinking,bow_threshold,distance_threshold,feature_threshold));//Create a new matcher

	vector< RGBDFrame * > frames;
	for(int i = 25; i <= 95; i+=5){
		printf("----------------------%i-------------------\nadding a new frame\n",i);
		
		//Get paths to image files
		char rgbbuf[512];
		char depthbuf[512];
		sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i);
		sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i);

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
