#include "ekz.h" //Include the library

#include "core/RGBDFrame.h"

using namespace std;

int main(int argc, char **argv){
	printf("starting testing software for pcd files\n");
	printf("sequentally matches frames in form of pcd files given as input\n");

	Map3D * m = new Map3D();	//Create a standard map object
	m->setVerbose(true);		//Set the map to give text output				
	for(int i = 1; i < argc; i++){
		string input = string(argv[i]);
		printf("input: %s\n",input.c_str());

		//Read cloud from a the user defined input
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input.c_str(), *cloud) == -1){continue;}

		//Add cloud to map
		m->addFrame(cloud);
	}
	
	vector<Matrix4f> poses = m->estimate();//Estimate poses for the frames using the map object.
	m->savePCD("test.pcd");
	//Print poses
	cout << "Poses:" << endl;
	for(unsigned int i = 0; i < poses.size(); i++){
		cout << poses.at(i) << endl << endl;
	}
	return 0;
}
