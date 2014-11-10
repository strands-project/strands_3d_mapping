#ifndef Calibration_H_
#define Calibration_H_
#include <string>
#include <stdio.h>
#include "../FeatureDescriptor/FeatureDescriptor.h"
#include <vector>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;
class Calibration{
	public:
	float fx;
	float fy;
	float cx;
	float cy;
	float ds;
	float scale;
	vector<FeatureDescriptor * > words;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pthread_mutex_t viewer_mutex;
	
	void loadWord(string path){
		printf("loadWord(%s)\n",path.c_str());
		string type = path.substr(path.find_last_of(".")+1);
		if(			type.compare("orb")  == 0){		//orb type feature
			words.push_back(new OrbFeatureDescriptor(path));
		}else if(	type.compare("surf64") == 0){	//Surf64 feature
			words.push_back(new SurfFeatureDescriptor64(path));
		}else if(	type.compare("surf128") == 0){	//Surf128 feature
			words.push_back(new SurfFeatureDescriptor128(path));
		}else{
			printf("couldnt load word, type not in calibration class\n");
		}
	}
	
	void loadWords(string path,string type, int nr_words){
		for(int i = 0; i < nr_words; i++){
			char buf[1024];
			sprintf(buf,"%s_%i.%s",path.c_str(),i,type.c_str());
			loadWord(string(buf));
		}
	}
};
#endif
