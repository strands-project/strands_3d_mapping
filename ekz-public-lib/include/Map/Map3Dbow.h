#ifndef Map3Dbow_H_
#define Map3Dbow_H_

#include "Map3D.h"

using namespace std;

class Map3Dbow: public Map3D
{
	public:
	string path;
	int nr_restarts_;
	int iterations_;
	int nr_clusters_;
	Map3Dbow(string file_path);
	Map3Dbow(string file_path, int nr_restarts, int iterations, int nr_clusters);
	~Map3Dbow(); 
	vector<Matrix4f> estimate();
	vector<FeatureDescriptor * > * kmeans(vector<FeatureDescriptor *> input, int nr_restarts, int iterations, int nr_clusters);
};
#endif
