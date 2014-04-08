#include "Map3D.h"
#include <map>
using namespace std;

bool comparison_Map3D (Transformation * i,Transformation * j) {
	if(i->src == j->src){return (i->weight<j->weight);}
	else{return (i->src->id<j->src->id);}
}

void gen_random(char *s, const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];}
    s[len] = 0;
}

Map3D::Map3D(){
	verbose = false;

	matcher 			= new AICK();					//The keypoint matcher to be used for sequential frames
	loopclosure_matcher = new AICK();					//The keypoint matcher to be used for loopclosure
	segmentation 		= new RGBDSegmentation();		//Base segmentation class, no segmentation to be used.
	extractor 			= new SurfExtractor();			//Keypoint extractor, currently using Surf.
	
	calibration = new Calibration();				//Standard kinect parameters for the recorded pcd files
	calibration->fx			= 525.0;				//Focal Length X
	calibration->fy			= 525.0;				//Focal Length Y
	calibration->cx			= 319.5;				//Center coordinate X
	calibration->cy			= 239.5;				//Center coordinate X
	calibration->ds			= 1;					//Depth scaling for camera
	calibration->scale		= 5000;					//Depth scaling in file due to discretization.
}

Map3D::~Map3D(){}
void Map3D::addFrame(string rgb_path, string depth_path){								addFrame(calibration,rgb_path,depth_path);}
void Map3D::addFrame(Calibration * cal,string rgb_path, string depth_path){				addFrame(new FrameInput(cal,rgb_path,depth_path));}

void Map3D::addFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){						addFrame(calibration,cloud);}
void Map3D::addFrame(Calibration * cal, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){	addFrame(new FrameInput(cal, *cloud,false, frames.size(), "./"));}

void Map3D::addFrame(FrameInput * fi) {													addFrame(new RGBDFrame(fi,extractor,segmentation,verbose));}
void Map3D::addFrame(RGBDFrame * frame){
	if(frames.size() > 0){transformations.push_back(matcher->getTransformation(frame, frames.back()));}
	frames.push_back(frame);
}

void Map3D::setVerbose(bool v){
	verbose = v;
	matcher->verbose = verbose;
	loopclosure_matcher->verbose = verbose;
	extractor->verbose = verbose;
}

void Map3D::setMatcher(FrameMatcher * fm){
	if(matcher != 0){delete matcher;}
	matcher = fm;
	matcher->verbose = verbose;
}
void Map3D::setLoopClosureMatcher(FrameMatcher * fm){
	if(loopclosure_matcher != 0){delete loopclosure_matcher;}
	loopclosure_matcher = fm;
	loopclosure_matcher->verbose = verbose;
}
void Map3D::setSegmentation(RGBDSegmentation * seg){
	if(segmentation != 0){delete segmentation;}
	segmentation = seg;
}
void Map3D::setFeatureExtractor(FeatureExtractor * fe){
	if(extractor != 0){delete extractor;}
	extractor = fe;
	extractor->verbose = verbose;
}
void Map3D::setCalibration(Calibration * cal){
	if(calibration != 0){delete calibration;}
	calibration = cal;
}

void Map3D::loadCalibrationWords(string path,string type, int nr_words){
	calibration->loadWords(path,type,nr_words);
}
	
void Map3D::addTransformation(Transformation * transformation){}

vector<Matrix4f> Map3D::estimate(){
	sort(transformations.begin(),transformations.end(),comparison_Map3D);
	poses.push_back(Matrix4f::Identity());
	matcher->debugg = true;
	for(unsigned int i = 0; i < transformations.size(); i++){
		if(verbose){printf("Frame:%i ---------> %i : %f\n",transformations.at(i)->src->id,transformations.at(i)->dst->id,transformations.at(i)->weight);}
		//cout << transformations.at(i)->transformationMatrix << endl;
		poses.push_back(poses.back()*transformations.at(i)->transformationMatrix);
	}
	return poses;
}
void Map3D::savePCD(string path){savePCD(path,false, false, 0.01);}
void Map3D::savePCD(string path,bool randomcolor, bool trajectory, float resolution){
	if(verbose){printf("Saving map in: %s\n",path.c_str());}
	getLargestComponent();

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setLeafSize (resolution, resolution, resolution);

	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(verbose){printf("nr frames: %i \n nr poses: %i\n",(int)frames.size(),(int)poses.size());}
	for(unsigned int i = 0; i < largest_component.size(); i+=1){
		//printf("largest_component.at(%i)=%i,frames.at(largest_component.at(%i))->id = %i\n",i,largest_component.at(i),i,frames.at(largest_component.at(i))->id);
		pcl::PointCloud<pcl::PointXYZRGB> c = frames.at(largest_component.at(i))->input->getCloud();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud2->points.resize(c.points.size());
		float tmp_r = 255*float(rand()%1000)/1000.0f;
		float tmp_g = 255*float(rand()%1000)/1000.0f;
		float tmp_b = 255*float(rand()%1000)/1000.0f;

		int nr_pts = 0;
		for(unsigned int j = 0; j < c.points.size(); j++){
			if(c.points[j].z == 0 ){
				if(trajectory){
					cloud2->points[nr_pts].x = c.points[j].x;
					cloud2->points[nr_pts].y = c.points[j].y;
					cloud2->points[nr_pts].z = c.points[j].z;
					cloud2->points[nr_pts].r = 255;
					cloud2->points[nr_pts].g = 0;
					cloud2->points[nr_pts].b = 255;
					nr_pts++;
				}
			}else{
				cloud2->points[nr_pts].x = c.points[j].x;
				cloud2->points[nr_pts].y = c.points[j].y;
				cloud2->points[nr_pts].z = c.points[j].z;
				cloud2->points[nr_pts].r = c.points[j].r;
				cloud2->points[nr_pts].g = c.points[j].g;
				cloud2->points[nr_pts].b = c.points[j].b;
				if(randomcolor){
					cloud2->points[nr_pts].r = tmp_r;
					cloud2->points[nr_pts].g = tmp_g;
					cloud2->points[nr_pts].b = tmp_b;
				}
				nr_pts++;
			}
		}

		cloud2->points.resize(nr_pts);

		pcl::PointCloud<pcl::PointXYZRGB> c2;
		sor.setInputCloud (cloud2);
		sor.filter (c2);


		pcl::PointCloud<pcl::PointXYZRGB> c_trans;
		pcl::transformPointCloud (c2, c_trans, poses.at(largest_component.at(i)));
		*cloud += c_trans;
	}

	pcl::PointCloud<pcl::PointXYZRGB> voxelcloud;

	sor.setInputCloud (cloud);
	sor.filter (voxelcloud);

	pcl::io::savePCDFileBinary (path, voxelcloud);
	if(verbose){std::cerr << "Saved " << voxelcloud.points.size () << " data points." << std::endl;}
}

//Ugly and probably full of bugs...
vector<int> Map3D::getLargestComponent(){
	int nrframes = frames.size();
	bool connections[frames.size()][frames.size()];
	bool taken[frames.size()];
	for(unsigned int i = 0; i < frames.size(); i++){
		taken[i]=false;
		for(unsigned int j = 0; j < frames.size(); j++){
			connections[i][j]=false;
		}
	}
	
	if(verbose){printf("nr transformations: %i\n",(int)transformations.size());}
	sort(transformations.begin(),transformations.end(),comparison_Map3D);
	
	//Drawing code
/*
	IplImage * img 					= cvCreateImage(cvSize(nrframes, nrframes), IPL_DEPTH_8U, 3);
	char * img_data					= (char *)(img->imageData);
	cvNamedWindow("connections", CV_WINDOW_AUTOSIZE );
	cvRectangle(img,cvPoint(0,0),cvPoint(10000,10000),cvScalar(255, 255, 255, 0), -1 , 8, 0);

	for(int i = 0; i < frames.size(); i++){
		for(int j = 0; j < frames.size(); j++){
			float dist = frames.at(i)->image_descriptor->distance(frames.at(j)->image_descriptor);
			float small = 0.000;
			float large = 0.02;
			if(dist < small){
				cvRectangle(img,cvPoint(i,j),cvPoint(i,j),cvScalar(0, 255, 0, 0), 1 , 8, 0);
				cvRectangle(img,cvPoint(j,i),cvPoint(j,i),cvScalar(0, 255, 0, 0), 1 , 8, 0);
			}else if(dist > large){

				cvRectangle(img,cvPoint(i,j),cvPoint(i,j),cvScalar(0, 0, 255, 0), 1 , 8, 0);
				cvRectangle(img,cvPoint(j,i),cvPoint(j,i),cvScalar(0, 0, 255, 0), 1 , 8, 0);
			}else{
				float val = (dist-small)/(large-small);
				cvRectangle(img,cvPoint(i,j),cvPoint(i,j),cvScalar(val*255, val*255, val*255, 0), 1 , 8, 0);
				cvRectangle(img,cvPoint(j,i),cvPoint(j,i),cvScalar(val*255, val*255, val*255, 0), 1 , 8, 0);
			}
		}
	}
	cvShowImage("connections", img);
	if(!cvSaveImage("connections1.png",img)){printf("Could not save: \n");}
	cvWaitKey(0);
*/

	for(unsigned int i = 0; i < transformations.size(); i++){
		Transformation * transformation = transformations.at(i);
		if(transformation->weight > 0)
		{
			int w = transformation->src->id;
			int h = transformation->dst->id;
			//cvRectangle(img,cvPoint(w,h),cvPoint(w,h),cvScalar(255, 0, 255, 0), 1 , 8, 0);
			//cvRectangle(img,cvPoint(h,w),cvPoint(h,w),cvScalar(255, 0, 255, 0), 1 , 8, 0);
			connections[transformation->src->id][transformation->dst->id]=true;
			connections[transformation->dst->id][transformation->src->id]=true;
		}
	}
/*
	for(int i = 0; i < nrframes; i++){
		for(int j = 0; j < nrframes; j++){
			printf("%i ",connections[i][j]);
		}
		printf("\n");
	}

	cvShowImage("connections", img);
	if(!cvSaveImage("connections2.png",img)){printf("Could not save: \n");}
	cvWaitKey(30);
	cvReleaseImage( &img);
*/

	vector<vector<int> > all_matched;
	for(unsigned int i = 0; i < frames.size(); i++){
		int id1 = frames.at(i)->id;
		if(!taken[id1]){
			all_matched.push_back(vector<int>());
			vector<int> todo;
			todo.push_back(i);
			taken[id1] = true;
			if(verbose){printf("Matched segment:");}
			while(todo.size()>0){
				id1 = frames.at(todo.back())->id;
				all_matched.back().push_back(todo.back());
				todo.pop_back();
				for(unsigned int j = 0; j < frames.size(); j++){
					int id2 = frames.at(j)->id;
					if(!taken[id2] && connections[id1][id2]){

						//if(j < 296 && j > 291){printf("connections[%i][%i] = %i and taken[%i] = %i\n",id,id2,connections[id][id2],id2,taken[id2]);}
						todo.push_back(j);taken[id2] = true;
					}
				}
			}
			if(verbose){printf("\n Size: %i\n",(int)all_matched.back().size());}
			if(all_matched.back().size() < 5){
				for(unsigned int k = 0; k < all_matched.back().size();k++){
					if(verbose){printf("id: %i\n",frames.at(all_matched.back().at(k))->id);}
				}
			}
		}
	}

	largest_component.clear();
	for(unsigned int i = 0; i < all_matched.size(); i++){
		if(all_matched.at(i).size() > largest_component.size()){largest_component = all_matched.at(i);}
	}

	sort(largest_component.begin(),largest_component.end());
	return largest_component;
}

/*
void Map3D::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
void Map3D::visualize(){}
void Map3D::showTuning(){}


vector<int> Map3D::getLargestComponent(){
		int nrframes = frames.size();
		IplImage * img 					= cvCreateImage(cvSize(nrframes, nrframes), IPL_DEPTH_8U, 3);
		char * img_data					= (char *)(img->imageData);
		
		cvNamedWindow("connections", CV_WINDOW_AUTOSIZE );

		cvRectangle(img,cvPoint(0,0),cvPoint(10000,10000),cvScalar(255, 255, 255, 0), -1 , 8, 0);


	sort(transformations.begin(),transformations.end(),comparison_Map3D);

	bool connections[frames.size()][frames.size()];
	bool taken[frames.size()];
	for(int i = 0; i < frames.size(); i++){
		taken[i]=false;
		for(int j = 0; j < frames.size(); j++){
			connections[i][j]=false;
			float dist = frames.at(i)->image_descriptor->distance(frames.at(j)->image_descriptor);
			float small = 0.000;
			float large = 0.02;
			if(dist < small){
				cvRectangle(img,cvPoint(i,j),cvPoint(i,j),cvScalar(0, 255, 0, 0), 1 , 8, 0);
				cvRectangle(img,cvPoint(j,i),cvPoint(j,i),cvScalar(0, 255, 0, 0), 1 , 8, 0);
			}else if(dist > large){
				cvRectangle(img,cvPoint(i,j),cvPoint(i,j),cvScalar(0, 0, 255, 0), 1 , 8, 0);
				cvRectangle(img,cvPoint(j,i),cvPoint(j,i),cvScalar(0, 0, 255, 0), 1 , 8, 0);
			}else{
				float val = (dist-small)/(large-small);
				cvRectangle(img,cvPoint(i,j),cvPoint(i,j),cvScalar(val*255, val*255, val*255, 0), 1 , 8, 0);
				cvRectangle(img,cvPoint(j,i),cvPoint(j,i),cvScalar(val*255, val*255, val*255, 0), 1 , 8, 0);
			}
		}
	}
	cvShowImage("connections", img);
	if(!cvSaveImage("connections1.png",img)){printf("Could not save: \n");}
	cvWaitKey(30);

	for(int i = 0; i < transformations.size(); i++){
		Transformation * transformation = transformations.at(i);
		if(transformation->weight > 0)
		{
			int w = transformation->src->id;
			int h = transformation->dst->id;
			cvRectangle(img,cvPoint(w,h),cvPoint(w,h),cvScalar(255, 0, 255, 0), 1 , 8, 0);
			cvRectangle(img,cvPoint(h,w),cvPoint(h,w),cvScalar(255, 0, 255, 0), 1 , 8, 0);
			connections[transformation->src->id][transformation->dst->id]=true;
			connections[transformation->dst->id][transformation->src->id]=true;

		}
	}
	cvShowImage("connections", img);
	if(!cvSaveImage("connections2.png",img)){printf("Could not save: \n");}
	cvWaitKey(30);
	cvReleaseImage( &img);

	vector<vector<int> > all_matched;
	for(int i = 0; i < frames.size(); i++){
		int id1 = frames.at(i)->id;
		if(!taken[id1]){
			all_matched.push_back(vector<int>());
			vector<int> todo;
			todo.push_back(i);
			taken[id1] = true;
			printf("Matched segment:"); 
			while(todo.size()>0){
				id1 = frames.at(todo.back())->id;
				all_matched.back().push_back(todo.back());
				todo.pop_back();
				for(int j = 0; j < frames.size(); j++){
					int id2 = frames.at(j)->id;
					if(!taken[id2] && connections[id1][id2]){
						//if(j < 296 && j > 291){printf("connections[%i][%i] = %i and taken[%i] = %i\n",id,id2,connections[id][id2],id2,taken[id2]);}
						todo.push_back(j);taken[id2] = true;
					}
				}
			}
			printf("\n Size: %i\n",all_matched.back().size());
			if(all_matched.back().size() < 5){
				for(int k = 0; k < all_matched.back().size();k++){
					printf("id: %i\n",frames.at(all_matched.back().at(k))->id);
				}
			}
		}
	}

	largest_component.clear();
	for(int i = 0; i < all_matched.size(); i++){
		if(all_matched.at(i).size() > largest_component.size()){largest_component = all_matched.at(i);}
	}
	sort(largest_component.begin(),largest_component.end());
	return largest_component;
}

void Map3D::cleanTransformations(float threshold){
	printf("cleanTransformations\n");
	for(int i = 0; i < transformations.size();){
		if(transformations.at(i)->weight > threshold){
			i++;
		}else{
			delete transformations.at(i);
			transformations.at(i) = transformations.back();
			transformations.pop_back();
		}
	}

	vector< vector <Transformation *> > cycles;
	vector< float > values;

	vector<Transformation *> trans_arr[frames.size()];

	for(int i = 0; i < frames.size(); i++){trans_arr[i].clear();}

	for(int i = 0; i < transformations.size();i++){
		Transformation * t = transformations.at(i);
		trans_arr[t->src->id].push_back(t);
		trans_arr[t->dst->id].push_back(t);
	}
	printf("transformations.size(): %i\n",transformations.size());
	vector<Transformation *> trans[frames.size()];

	int depth = 5;
	for(int i = 0; i < transformations.size();i++){
		for(int i = 0; i < frames.size();i++){trans[i].clear();}

		Transformation * t = transformations.at(i);
		int start = t->src->id;
		int stop = t->dst->id;

		for(int l = 0; l < trans_arr[start].size(); l++){
			if(trans_arr[start].at(l) == t){
				trans_arr[start].at(l) = trans_arr[start].back();
				trans_arr[start].pop_back();
			}
		}

		for(int l = 0; l < trans_arr[stop].size(); l++){
			if(trans_arr[stop].at(l) == t){
				trans_arr[stop].at(l) = trans_arr[stop].back();
				trans_arr[stop].pop_back();
			}
		}

		trans[start].push_back(t);

		vector< int > todo_now;
		vector< int > todo_next;

		todo_now.push_back(start);
		for(int d = 0; d < depth; d++){
			for(int k = 0; k < todo_now.size(); k++){
				int kk = todo_now.at(k);
				for(int l = 0; l < trans_arr[kk].size(); l++){
					Transformation * t2 = trans_arr[kk].at(l);
					int dst_id = t2->dst->id;
					int src_id = t2->src->id;
					if(src_id == stop || dst_id == stop){
						cycles.push_back(vector< Transformation * >());
						for(int m = 0; m < trans[kk].size();m++){cycles.back().push_back(trans[kk].at(m));}
						cycles.back().push_back(t2);
					}else if(trans[src_id].size() == 0){
						for(int m = 0; m < trans[kk].size();m++){trans[src_id].push_back(trans[kk].at(m));}
						trans[src_id].push_back(t2);
						todo_next.push_back(src_id);
					}else if(trans[dst_id].size() == 0){
						for(int m = 0; m < trans[kk].size();m++){trans[dst_id].push_back(trans[kk].at(m));}
						trans[dst_id].push_back(t2);
						todo_next.push_back(dst_id);
					}
				}
			}
			todo_now = todo_next;
			todo_next.clear();
		}
	}
	
	vector<float> 			score;
	vector< vector<int> >	edges_index;
	vector< float >			edges_score;
	score.resize(transformations.size());
	map<Transformation *, int> m;

	float w_zero = 10.9;
	float w_scale = 5;
	for(int i = 0; i < transformations.size();i++){
		m.insert(std::pair<Transformation *, int>(transformations.at(i),i));
		float w = transformations.at(i)->weight;
		score.at(i) = w_scale*(w-w_zero);
		//if(score.at(i) < -5*w_scale){score.at(i) = -5*w_scale;}
	} 

	float e_zero = 0.005;
	float e_scale = 1;
	for(int i = 0; i < cycles.size(); i++){
		edges_index.push_back( vector<int>() );
		Transformation * current = cycles.at(i).at(0);
		int last_id = current->src->id;
		Eigen::Matrix4f diff = current->transformationMatrix;
		
		edges_index.back().push_back(m.find(current)->second);

		for(int j = 1; j < cycles.at(i).size(); j++){
			current = cycles.at(i).at(j);
			edges_index.back().push_back(m.find(current)->second);

			if(current->dst->id == last_id){
				diff *= current->transformationMatrix;
				last_id = current->src->id;
			}else{
				diff *= current->transformationMatrix.inverse();
				last_id = current->dst->id;
			}
		}

		float avg_error = sqrt(diff(0,3)*diff(0,3)+diff(1,3)*diff(1,3)+diff(2,3)*diff(2,3)) / float(cycles.at(i).size());
		float sc = e_scale*(e_zero-avg_error)/e_zero;
		if(sc < -10*e_scale){sc = -10*e_scale;}
		//cout<< "avg_error "<< avg_error << " sc: " << sc <<endl;

		edges_score.push_back(sc);
		for(int j = 0; j < edges_index.back().size(); j++){
			score.at(edges_index.back().at(j))+=sc;
		}
	}

	vector<int> 			vertex_ind;
	vector< vector<int> >	vertex_edges;

	vertex_ind.resize(score.size());
	vertex_edges.resize(score.size());
	for(int i = 0; i < vertex_ind.size(); i++){vertex_ind.at(i) = i;}

	for(int i = 0; i < edges_index.size(); i++){
		vector<int> ed = edges_index.at(i);
		for(int j = 0; j < ed.size();j++){
			vertex_edges.at(ed.at(j)).push_back(i);
		}
	}

	double worst_score = -1;
	int worst_ind;
	vector<int> to_remove;
	while(worst_score < 0 && vertex_ind.size() > 0){
		int w_ind = 0;
		worst_ind	= vertex_ind.at(w_ind);
		worst_score	= score.at(worst_ind);
		for(int i = 1; i < vertex_ind.size();i++){
			if(score.at(vertex_ind.at(i)) < worst_score){
				w_ind = i;
				worst_ind	= vertex_ind.at(i);
				worst_score	= score.at(vertex_ind.at(i));
			}
		}
		if(worst_score < 0){
			//printf("worst: %i -> %f\n",worst_ind,worst_score);
			//transformations.at(worst_ind)->print();
			to_remove.push_back(worst_ind);
	
			//For all edges remove scores

			for(int i = 0; i < vertex_edges.at(worst_ind).size();i++){
				int e_ind = vertex_edges.at(worst_ind).at(i);
				float sc = edges_score.at(e_ind);
				//printf("e_ind: %i sc: %f\n",e_ind,sc);
				edges_score.at(e_ind) = 0;//LAZY way of removing edge
				for(int j = 0; j < edges_index.at(e_ind).size(); j++){
					int vert = edges_index.at(e_ind).at(j);
					//printf("%i ",vert);
					score.at(vert) -= sc;
				}
				//printf("\n");
			}
			vertex_ind.at(w_ind) = vertex_ind.back();
			vertex_ind.pop_back();
		}
		//exit(0);
	}
	sort(to_remove.begin(),to_remove.end());
	printf("removesize: %i\n",to_remove.size());
	for(int i = to_remove.size()-1; i >=  0; i--){
		//printf("To remove: %i\n",to_remove.at(i));
		delete transformations.at(to_remove.at(i));
		transformations.at(to_remove.at(i)) = transformations.back();
		transformations.pop_back();
	} 
}

unsigned long Map3D::getCurrentTime(){
	struct timeval start;
	gettimeofday(&start, NULL);
	return start.tv_sec*1000000+start.tv_usec;
}

g2o::SE3Quat Map3D::getQuat(Eigen::Matrix4f mat){
	Eigen::Affine3f eigenTransform(mat);
	Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
	g2o::SE3Quat poseSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
	return poseSE3;
}

void Map3D::drawTraj(){
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setLeafSize (0.005f, 0.005f, 0.005f);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	printf("nr frames: %i \n nr poses: %i\n",frames.size(),poses.size());
	for(int i = 0; i < poses.size(); i+=1){
		pcl::PointCloud<pcl::PointXYZRGB> c = frames.at(i)->input->getCloud();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud2->points.resize(c.points.size());
		float tmp_r = 255*float(rand()%1000)/1000.0f;
		float tmp_g = 255*float(rand()%1000)/1000.0f;
		float tmp_b = 255*float(rand()%1000)/1000.0f;

		for(int j = 0; j < c.points.size(); j++){
			cloud2->points[j].x = c.points[j].x;
			cloud2->points[j].y = c.points[j].y;
			cloud2->points[j].z = c.points[j].z;
			if(cloud2->points[j].z == 0 || cloud2->points[j].z > 3){
				cloud2->points[j].r = 255;
				cloud2->points[j].g = 0;
				cloud2->points[j].b = 255;
				cloud2->points[j].x = 0;
				cloud2->points[j].y = 0;
				cloud2->points[j].z = 0;
			}else{
				cloud2->points[j].r = c.points[j].r;
				cloud2->points[j].g = c.points[j].g;
				cloud2->points[j].b = c.points[j].b;
			}
		}

		pcl::PointCloud<pcl::PointXYZRGB> c2;
		sor.setInputCloud (cloud2);
		sor.filter (c2);

		pcl::PointCloud<pcl::PointXYZRGB> c_trans;
		pcl::transformPointCloud (c2, c_trans, poses.at(i));
		*cloud += c_trans;
	}

	pcl::PointCloud<pcl::PointXYZRGB> voxelcloud;
	sor.setInputCloud (cloud);
	sor.filter (voxelcloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelcloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	*voxelcloud_ptr = voxelcloud;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer traj"));
	viewer->setBackgroundColor (255, 255, 255);
	viewer->initCameraParameters ();

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(voxelcloud_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB> (voxelcloud_ptr, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	char buf[1000];
	for(int c = 1; c < poses.size(); c++){
		pcl::PointXYZ a,b;
		a.x = poses.at(c)(0,3);		a.y = poses.at(c)(1,3);		a.z = poses.at(c)(2,3);
		b.x = poses.at(c-1)(0,3);	b.y = poses.at(c-1)(1,3);	b.z = poses.at(c-1)(2,3);
		sprintf(buf,"p%i",c);
		viewer->addLine<pcl::PointXYZ> (a,b,255,0,0, buf);
	}

	for(int c = 1; c < poses.size(); c++){
		Matrix4f m0 = frames.at(0)->input->gt.matrix().inverse()*frames.at(c-1)->input->gt.matrix();
		Matrix4f m1 = frames.at(0)->input->gt.matrix().inverse()*frames.at(c)->input->gt.matrix();
		pcl::PointXYZ a,b;
		a.x = m0(0,3);	a.y = m0(1,3);	a.z = m0(2,3);
		b.x = m1(0,3);	b.y = m1(1,3);	b.z = m1(2,3);
		//cout << a << " " << b << endl;
		sprintf(buf,"gt%i",c);
		viewer->addLine<pcl::PointXYZ> (a,b,0,255,0, buf);
	}
	
	while (!viewer->wasStopped ()){
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void Map3D::saveFBformat(string path){
	printf("Saving map in: %s\n",path.c_str());
	//getLargestComponent();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	printf("nr frames: %i \n nr poses: %i\n",frames.size(),poses.size());

	ofstream myfile;
	myfile.open(path.c_str());
	Matrix4f gt1inv = frames.at(0)->input->gt.matrix().inverse();
	
	char buf[1000];
	for(int i = 0; i < poses.size(); i++){

		//printf("pose: %i\n",i);
		Matrix4f p = poses.at(i);
		Eigen::Affine3d a(p.cast<double>());
		Eigen::Quaterniond qr(a.rotation());

		double timestamp = frames.at(i)->input->rgb_timestamp;

		float tx = a(0,3);
		float ty = a(1,3);
		float tz = a(2,3);
		float qx = qr.x();
		float qy = qr.y();
		float qz = qr.z();
		float qw = qr.w();

		int n = sprintf(buf,"%f %f %f %f %f %f %f %f\n",timestamp,tx,ty,tz,qx,qy,qz,qw);
		myfile << buf;
		//cout << i << " " << poses.size() << " " <<buf;
	}
	myfile.close();
}

void Map3D::savePoses(string path){
	printf("Saving map in: %s\n",path.c_str());
	printf("nr frames: %i \n nr poses: %i\n",frames.size(),poses.size());

	ofstream myfile;
	myfile.open(path.c_str());

	for(int i = 0; i < poses.size(); i+=1){
		unsigned int found = frames.at(i)->input->name.find_last_of("/");
		myfile<< frames.at(i)->input->name.substr(found+1) << endl << poses.at(i) << endl;
	}
	myfile.close();
}

bool comparison_frameid (RGBDFrame * i,RGBDFrame * j) {	return i->id < j->id;}
void Map3D::sortFrames(){ sort(frames.begin(),frames.end(),comparison_frameid);}

void Map3D::treePoseEst(){
	queue<int> todo;
	todo.push(0);
	bool taken[frames.size()];
	for(int i = 0; i < frames.size(); i++){taken[i]=false;}
	taken[0]=true;

	poses.resize(frames.size());
	poses.at(0) = Matrix4f::Identity();

	while(todo.size() > 0){
		int current = todo.front();
		todo.pop();
		printf("todo: %i\n",current);

		for(int i = 0; i < transformations.size(); i++){
			Transformation * transformation = transformations.at(i);
			if(transformation->weight > 0){
				int src_id = transformation->src->id;
				int dst_id = transformation->dst->id;

				if(src_id == current && !taken[dst_id]){
					taken[dst_id]=true;
					todo.push(dst_id);
					poses.at(dst_id)=poses.at(src_id)*transformation->transformationMatrix.inverse();
				}

				if(dst_id == current && !taken[src_id]){
					taken[src_id]=true;
					todo.push(src_id);
					poses.at(src_id)=poses.at(dst_id)*transformation->transformationMatrix;
				}


			}
		}
	}
}
*/
