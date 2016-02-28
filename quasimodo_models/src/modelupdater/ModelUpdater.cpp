#include "ModelUpdater.h"

#include <boost/graph/incremental_components.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/graph/copy.hpp>
#include <unordered_map>

typedef boost::property<boost::edge_weight_t, float> edge_weight_property;
typedef boost::property<boost::vertex_name_t, size_t> vertex_name_property;
using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, vertex_name_property, edge_weight_property>;
using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
using VertexIndex = boost::graph_traits<Graph>::vertices_size_type;
using Edge = boost::graph_traits<Graph>::edge_descriptor;
using Components = boost::component_index<VertexIndex>;

using namespace std;

namespace reglib
{

double getTime(){
	struct timeval start1;
	gettimeofday(&start1, NULL);
	return double(start1.tv_sec+(start1.tv_usec/1000000.0));
}

float simd_score(int * part,int nr_data, float * scores, int padding){
	int dataAndPadding = nr_data+padding;
	__m128*  scores_ptr = (__m128*)scores;
	__m128i* part_ptr =   (__m128i*)part;
	__m128 simd_sum {0,0,0,0};
	for(int i = 0; i < nr_data; i++){
		int parti = part[i];
		__m128i simd_parti = _mm_set_epi32(parti,parti,parti,parti);
		int muli = (dataAndPadding*i)/4;
		for(int j = (i+1)/4; j < dataAndPadding/4; j++){
			simd_sum = _mm_add_ps(simd_sum,_mm_and_ps(scores_ptr[muli+j],(__m128)_mm_cmpeq_epi32(simd_parti, part_ptr[j])));// = rand()%1000 - 500;
		}
	}
	float * simd_sum_vec = (float*)(&simd_sum);
	float ssum = simd_sum_vec[0] + simd_sum_vec[1] + simd_sum_vec[2] + simd_sum_vec[3];
	return 2*ssum;
}

float build_part_var_fast(int dims, float* best_score, int* best_part, int * var, int * part,int nr_data, float * scores, int padding, int todo){
	int indtoset = var[todo];
	float best = -99999999999999999999999999;
	if(todo > 0){
		for(int i = 0; i < dims; i++){
			part[indtoset] = i;
			float current = build_part_var_fast(dims,best_score, best_part,var, part,nr_data,scores, padding, todo-1);
			best = std::max(best,current);
		}
	}else{
		for(int i = 0; i < dims; i++){
			part[indtoset] = i;
			float current = simd_score(part,nr_data,scores, padding);
			best = std::max(best,current);
			if(current > best_score[0]){
				best_score[0] = current;
				for(int i = 0; i < nr_data+padding; i++){best_part[i] = part[i];}
			}
		}
	}
	return best;
}

std::vector<int> getPartition2(std::vector< std::vector< float > > & scores, int dims){
	int nr_data = scores.size();
	int padding = 4-(nr_data-4*(nr_data/4));

	float* scorespadded;
	posix_memalign((void**)&scorespadded, 16,  (nr_data+padding) *nr_data * sizeof(float));

	for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
	for(int i = 0; i < nr_data; i++){
		for(int j = i+1; j < nr_data; j++){
			scorespadded[(nr_data+padding)*i+j] = scores[i][j];
		}
	}

	int * var = new int[nr_data];
	for(int i = 0; i < nr_data; i++){var[i] = i;}

	int* partpadded;
	posix_memalign((void**)&partpadded, 16,  (nr_data+padding) * sizeof(int));
	for(int i = 0; i < nr_data+padding; i++){partpadded[i] = 0;}

	float* best_score = new float[1];
	int* best_partpadded;
	posix_memalign((void**)&best_partpadded, 16,  (nr_data+padding) * sizeof(int));

	for(int i = 0; i < nr_data+padding; i++){best_partpadded[i] = partpadded[i];}
	best_score[0] = simd_score(best_partpadded,nr_data,scorespadded, padding);

	for(int i = 0; i < nr_data; i++){partpadded[i] = best_partpadded[i];}
	build_part_var_fast(dims,best_score, best_partpadded, var,partpadded,nr_data, scorespadded,padding, nr_data-1);


	std::vector<int> ret;
	ret.resize(nr_data);
	for(int i = 0; i < nr_data; i++){ret[i] = best_partpadded[i];}

	delete[] best_score;
	delete[] var;
	delete[] partpadded;
	delete[] scorespadded;
	delete[] best_partpadded;

	return ret;
}

float graph_cut(vector<Graph*>& graphs_out,vector<vector<int>>& second_graphinds, Graph& graph_in, std::vector<int> graph_inds){
	using adjacency_iterator = boost::graph_traits<Graph>::adjacency_iterator;
	typename boost::property_map<Graph, boost::vertex_index_t>::type vertex_id		= boost::get(boost::vertex_index, graph_in);
	typename boost::property_map<Graph, boost::edge_weight_t>::type  edge_id		= boost::get(boost::edge_weight, graph_in);
	typename boost::property_map<Graph, boost::vertex_name_t>::type  vertex_name	= boost::get(boost::vertex_name, graph_in);

	BOOST_AUTO(parities, boost::make_one_bit_color_map(boost::num_vertices(graph_in), boost::get(boost::vertex_index, graph_in)));

	float w = boost::stoer_wagner_min_cut(graph_in, boost::get(boost::edge_weight, graph_in), boost::parity_map(parities));

	unordered_map<VertexIndex, VertexIndex> mappings;
	VertexIndex counters[2] = {0, 0};

	graphs_out.push_back(new Graph(1));
	graphs_out.push_back(new Graph(1));
	second_graphinds.push_back(vector<int>());
	second_graphinds.push_back(vector<int>());
	//std::cout << "One set of vertices consists of:" << std::endl;
	bool flag;
	Edge edge;
	for (size_t i = 0; i < boost::num_vertices(graph_in); ++i) {
		int first = boost::get(parities, i);
		second_graphinds[first].push_back(graph_inds[i]);
		// iterate adjacent edges
		adjacency_iterator ai, ai_end;
		for (tie(ai, ai_end) = boost::adjacent_vertices(i, graph_in);  ai != ai_end; ++ai) {
			VertexIndex neighbor_index = boost::get(vertex_id, *ai);
			int second = boost::get(parities, neighbor_index);
			if (first == second && neighbor_index < i) {
				tie(edge, flag) = boost::edge(i, neighbor_index, graph_in);
				edge_weight_property weight = boost::get(edge_id, edge);
				if (mappings.count(i) == 0) {
					mappings[i] = counters[first]++;
				}
				if (mappings.count(neighbor_index) == 0) {
					mappings[neighbor_index] = counters[first]++;
				}
				tie(edge, flag) = boost::add_edge(mappings[neighbor_index], mappings[i], weight, *graphs_out[first]);

				typename boost::property_map<Graph, boost::vertex_name_t>::type vertex_name_first = boost::get(boost::vertex_name, *graphs_out[first]);
				boost::get(vertex_name_first, mappings[i]) = boost::get(vertex_name, i);
				boost::get(vertex_name_first, mappings[neighbor_index]) = boost::get(vertex_name, *ai);
			}
		}
	}
	return w;
}

float recursive_split(std::vector<Graph*> * graphs_out,std::vector<std::vector<int>> * graphinds_out, Graph * graph, std::vector<int> graph_inds){
	if(boost::num_vertices(*graph) == 1){
		graphs_out->push_back(graph);
		graphinds_out->push_back(graph_inds);
		return 0;
	}

	vector<Graph*> second_graphs;
	vector<vector<int>> second_graphinds;
	float w = graph_cut(second_graphs,second_graphinds,*graph,graph_inds);
	if(w <= 0){
		delete graph;
		return 2*w + recursive_split(graphs_out, graphinds_out,second_graphs.front(),second_graphinds.front()) + recursive_split(graphs_out, graphinds_out, second_graphs.back(),second_graphinds.back());
	}else{
		graphs_out->push_back(graph);
		graphinds_out->push_back(graph_inds);
		delete second_graphs.front();
		delete second_graphs.back();
		return 0;
	}
}

std::vector<int> partition_graph(std::vector< std::vector< float > > & scores){
	int nr_data = scores.size();
	Graph* graph = new Graph(nr_data);
	std::vector<int> graph_inds;
	graph_inds.resize(nr_data);

	typename boost::property_map<Graph, boost::vertex_name_t>::type vertex_name = boost::get(boost::vertex_name, *graph);

	float sum = 0;
	for(int i = 0; i < nr_data; i++){
		graph_inds[i] = i;
		for(int j = i+1; j < nr_data; j++){
			float weight = scores[i][j];
			if(weight != 0){
				sum += 2*weight;
				edge_weight_property e = weight;
				boost::add_edge(i, j, e, *graph);
			}
		}
	}

	std::vector<Graph*> * graphs_out = new std::vector<Graph*>();
	std::vector<std::vector<int>> * graphinds_out = new std::vector<std::vector<int>>();
	float best = sum-recursive_split(graphs_out,graphinds_out, graph,graph_inds );

	std::vector<int> part;
	part.resize(nr_data);
	//int * part = new int[nr_data];
	for(unsigned int i = 0; i < graphinds_out->size(); i++){
		for(unsigned int j = 0; j < graphinds_out->at(i).size(); j++){
			part[graphinds_out->at(i).at(j)] = i;
		}
	}
	return part;

//	printf("best: %f\n",best/float(nr_data*nr_data));
//	for(unsigned int i = 0; i < graphinds_out->size(); i++){
//		printf("%i -> ",i);
//		for(unsigned int j = 0; j < graphinds_out->at(i).size(); j++){printf("%i ",graphinds_out->at(i).at(j));}
//		printf("\n");
//	}
//	for(int i = 0; i < nr_data; i++){printf("%i ",part[i]);}printf("\n");
}

std::vector<int> ModelUpdater::getPartition(std::vector< std::vector< float > > & scores, int dims, int nr_todo, double timelimit){
	return partition_graph(scores);
	/*
	int nr_data = scores.size();
	if(pow(dims,nr_data) < pow(2,22)){
		printf("SMALL CASE\n");
		return getPartition2(scores, dims);
	}
	int padding = 4-(nr_data-4*(nr_data/4));

	float* scorespadded;
	posix_memalign((void**)&scorespadded, 16,  (nr_data+padding) *nr_data * sizeof(float));

	for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
	for(int i = 0; i < nr_data; i++){
		for(int j = i+1; j < nr_data; j++){
			scorespadded[(nr_data+padding)*i+j] = scores[i][j];
		}
	}

	int * var = new int[nr_data];
	for(int i = 0; i < nr_data; i++){var[i] = i;}

	int* partpadded;
	posix_memalign((void**)&partpadded, 16,  (nr_data+padding) * sizeof(int));
	for(int i = 0; i < nr_data+padding; i++){partpadded[i] = 0;}

	float* best_score = new float[1];
	int* best_partpadded;
	posix_memalign((void**)&best_partpadded, 16,  (nr_data+padding) * sizeof(int));

	for(int i = 0; i < nr_data+padding; i++){best_partpadded[i] = partpadded[i];}
	best_score[0] = simd_score(best_partpadded,nr_data,scorespadded, padding);

	if(nr_todo > nr_data){nr_todo = nr_data;}

	double start = getTime();
	for(int it = 0; (getTime()-start) < timelimit; it++){
		for(int i = 0; i < nr_todo; i++){//Randomize order to select variables for changes
			int ind = rand()%nr_data;
			int tmp = var[ind];
			var[ind] = var[i];
			var[i] = tmp;
		}

		for(int i = 0; i < nr_data; i++){partpadded[i] = best_partpadded[i];}
		build_part_var_fast(dims,best_score, best_partpadded, var,partpadded,nr_data, scorespadded,padding, nr_todo-1);
	}

	std::vector<int> ret;
	ret.resize(nr_data);
	for(int i = 0; i < nr_data; i++){ret[i] = best_partpadded[i];}

	delete[] best_score;
	delete[] var;
	delete[] partpadded;
	delete[] scorespadded;
	delete[] best_partpadded;

	return ret;
	*/
}

ModelUpdater::ModelUpdater(){model = 0;}
ModelUpdater::ModelUpdater(Model * model_){	model = model_;}
ModelUpdater::~ModelUpdater(){}

FusionResults ModelUpdater::registerModel(Model * model2, Eigen::Matrix4d guess, double uncertanity){return FusionResults();}

void ModelUpdater::fuse(Model * model2, Eigen::Matrix4d guess, double uncertanity){
	//Blind addidtion of new model to old model...
	for(unsigned int i = 0; i < model2->frames.size();i++){
		model->frames.push_back(model2->frames[i]);
		model->masks.push_back(model2->masks[i]);
		model->relativeposes.push_back(guess*model2->relativeposes[i]);
	}
}

UpdatedModels ModelUpdater::fuseData(FusionResults * f, Model * model1,Model * model2){return UpdatedModels();}

void ModelUpdater::refine(){


}//No refinement behaviour added yet

void ModelUpdater::show(bool stop){
	//printf("void ModelUpdater::show()\n");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = model->getPCLcloud(1,false);
	viewer->removeAllPointClouds();
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
	if(stop){	viewer->spin();}
	else{		viewer->spinOnce();}
	viewer->removeAllPointClouds();
}

//Backproject and prune occlusions
void ModelUpdater::pruneOcclusions(){}

OcclusionScore ModelUpdater::computeOcclusionScore(RGBDFrame * src, cv::Mat src_mask, RGBDFrame * dst, cv::Mat dst_mask,Eigen::Matrix4d p, bool debugg){
	OcclusionScore oc;

	unsigned char  * src_maskdata		= (unsigned char	*)(src_mask.data);
	unsigned char  * src_rgbdata		= (unsigned char	*)(src->rgb.data);
	unsigned short * src_depthdata		= (unsigned short	*)(src->depth.data);
	float		   * src_normalsdata	= (float			*)(src->normals.data);

	unsigned char  * dst_maskdata		= (unsigned char	*)(dst_mask.data);
	unsigned char  * dst_rgbdata		= (unsigned char	*)(dst->rgb.data);
	unsigned short * dst_depthdata		= (unsigned short	*)(dst->depth.data);
	float		   * dst_normalsdata	= (float			*)(dst->normals.data);

	float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
	float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
	float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

	Camera * src_camera				= src->camera;
	const unsigned int src_width	= src_camera->width;
	const unsigned int src_height	= src_camera->height;
	const float src_idepth			= src_camera->idepth_scale;
	const float src_cx				= src_camera->cx;
	const float src_cy				= src_camera->cy;
	const float src_ifx				= 1.0/src_camera->fx;
	const float src_ify				= 1.0/src_camera->fy;

	Camera * dst_camera				= dst->camera;
	const unsigned int dst_width	= dst_camera->width;
	const unsigned int dst_height	= dst_camera->height;
	const float dst_idepth			= dst_camera->idepth_scale;
	const float dst_cx				= dst_camera->cx;
	const float dst_cy				= dst_camera->cy;
	const float dst_fx				= dst_camera->fx;
	const float dst_fy				= dst_camera->fy;
	const float dst_ifx				= 1.0/dst_camera->fx;
	const float dst_ify				= 1.0/dst_camera->fy;
	const unsigned int dst_width2	= dst_camera->width  - 2;
	const unsigned int dst_height2	= dst_camera->height - 2;

	//bool debugg = false;
	cv::Mat debugg_img;
	unsigned char  * debugg_img_data;
	if(debugg){
		debugg_img		= src->rgb.clone();
		debugg_img_data	= (unsigned char	*)(debugg_img.data);
	}

	std::vector<float> residuals;
	std::vector<int> ws;
	std::vector<int> hs;
	residuals.reserve(src_width*src_height);
	if(debugg){
		ws.reserve(src_width*src_height);
		hs.reserve(src_width*src_height);
	}

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	scloud->points.resize(src_width*src_height);
	dcloud->points.resize(dst_width*dst_height);

    double sum = 0;
    double count = 0;
    for(unsigned int src_w = 0; src_w < src_width-1; src_w++){
        for(unsigned int src_h = 0; src_h < src_height;src_h++){
            int src_ind0 = src_h*src_width+src_w-1;
            int src_ind1 = src_h*src_width+src_w;
            int src_ind2 = src_h*src_width+src_w+1;
            if(src_maskdata[src_ind0] == 255 && src_maskdata[src_ind1] == 255 && src_maskdata[src_ind2] == 255){// && p.z > 0 && !isnan(p.normal_x)){
                float z0 = src_idepth*float(src_depthdata[src_ind0]);
                float z1 = src_idepth*float(src_depthdata[src_ind1]);
                float z2 = src_idepth*float(src_depthdata[src_ind2]);

                double diff0 = (z0-z1)/(z0*z0+z1*z1);
                double diff1 = (z2-z1)/(z2*z2+z1*z1);
                if( fabs(diff0) < fabs(diff1)){
                    if(diff0 != 0){
                        sum += diff0*diff0;
                        count++;
                    }
                }else{
                    if(diff1 != 0){
                        sum += diff1*diff1;
                        count++;
                    }
                }

            }
        }
    }
    double me = sqrt(sum/(count+1));
    double pred = 2.0*sqrt(2)*me;
    //printf("pred error: %f -> %f \n",me,sqrt(2)*me);

	for(unsigned int src_w = 0; src_w < src_width; src_w++){
		for(unsigned int src_h = 0; src_h < src_height;src_h++){
			int src_ind = src_h*src_width+src_w;
			if(src_maskdata[src_ind] == 255){// && p.z > 0 && !isnan(p.normal_x)){
				float z = src_idepth*float(src_depthdata[src_ind]);
				float nx = src_normalsdata[3*src_ind+0];

				if(z > 0 && nx != 2){
					float ny = src_normalsdata[3*src_ind+1];
					float nz = src_normalsdata[3*src_ind+2];

					float x = (float(src_w) - src_cx) * z * src_ifx;
					float y = (float(src_h) - src_cy) * z * src_ify;

					float tx	= m00*x + m01*y + m02*z + m03;
					float ty	= m10*x + m11*y + m12*z + m13;
					float tz	= m20*x + m21*y + m22*z + m23;
					float tnx	= m00*nx + m01*ny + m02*nz;
					float tny	= m10*nx + m11*ny + m12*nz;
					float tnz	= m20*nx + m21*ny + m22*nz;

					float itz	= 1.0/tz;
					float dst_w	= dst_fx*tx*itz + dst_cx;
					float dst_h	= dst_fy*ty*itz + dst_cy;

					if((dst_w > 0) && (dst_h > 0) && (dst_w < dst_width2) && (dst_h < dst_height2)){
						unsigned int dst_ind = unsigned(dst_h+0.5) * dst_width + unsigned(dst_w+0.5);

						float dst_z = dst_idepth*float(dst_depthdata[dst_ind]);
						if(dst_z > 0){
							float dst_x = (float(dst_w) - dst_cx) * dst_z * dst_ifx;
							float dst_y = (float(dst_h) - dst_cy) * dst_z * dst_ify;
							//float diff_z2 = tnx*(dst_x-tx)+tny*(dst_y-ty)+tnz*(dst_z-tz);
							float diff_z2 = (dst_z-tz)/(z*z+dst_z*dst_z);

							float diff_z = (dst_z-tz)/(z*z+dst_z*dst_z);//if tz < dst_z then tz infront and diff_z > 0
							if(diff_z < 0 && diff_z2 > 0){diff_z2 *= -1;}
							if(diff_z > 0 && diff_z2 < 0){diff_z2 *= -1;}
							diff_z2 /= (z*z+dst_z*dst_z);//if tz < dst_z then tz infront and diff_z > 0



							residuals.push_back(diff_z);

//							if(residuals.size()%1000 == 1){
//printf("w: %5.5i h: %5.5i z: %5.5f tz: %5.5f dz: %5.5f diff: %5.5f scaled: %5.5f\n",src_w,src_h, z,tz,dst_z,(dst_z-tz),(dst_z-tz)/(z*z+dst_z*dst_z));
//								printf("%5.5i scaled: %10.10f scaled: %10.10f\n",residuals.size(),residuals.back(),(dst_z-tz)/(z*z+dst_z*dst_z));
//							}
							if(debugg){
								scloud->points[src_ind].x = tx;
								scloud->points[src_ind].y = ty;
								scloud->points[src_ind].z = tz+2;
								scloud->points[src_ind].r = 0;
								scloud->points[src_ind].g = 0;
								scloud->points[src_ind].b = 255;
								ws.push_back(src_w);
								hs.push_back(src_h);
							}
						}
					}
				}
			}
		}
	}


//exit(0);
	if(debugg){
		for(unsigned int dst_w = 0; dst_w < dst_width; dst_w++){
			for(unsigned int dst_h = 0; dst_h < dst_height;dst_h++){
				int dst_ind = dst_h*dst_width+dst_w;
				if(true || dst_maskdata[dst_ind] == 255){// && p.z > 0 && !isnan(p.normal_x)){
					float z = dst_idepth*float(dst_depthdata[dst_ind]);
					if(z > 0){// && (dst_w%3 == 0) && (dst_h%3 == 0)){
						float x = (float(dst_w) - dst_cx) * z * dst_ifx;
						float y = (float(dst_h) - dst_cy) * z * dst_ify;
						dcloud->points[dst_ind].x = x;
						dcloud->points[dst_ind].y = y;
						dcloud->points[dst_ind].z = z+2;
						dcloud->points[dst_ind].r = dst_rgbdata[3*dst_ind+2];
						dcloud->points[dst_ind].g = dst_rgbdata[3*dst_ind+1];
						dcloud->points[dst_ind].b = dst_rgbdata[3*dst_ind+0];
					}
				}
			}
		}
	}

/*
	for(unsigned int i = 0; i < residuals.size(); i+=1000){
		float r = residuals[i];
		printf("%5.5i -> %10.10f\n",i,r);
	}
*/
	//DistanceWeightFunction2PPR * func = new DistanceWeightFunction2PPR(100);
	DistanceWeightFunction2PPR2 * func = new DistanceWeightFunction2PPR2();
	func->maxp			= 1.0;
	func->update_size	= true;
    func->zeromean      = true;
    func->startreg		= 0.0001;
    func->debugg_print	= debugg;
	func->bidir			= true;
    func->maxnoise      = pred;
	func->reset();

	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(1,residuals.size());
	for(int i = 0; i < residuals.size(); i++){X(0,i) = residuals[i];}
	func->computeModel(X);

    //printf("predicted: %f found: %f\n",pred,func->getNoise()-func->startreg);

	Eigen::VectorXd  W = func->getProbs(X);
/*
	int dbres = 500;
	Eigen::MatrixXd X2 = Eigen::MatrixXd::Zero(1,dbres);
	for(int i = 0; i < dbres; i++){X2(0,i) = func->maxd*double(i-dbres/2)/double(dbres/2);}

	//func->computeModel(X2);
	Eigen::VectorXd  W2 = func->getProbs(X2);
	//printf("rscore = [");			for(unsigned int k = 0; k < dbres; k++){printf("%i ",int(W2(k)));}		printf("];\n");

	printf("rscore = [");
	for(unsigned int i = 0; i < dbres; i++){
		float r = X2(0,i);
		float weight = W2(i);
		float ocl = 0;
		if(r > 0){ocl += 1-weight;}
		double score = weight-5.0*ocl;
		printf("%5.5f ",score);

	}
	printf("];\n");
*/
	delete func;
/*
	for(unsigned int i = 0; i < dbres; i++){
		float r = X2(0,i);
		float weight = W2(i);
		printf("i:%5.5i r:%10.10f weight:%10.10f\n",i,r,weight);
	}
	*/
	//printf("rscore = [");			for(unsigned int k = 0; k < dbres; k++){printf("%i ",int(W2(k)));}		printf("];\n");
	//delete func;
/*
	for(unsigned int i = 0; i < 100; i++){
		int ind = rand()%residuals.size();
		float r = residuals[ind];
		float weight = W(ind);
		printf("%5.5i %5.5i -> %10.10f %10.10f\n",i,ind,r,weight);
	}
*/
//exit(0);
	for(unsigned int i = 0; i < residuals.size(); i++){
		float r = residuals[i];
		float weight = W(i);
		//if(fabs(r) > 0.0005){weight = 0;}//Replace with PPR

		float ocl = 0;
		if(r > 0){ocl += 1-weight;}

		oc.score		+= weight;
		oc.occlusions	+= ocl;

		if(debugg){
			int w = ws[i];
			int h = hs[i];
			unsigned int src_ind = h * src_width + w;


			if(ocl > 0.01 || weight > 0.01){
				scloud->points[src_ind].r = 255.0*ocl;
				scloud->points[src_ind].g = 255.0*weight;
				scloud->points[src_ind].b = 0;
			}else{
				scloud->points[src_ind].x = 0;
				scloud->points[src_ind].y = 0;
				scloud->points[src_ind].z = 0;
			}
			debugg_img_data[3*src_ind+0] = 0;
			debugg_img_data[3*src_ind+1] = 255.0*weight;
			debugg_img_data[3*src_ind+2] = 255.0*ocl;
		}
	}

	if(debugg){
		viewer->removeAllPointClouds();
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
		cv::namedWindow("debuggimage",		cv::WINDOW_AUTOSIZE );
		cv::imshow(		"debuggimage",		debugg_img );
		//cv::waitKey(30);
		viewer->spin();
		viewer->removeAllPointClouds();
	}
	return oc;
}
/*
OcclusionScore ModelUpdater::computeAllOcclusionScore(RGBDFrame * src, cv::Mat src_mask, RGBDFrame * dst, cv::Mat dst_mask,Eigen::Matrix4d p, bool debugg){
	OcclusionScore oc;

	unsigned char  * src_maskdata		= (unsigned char	*)(src_mask.data);
	unsigned char  * src_rgbdata		= (unsigned char	*)(src->rgb.data);
	unsigned short * src_depthdata		= (unsigned short	*)(src->depth.data);
	float		   * src_normalsdata	= (float			*)(src->normals.data);

	unsigned char  * dst_maskdata		= (unsigned char	*)(dst_mask.data);
	unsigned char  * dst_rgbdata		= (unsigned char	*)(dst->rgb.data);
	unsigned short * dst_depthdata		= (unsigned short	*)(dst->depth.data);
	float		   * dst_normalsdata	= (float			*)(dst->normals.data);

	float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
	float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
	float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

	Camera * src_camera				= src->camera;
	const unsigned int src_width	= src_camera->width;
	const unsigned int src_height	= src_camera->height;
	const float src_idepth			= src_camera->idepth_scale;
	const float src_cx				= src_camera->cx;
	const float src_cy				= src_camera->cy;
	const float src_ifx				= 1.0/src_camera->fx;
	const float src_ify				= 1.0/src_camera->fy;

	Camera * dst_camera				= dst->camera;
	const unsigned int dst_width	= dst_camera->width;
	const unsigned int dst_height	= dst_camera->height;
	const float dst_idepth			= dst_camera->idepth_scale;
	const float dst_cx				= dst_camera->cx;
	const float dst_cy				= dst_camera->cy;
	const float dst_fx				= dst_camera->fx;
	const float dst_fy				= dst_camera->fy;
	const float dst_ifx				= 1.0/dst_camera->fx;
	const float dst_ify				= 1.0/dst_camera->fy;
	const unsigned int dst_width2	= dst_camera->width  - 2;
	const unsigned int dst_height2	= dst_camera->height - 2;

	//bool debugg = false;
	cv::Mat debugg_img;
	unsigned char  * debugg_img_data;
	if(debugg){
		debugg_img		= src->rgb.clone();
		debugg_img_data	= (unsigned char	*)(debugg_img.data);
	}

	std::vector<float> residuals;
	std::vector<int> ws;
	std::vector<int> hs;
	residuals.reserve(src_width*src_height);
	if(debugg){
		ws.reserve(src_width*src_height);
		hs.reserve(src_width*src_height);
	}

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	scloud->points.resize(src_width*src_height);
	dcloud->points.resize(dst_width*dst_height);

	for(unsigned int src_w = 0; src_w < src_width; src_w++){
		for(unsigned int src_h = 0; src_h < src_height;src_h++){
			int src_ind = src_h*src_width+src_w;
			if(src_maskdata[src_ind] == 255){// && p.z > 0 && !isnan(p.normal_x)){
				float z = src_idepth*float(src_depthdata[src_ind]);
				float nx = src_normalsdata[3*src_ind+0];

				if(z > 0 && nx != 2){
					float ny = src_normalsdata[3*src_ind+1];
					float nz = src_normalsdata[3*src_ind+2];

					float x = (float(src_w) - src_cx) * z * src_ifx;
					float y = (float(src_h) - src_cy) * z * src_ify;

					float tx	= m00*x + m01*y + m02*z + m03;
					float ty	= m10*x + m11*y + m12*z + m13;
					float tz	= m20*x + m21*y + m22*z + m23;
					if(debugg){
						scloud->points[src_ind].x = tx;
						scloud->points[src_ind].y = ty;
						scloud->points[src_ind].z = tz;
						scloud->points[src_ind].r = 0;
						scloud->points[src_ind].g = 0;
						scloud->points[src_ind].b = 255;
					}
					float tnx	= m00*nx + m01*ny + m02*nz;
					float tny	= m10*nx + m11*ny + m12*nz;
					float tnz	= m20*nx + m21*ny + m22*nz;

					float itz	= 1.0/tz;
					float dst_w	= dst_fx*tx*itz + dst_cx;
					float dst_h	= dst_fy*ty*itz + dst_cy;

					if((dst_w > 0) && (dst_h > 0) && (dst_w < dst_width2) && (dst_h < dst_height2)){
						unsigned int dst_ind = unsigned(dst_h+0.5) * dst_width + unsigned(dst_w+0.5);

						float dst_z = dst_idepth*float(dst_depthdata[dst_ind]);
						if(dst_z > 0){
							float dst_x = (float(dst_w) - dst_cx) * dst_z * dst_ifx;
							float dst_y = (float(dst_h) - dst_cy) * dst_z * dst_ify;
							float diff_z2 = tnx*(dst_x-tx)+tny*(dst_y-ty)+tnz*(dst_z-tz);

							float diff_z = (dst_z-tz)/(z*z+dst_z*dst_z);//if tz < dst_z then tz infront and diff_z > 0
							if(diff_z < 0 && diff_z2 > 0){diff_z2 *= -1;}
							if(diff_z > 0 && diff_z2 < 0){diff_z2 *= -1;}
							diff_z2 /= (z*z+dst_z*dst_z);//if tz < dst_z then tz infront and diff_z > 0

							residuals.push_back(diff_z2);
							if(debugg){
								ws.push_back(src_w);
								hs.push_back(src_h);
							}
						}
					}
				}
			}
		}
	}

	if(debugg){
		for(unsigned int dst_w = 0; dst_w < dst_width; dst_w++){
			for(unsigned int dst_h = 0; dst_h < dst_height;dst_h++){
				int dst_ind = dst_h*dst_width+dst_w;
				if(dst_maskdata[dst_ind] == 255){// && p.z > 0 && !isnan(p.normal_x)){
					float z = dst_idepth*float(dst_depthdata[dst_ind]);
					if(z > 0){
						float x = (float(dst_w) - dst_cx) * z * dst_ifx;
						float y = (float(dst_h) - dst_cy) * z * dst_ify;
						dcloud->points[dst_ind].x = x;
						dcloud->points[dst_ind].y = y;
						dcloud->points[dst_ind].z = z;
						dcloud->points[dst_ind].r = 0;
						dcloud->points[dst_ind].g = 0;
						dcloud->points[dst_ind].b = 255;
					}
				}
			}
		}
	}

	for(unsigned int i = 0; i < residuals.size(); i++){
		float r = residuals[i];
		float weight = 1;
		if(fabs(r) > 0.0005){weight = 0;}//Replace with PPR

		float ocl = 0;
		if(r > 0){ocl += 1-weight;}

		oc.score		+= weight;
		oc.occlusions	+= ocl;

		if(debugg){
			int w = ws[i];
			int h = hs[i];
			unsigned int src_ind = h * src_width + w;
			if(ocl > 0 || weight > 0){
				scloud->points[src_ind].r = 255.0*ocl;
				scloud->points[src_ind].g = 255.0*weight;
				scloud->points[src_ind].b = 0;
			}
			debugg_img_data[3*src_ind+0] = 0;
			debugg_img_data[3*src_ind+1] = 255.0*weight;
			debugg_img_data[3*src_ind+2] = 255.0*ocl;
		}
	}

	if(debugg){
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
		cv::namedWindow("debuggimage",		cv::WINDOW_AUTOSIZE );
		cv::imshow(		"debuggimage",		debugg_img );
		//cv::waitKey(30);
		viewer->spin();
		viewer->removeAllPointClouds();
	}
	return oc;
}
*/
using namespace std;
using namespace Eigen;

vector<vector< OcclusionScore > > ModelUpdater::computeAllOcclusionScores(RGBDFrame * src, cv::Mat src_mask, RGBDFrame * dst, cv::Mat dst_mask,Eigen::Matrix4d p, bool debugg){
	unsigned char  * src_maskdata		= (unsigned char	*)(src_mask.data);
	unsigned char  * src_rgbdata		= (unsigned char	*)(src->rgb.data);
	unsigned short * src_depthdata		= (unsigned short	*)(src->depth.data);
	float		   * src_normalsdata	= (float			*)(src->normals.data);

	unsigned char  * dst_maskdata		= (unsigned char	*)(dst_mask.data);
	unsigned char  * dst_rgbdata		= (unsigned char	*)(dst->rgb.data);
	unsigned short * dst_depthdata		= (unsigned short	*)(dst->depth.data);
	float		   * dst_normalsdata	= (float			*)(dst->normals.data);

	float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
	float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
	float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

	Camera * src_camera				= src->camera;
	const unsigned int src_width	= src_camera->width;
	const unsigned int src_height	= src_camera->height;
	const float src_idepth			= src_camera->idepth_scale;
	const float src_cx				= src_camera->cx;
	const float src_cy				= src_camera->cy;
	const float src_ifx				= 1.0/src_camera->fx;
	const float src_ify				= 1.0/src_camera->fy;
	const int * src_labels			= src->labels;
	const int src_nr_labels			= src->nr_labels;

	Camera * dst_camera				= dst->camera;
	const unsigned int dst_width	= dst_camera->width;
	const unsigned int dst_height	= dst_camera->height;
	const float dst_idepth			= dst_camera->idepth_scale;
	const float dst_cx				= dst_camera->cx;
	const float dst_cy				= dst_camera->cy;
	const float dst_fx				= dst_camera->fx;
	const float dst_fy				= dst_camera->fy;
	const float dst_ifx				= 1.0/dst_camera->fx;
	const float dst_ify				= 1.0/dst_camera->fy;
	const unsigned int dst_width2	= dst_camera->width  - 2;
	const unsigned int dst_height2	= dst_camera->height - 2;
	const int * dst_labels			= dst->labels;
	const int dst_nr_labels			= src->nr_labels;

	vector< vector< OcclusionScore > > all_scores;
	all_scores.resize(src_nr_labels);
	for(int i = 0; i < src_nr_labels; i++){
		all_scores[i].resize(dst_nr_labels);
	}

	std::vector< std::vector< std::vector<float> > > all_residuals;
	all_residuals.resize(src_nr_labels);
	for(int i = 0; i < src_nr_labels; i++){
		all_residuals[i].resize(dst_nr_labels);
	}

	for(unsigned int src_w = 0; src_w < src_width; src_w++){
		for(unsigned int src_h = 0; src_h < src_height;src_h++){
			int src_ind = src_h*src_width+src_w;
			if(src_maskdata[src_ind] == 255){// && p.z > 0 && !isnan(p.normal_x)){
				float z = src_idepth*float(src_depthdata[src_ind]);
				float nx = src_normalsdata[3*src_ind+0];

				if(z > 0 && nx != 2){
					float ny = src_normalsdata[3*src_ind+1];
					float nz = src_normalsdata[3*src_ind+2];

					float x = (float(src_w) - src_cx) * z * src_ifx;
					float y = (float(src_h) - src_cy) * z * src_ify;

					float tx	= m00*x + m01*y + m02*z + m03;
					float ty	= m10*x + m11*y + m12*z + m13;
					float tz	= m20*x + m21*y + m22*z + m23;

					float tnx	= m00*nx + m01*ny + m02*nz;
					float tny	= m10*nx + m11*ny + m12*nz;
					float tnz	= m20*nx + m21*ny + m22*nz;

					float itz	= 1.0/tz;
					float dst_w	= dst_fx*tx*itz + dst_cx;
					float dst_h	= dst_fy*ty*itz + dst_cy;

					if((dst_w > 0) && (dst_h > 0) && (dst_w < dst_width2) && (dst_h < dst_height2)){
						unsigned int dst_ind = unsigned(dst_h+0.5) * dst_width + unsigned(dst_w+0.5);

						float dst_z = dst_idepth*float(dst_depthdata[dst_ind]);
						if(dst_z > 0){
							float dst_x = (float(dst_w) - dst_cx) * dst_z * dst_ifx;
							float dst_y = (float(dst_h) - dst_cy) * dst_z * dst_ify;
							float diff_z2 = tnx*(dst_x-tx)+tny*(dst_y-ty)+tnz*(dst_z-tz);

							float diff_z = (dst_z-tz)/(z*z+dst_z*dst_z);//if tz < dst_z then tz infront and diff_z > 0
							if(diff_z < 0 && diff_z2 > 0){diff_z2 *= -1;}
							if(diff_z > 0 && diff_z2 < 0){diff_z2 *= -1;}
							diff_z2 /= (z*z+dst_z*dst_z);//if tz < dst_z then tz infront and diff_z > 0

							int src_label = src_labels[src_ind];
							int dst_label = dst_labels[dst_ind];
							all_residuals[src_label][dst_label].push_back(diff_z2);
						}
					}
				}
			}
		}
	}

	DistanceWeightFunction2PPR * func = new DistanceWeightFunction2PPR();
	func->update_size = true;
    func->startreg = 0.00001;
	func->debugg_print = true;
	func->reset();


	delete func;

	for(int i = 0; i < src_nr_labels; i++){
		for(int j = 0; j < dst_nr_labels; i++){
			std::vector<float> & resi = all_residuals[i][j];
			OcclusionScore score;
			for(unsigned int k = 0; k < resi.size(); k++){
				float r = resi[k];
				float weight = 1;
				if(fabs(r) > 0.0005){weight = 0;}//Replace with PPR

				float ocl = 0;
				if(r > 0){ocl += 1-weight;}

				score.score			+= weight;
				score.occlusions	+= ocl;
			}
			all_scores[i][j] = score;
		}
	}
	return all_scores;
}


vector<vector < OcclusionScore > > ModelUpdater::getOcclusionScores(vector<Matrix4d> current_poses, vector<RGBDFrame*> current_frames,vector<cv::Mat> current_masks, bool debugg_scores){
	vector<vector < OcclusionScore > > scores;
	scores.resize(current_frames.size());
	for(int i = 0; i < current_frames.size(); i++){scores[i].resize(current_frames.size());}

	for(int i = 0; i < current_frames.size(); i++){
		for(int j = i+1; j < current_frames.size(); j++){
			Eigen::Matrix4d relative_pose = current_poses[i].inverse() * current_poses[j];
            scores[j][i]		= computeOcclusionScore(current_frames[j], current_masks[j], current_frames[i], current_masks[i],relative_pose,debugg_scores);
			scores[i][j]		= computeOcclusionScore(current_frames[i], current_masks[i], current_frames[j], current_masks[j],relative_pose.inverse(),debugg_scores);
		}
	}

	return scores;
}

/*
vector< vector< vector< vector< OcclusionScore > > > > ModelUpdater::getAllOcclusionScores(vector<Matrix4d> current_poses, vector<RGBDFrame*> current_frames,vector<cv::Mat> current_masks){
	vector< vector< vector< vector< OcclusionScore > > > > scores;
	scores.resize(current_frames.size());
	for(int i = 0; i < current_frames.size(); i++){scores[i].resize(current_frames.size());}
1
	bool debugg_scores = false;

	for(int i = 0; i < current_frames.size(); i++){
		for(int j = i+1; j < current_frames.size(); j++){
			Eigen::Matrix4d relative_pose = current_poses[i].inverse() * current_poses[j];
			scores[j][i]		= computeAllOcclusionScores(current_frames[j], current_masks[j], current_frames[i], current_masks[i],relative_pose,debugg_scores);
			scores[i][j]		= computeAllOcclusionScores(current_frames[i], current_masks[i], current_frames[j], current_masks[j],relative_pose.inverse(),debugg_scores);
		}
	}

	return scores;
}
*/

CloudData * ModelUpdater::getCD(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames,std::vector<cv::Mat> current_masks, int step){
	/*
	unsigned char  * maskdata		= (unsigned char	*)(mask.data);
	unsigned char  * rgbdata		= (unsigned char	*)(frame->rgb.data);
	unsigned short * depthdata		= (unsigned short	*)(frame->depth.data);
	float		   * normalsdata	= (float			*)(frame->normals.data);

	unsigned int frameid = frame->id;

	float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
	float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
	float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

	Camera * camera				= frame->camera;
	const unsigned int width	= camera->width;
	const unsigned int height	= camera->height;
	const float idepth			= camera->idepth_scale;
	const float cx				= camera->cx;
	const float cy				= camera->cy;
	const float ifx				= 1.0/camera->fx;
	const float ify				= 1.0/camera->fy;

	bool reprojected [width*height];
	for(unsigned int i = 0; i < width*height; i++){reprojected[i] = false;}

	for(unsigned int w = 0; w < width; w++){
		for(unsigned int h = 0; h < height;h++){
			int ind = h*width+w;
			if(maskdata[ind] == 255 && !reprojected[ind]){// && p.z > 0 && !isnan(p.normal_x)){
				float z = idepth*float(depthdata[ind]);
				float nx = normalsdata[3*ind+0];

				if(z > 0 && nx != 2){
					float ny = normalsdata[3*ind+1];
					float nz = normalsdata[3*ind+2];

					float x = (w - cx) * z * ifx;
					float y = (h - cy) * z * ify;

					float px	= m00*x + m01*y + m02*z + m03;
					float py	= m10*x + m11*y + m12*z + m13;
					float pz	= m20*x + m21*y + m22*z + m23;
					float pnx	= m00*nx + m01*ny + m02*nz;
					float pny	= m10*nx + m11*ny + m12*nz;
					float pnz	= m20*nx + m21*ny + m22*nz;

					float pb = rgbdata[3*ind+0];
					float pg = rgbdata[3*ind+1];
					float pr = rgbdata[3*ind+2];

					Vector3f	pxyz	(px	,py	,pz );
					Vector3f	pnxyz	(pnx,pny,pnz);
					Vector3f	prgb	(pr	,pg	,pb );
                    float	Villa_i_Sollentuna_65057392.htm?ca=11	weight	= 1.0/(z*z);
					points.push_back(superpoint(pxyz,pnxyz,prgb, weight, weight, frameid));
				}
			}
		}
	}
	*/
	return 0;
}

void ModelUpdater::computeMassRegistration(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames,std::vector<cv::Mat> current_masks){}

std::vector<std::vector < float > > ModelUpdater::getScores(std::vector<std::vector < OcclusionScore > > occlusionScores, float occlusion_penalty){
	std::vector<std::vector < float > > scores;
	scores.resize(occlusionScores.size());
	for(int i = 0; i < occlusionScores.size(); i++){scores[i].resize(occlusionScores.size());}

	for(int i = 0; i < scores.size(); i++){
		scores[i][i] = 0;
		for(int j = i+1; j < scores.size(); j++){
			scores[i][j] = occlusionScores[i][j].score+occlusionScores[j][i].score - occlusion_penalty*(occlusionScores[i][j].occlusions+occlusionScores[j][i].occlusions);
			scores[j][i] = scores[i][j];
		}
	}
/*
	printf("=======================================================\n");
	for(int i = 0; i < scores.size(); i++){
		for(int j = 0; j < scores.size(); j++){
			printf("%5.5f ",scores[i][j]);
		}
		printf("\n");
	}
	printf("=======================================================\n");
*/
	return scores;
}
/*
	int * partition = partition_aprox(scores,3,3,2);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	for(int i = 0; i < current_frames.size(); i++){
		int partid = partition[i];
		unsigned char  * maskdata		= (unsigned char	*)(current_masks[i].data);
		unsigned short * depthdata		= (unsigned short	*)(current_frames[i]->depth.data);
		Eigen::Matrix4d p = current_poses[i];

		float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
		float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
		float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

		Camera * camera				= current_frames[i]->camera;
		const unsigned int width	= camera->width;
		const unsigned int height	= camera->height;
		const float idepth			= camera->idepth_scale;
		const float cx				= camera->cx;
		const float cy				= camera->cy;
		const float ifx				= 1.0/camera->fx;
		const float ify				= 1.0/camera->fy;

		//dcloud->points.resize(width*height);
		for(unsigned int w = 0; w < width; w++){
			for(unsigned int h = 0; h < height;h++){
				int ind = h*width+w;
				if(maskdata[ind] == 255){// && p.z > 0 && !isnan(p.normal_x)){
					float z = idepth*float(depthdata[ind]);
					if(z > 0){
						float x = (float(w) - cx) * z * ifx;
						float y = (float(h) - cy) * z * ify;

						float tx	= m00*x + m01*y + m02*z + m03;
						float ty	= m10*x + m11*y + m12*z + m13;
						float tz	= m20*x + m21*y + m22*z + m23;

						pcl::PointXYZRGBNormal point;
						point.x = tx;
						point.y = ty;
						point.z = tz;
						if(partid == 0){
							point.r = 0;
							point.g = 255;
							point.b = 0;
						}else if(partid == 1){
							point.r = 255;
							point.g = 0;
							point.b = 0;
						}else if(partid == 2){
							point.r = 0;
							point.g = 0;
							point.b = 255;
						}else if(partid == 3){
							point.r = 255;
							point.g = 0;
							point.b = 255;
						}else if(partid == 4){
							point.r = 0;
							point.g = 255;
							point.b = 255;
						}else if(partid == 5){
							point.r = 255;
							point.g = 255;
							point.b = 0;
						}else if(partid == 6){
							point.r = 0;
							point.g = 0;
							point.b = 0;
						}
						dcloud->points.push_back(point);
					}
				}
			}
		}
	}

	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	viewer->spin();
	viewer->removeAllPointClouds();
	delete[] partition;
}

*/

}
