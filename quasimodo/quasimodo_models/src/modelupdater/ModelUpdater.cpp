#include "modelupdater/ModelUpdater.h"

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
}

std::vector<int> ModelUpdater::getPartition(std::vector< std::vector< float > > & scores, int dims, int nr_todo, double timelimit){
	return partition_graph(scores);
}

ModelUpdater::ModelUpdater(){
	occlusion_penalty = 10;
    massreg_timeout = 60;
    model = 0;
}

ModelUpdater::ModelUpdater(Model * model_){	model = model_;}
ModelUpdater::~ModelUpdater(){}

FusionResults ModelUpdater::registerModel(Model * model2, Eigen::Matrix4d guess, double uncertanity){return FusionResults();}

void ModelUpdater::fuse(Model * model2, Eigen::Matrix4d guess, double uncertanity){
	for(unsigned int i = 0; i < model2->frames.size();i++){
		model->frames.push_back(model2->frames[i]);
		//model->masks.push_back(model2->masks[i]);
		model->modelmasks.push_back(model2->modelmasks[i]);
		model->relativeposes.push_back(guess*model2->relativeposes[i]);
	}
}

UpdatedModels ModelUpdater::fuseData(FusionResults * f, Model * model1,Model * model2){return UpdatedModels();}

void ModelUpdater::refine(double reg,bool useFullMask){
	//return ;
	printf("void ModelUpdater::refine()\n");

	std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(model->relativeposes, model->frames,model->modelmasks,false);
	std::vector<std::vector < float > > scores = getScores(ocs);

	double sumscore_bef = 0;
    for(unsigned int i = 0; i < scores.size(); i++){
        for(unsigned int j = 0; j < scores.size(); j++){
			sumscore_bef += scores[i][j];
		}
	}

	MassFusionResults mfr;
	MassRegistrationPPR * massreg = new MassRegistrationPPR(reg);
    massreg->timeout = massreg_timeout;
	massreg->viewer = viewer;
	massreg->visualizationLvl = 0;
	massreg->maskstep = 4;//std::max(1,int(0.5+0.02*double(model->frames.size())));
	massreg->nomaskstep = std::max(1,int(0.5+1.0*double(model->frames.size())));

	printf("maskstep: %i nomaskstep: %i\n",massreg->maskstep,massreg->nomaskstep);

	//massreg->type = PointToPoint;
	if(useFullMask){
//        massreg->visualizationLvl = 0;
//		MassRegistrationPPRColor * massregC = new MassRegistrationPPRColor(reg);
//		massregC->viewer = viewer;
//		massregC->visualizationLvl = 1;
//		massregC->steps = 8;
//		massregC->stopval = 0.001;
//		massregC->setData(model->frames,model->modelmasks);
//		mfr = massregC->getTransforms(model->relativeposes);
//		model->relativeposes = mfr.poses;
//	exit(0);

		massreg->setData(model->frames,model->modelmasks);
		massreg->nomask = false;
//		massreg->visualizationLvl = 0;
		massreg->stopval = 0.001;
		massreg->steps = 10;

		mfr = massreg->getTransforms(model->relativeposes);
		//model->relativeposes = mfr.poses;
		//massreg->steps = 8;
		//mfr = massreg->getTransforms(model->relativeposes);
		//model->relativeposes = mfr.poses;
//		exit(0);

//		massreg->visualizationLvl = 2;
//		massreg->stopval = 0.002;
//        massreg->steps = 4;
//		mfr = massreg->getTransforms(model->relativeposes);
//		model->relativeposes = mfr.poses;

	}else{
printf("%s::%i\n",__FILE__,__LINE__);
		exit(0);

		massreg->nomask = true;
		massreg->visualizationLvl = 3;
		massreg->steps = 10;
		massreg->stopval = 0.001;
		//massreg->setData(model->frames,model->masks);
		massreg->setData(model->frames,model->modelmasks);
		mfr = massreg->getTransforms(model->relativeposes);
		//model->relativeposes = mfr.poses;

	}

	std::vector<std::vector < OcclusionScore > > ocs2 = getOcclusionScores(mfr.poses, model->frames,model->modelmasks,false);
	std::vector<std::vector < float > > scores2 = getScores(ocs2);

	double sumscore_aft = 0;
    for(unsigned int i = 0; i < scores2.size(); i++){
        for(unsigned int j = 0; j < scores2.size(); j++){
			sumscore_aft += scores2[i][j];
		}
	}
	if(sumscore_aft >= sumscore_bef){
		model->relativeposes = mfr.poses;
		model->scores = scores2;
		model->total_scores = sumscore_aft;
	}
}

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

void ModelUpdater::computeResiduals(std::vector<float> & residuals, std::vector<float> & weights,
                                    RGBDFrame * src, cv::Mat src_mask, ModelMask * src_modelmask,
                                    RGBDFrame * dst, cv::Mat dst_mask, ModelMask * dst_modelmask,
                                    Eigen::Matrix4d p, bool debugg){

    unsigned char  * src_maskdata		= (unsigned char	*)(src_modelmask->mask.data);
    unsigned char  * src_rgbdata		= (unsigned char	*)(src->rgb.data);
    unsigned short * src_depthdata		= (unsigned short	*)(src->depth.data);
    float		   * src_normalsdata	= (float			*)(src->normals.data);

    unsigned char  * dst_maskdata		= (unsigned char	*)(dst_modelmask->mask.data);
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

    const unsigned int dst_width2	= dst_camera->width  - 2;
    const unsigned int dst_height2	= dst_camera->height - 2;

    std::vector<int> & testw = src_modelmask->testw;
    std::vector<int> & testh = src_modelmask->testh;

    unsigned int test_nrdata = testw.size();
    for(unsigned int ind = 0; ind < test_nrdata;ind++){
        unsigned int src_w = testw[ind];
        unsigned int src_h = testh[ind];

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
                //float tnx	= m00*nx + m01*ny + m02*nz;
                //float tny	= m10*nx + m11*ny + m12*nz;
                float tnz	= m20*nx + m21*ny + m22*nz;

                float itz	= 1.0/tz;
                float dst_w	= dst_fx*tx*itz + dst_cx;
                float dst_h	= dst_fy*ty*itz + dst_cy;

                if((dst_w > 0) && (dst_h > 0) && (dst_w < dst_width2) && (dst_h < dst_height2)){
                    unsigned int dst_ind = unsigned(dst_h+0.5) * dst_width + unsigned(dst_w+0.5);

                    float dst_z = dst_idepth*float(dst_depthdata[dst_ind]);
                    if(dst_z > 0){
                        float diff_z = (dst_z-tz)/(z*z+dst_z*dst_z);//if tz < dst_z then tz infront and diff_z > 0
                        residuals.push_back(diff_z);
                        weights.push_back(1.0);
                    }
                }
            }
        }
    }
}

void ModelUpdater::recomputeScores(){
//	printf("recomputeScores\n");
	std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(model->relativeposes,model->frames,model->modelmasks,false);
	model->scores = getScores(ocs);

	model->total_scores = 0;
    for(unsigned int i = 0; i < model->scores.size(); i++){
        for(unsigned int j = 0; j < model->scores.size(); j++){
			model->total_scores += model->scores[i][j];
		}
	}
}

OcclusionScore ModelUpdater::computeOcclusionScore(RGBDFrame * src, ModelMask * src_modelmask, RGBDFrame * dst, ModelMask * dst_modelmask, Eigen::Matrix4d p, int step, bool debugg){
	OcclusionScore oc;

	unsigned char  * src_maskdata		= (unsigned char	*)(src_modelmask->mask.data);
	unsigned char  * src_rgbdata		= (unsigned char	*)(src->rgb.data);
	unsigned short * src_depthdata		= (unsigned short	*)(src->depth.data);
	float		   * src_normalsdata	= (float			*)(src->normals.data);

	unsigned char  * dst_maskdata		= (unsigned char	*)(dst_modelmask->mask.data);
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
	std::vector<float> weights;
	std::vector<int> ws;
	std::vector<int> hs;
	residuals.reserve(src_width*src_height);
	weights.reserve(src_width*src_height);
	if(debugg){
		ws.reserve(src_width*src_height);
		hs.reserve(src_width*src_height);
	}

	double sum = 0;
	double count = 0;

	std::vector<int> & testw = src_modelmask->testw;
	std::vector<int> & testh = src_modelmask->testh;

	unsigned int test_nrdata = testw.size();
	unsigned int indstep = step;
//	int indstep = std::max(1.0,double(test_nrdata)/double(step));
//	printf("indstep: %i\n",indstep);
//	for(unsigned int ind = 0; ind < test_nrdata;ind+=indstep){
//	//for(unsigned int src_w = 0; src_w < src_width-1; src_w++){
//	//    for(unsigned int src_h = 0; src_h < src_height;src_h++){
//		unsigned int src_w = testw[ind];
//		unsigned int src_h = testh[ind];

//		int src_ind0 = src_h*src_width+src_w-1;
//		int src_ind1 = src_h*src_width+src_w;
//		int src_ind2 = src_h*src_width+src_w+1;
//		if(src_maskdata[src_ind0] == 255 && src_maskdata[src_ind1] == 255 && src_maskdata[src_ind2] == 255){// && p.z > 0 && !isnan(p.normal_x)){
//			float z0 = src_idepth*float(src_depthdata[src_ind0]);
//			float z1 = src_idepth*float(src_depthdata[src_ind1]);
//			float z2 = src_idepth*float(src_depthdata[src_ind2]);

//			double diff0 = (z0-z1)/(z0*z0+z1*z1);
//			double diff1 = (z2-z1)/(z2*z2+z1*z1);
//			if( fabs(diff0) < fabs(diff1)){
//				if(diff0 != 0){
//					sum += diff0*diff0;
//					count++;
//				}
//			}else{
//				if(diff1 != 0){
//					sum += diff1*diff1;
//					count++;
//				}
//			}
//		}
//	}
//    double me = sqrt(sum/(count+1));
//    double pred = 1.3*sqrt(2)*me;
/*
    for(unsigned int ind = 0; ind < test_nrdata;ind++){
    //for(unsigned int src_w = 0; src_w < src_width-1; src_w++){
    //    for(unsigned int src_h = 0; src_h < src_height;src_h++){
        unsigned int src_w = testw[ind];
        unsigned int src_h = testh[ind];

        int src_ind = src_h*src_width+src_w;
        if(src_maskdata[src_ind] == 255){// && p.z > 0 && !isnan(p.normal_x)){
            float z = src_idepth*float(src_depthdata[src_ind]);
            float nx = src_normalsdata[3*src_ind+0];
            if(z > 0 && nx != 2){
            //if(z > 0){
                float x = (float(src_w) - src_cx) * z * src_ifx;
                float y = (float(src_h) - src_cy) * z * src_ify;

                float tx	= m00*x + m01*y + m02*z + m03;
                float ty	= m10*x + m11*y + m12*z + m13;
                float tz	= m20*x + m21*y + m22*z + m23;

                //float tnx	= m00*nx + m01*ny + m02*nz;
                //float tny	= m10*nx + m11*ny + m12*nz;
                float tnz	= m20*nx + m21*ny + m22*nz;

                float itz	= 1.0/tz;
                float dst_w	= dst_fx*tx*itz + dst_cx;
                float dst_h	= dst_fy*ty*itz + dst_cy;

                if((dst_w > 0) && (dst_h > 0) && (dst_w < dst_width2) && (dst_h < dst_height2)){
                    unsigned int dst_ind = unsigned(dst_h+0.5) * dst_width + unsigned(dst_w+0.5);
                    float dst_z = dst_idepth*float(dst_depthdata[dst_ind]);
                    if(dst_z > 0){
                        float diff_z = (dst_z-tz)/(z*z+dst_z*dst_z);//if tz < dst_z then tz infront and diff_z > 0
                        residuals.push_back(diff_z);
                        if(debugg){
                            ws.push_back(src_w);
                            hs.push_back(src_h);
                        }
                    }
                }
            }
        }
    }
*/
	for(unsigned int ind = 0; ind < test_nrdata;ind+=indstep){
		unsigned int src_w = testw[ind];
		unsigned int src_h = testh[ind];

		int src_ind = src_h*src_width+src_w;
        if(src_maskdata[src_ind] == 255){
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
				float itz	= 1.0/tz;
				float dst_w	= dst_fx*tx*itz + dst_cx;
				float dst_h	= dst_fy*ty*itz + dst_cy;

				if((dst_w > 0) && (dst_h > 0) && (dst_w < dst_width2) && (dst_h < dst_height2)){
					unsigned int dst_ind = unsigned(dst_h+0.5) * dst_width + unsigned(dst_w+0.5);

					float dst_z = dst_idepth*float(dst_depthdata[dst_ind]);
					if(dst_z > 0){
						float diff_z = (dst_z-tz)/(z*z+dst_z*dst_z);//if tz < dst_z then tz infront and diff_z > 0
						residuals.push_back(diff_z);

						float tnx	= m00*nx + m01*ny + m02*nz;
						float tny	= m10*nx + m11*ny + m12*nz;
						float tnz	= m20*nx + m21*ny + m22*nz;

						float dst_x = (float(dst_w) - dst_cx) * dst_z * dst_ifx;
						float dst_y = (float(dst_h) - dst_cy) * dst_z * dst_ify;
						float angle = (tnx*dst_x+tny*dst_y+tnz*dst_z)/sqrt(dst_x*dst_x + dst_y*dst_y + dst_z*dst_z);
						weights.push_back(1-angle);
						if(debugg){
							ws.push_back(src_w);
							hs.push_back(src_h);
						}
					}
				}
			}
		}
	}




//	DistanceWeightFunction2PPR2 * func = new DistanceWeightFunction2PPR2();
//	func->maxp			= 1.0;
//	func->update_size	= true;
//	func->zeromean      = true;
//	func->startreg		= 0.0001;
//	func->debugg_print	= debugg;
//	func->bidir			= true;
//	func->maxnoise      = pred;
//	func->reset();

	DistanceWeightFunction2 * func = new DistanceWeightFunction2();
	func->f = THRESHOLD;
	func->p = 0.005;

	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(1,residuals.size());
    for(unsigned int i = 0; i < residuals.size(); i++){X(0,i) = residuals[i];}
	func->computeModel(X);

	Eigen::VectorXd  W = func->getProbs(X);

	delete func;

	for(unsigned int i = 0; i < residuals.size(); i++){
		float r = residuals[i];
		float weight = W(i);
		float ocl = 0;
		if(r > 0){ocl += 1-weight;}
		oc.score		+= weight*weights.at(i);
		oc.occlusions	+= ocl*weights.at(i);
	}

	//debugg = true;
	if(debugg){
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		scloud->points.resize(src_width*src_height);
		dcloud->points.resize(dst_width*dst_height);
		for(unsigned int src_w = 0; src_w < src_width-1; src_w++){
			for(unsigned int src_h = 0; src_h < src_height;src_h++){
				int src_ind = src_h*src_width+src_w;
				if(src_maskdata[src_ind] != 255){continue;}
				float z = src_idepth*float(src_depthdata[src_ind]);
				if(z > 0){
					float x = (float(src_w) - src_cx) * z * src_ifx;
					float y = (float(src_h) - src_cy) * z * src_ify;
					float tx	= m00*x + m01*y + m02*z + m03;
					float ty	= m10*x + m11*y + m12*z + m13;
					float tz	= m20*x + m21*y + m22*z + m23;
					scloud->points[src_ind].x = tx;
					scloud->points[src_ind].y = ty;
					scloud->points[src_ind].z = tz+2;
					scloud->points[src_ind].r = 0;
					scloud->points[src_ind].g = 0;
					scloud->points[src_ind].b = 255;
				}
			}
		}
		for(unsigned int dst_w = 0; dst_w < dst_width; dst_w++){
			for(unsigned int dst_h = 0; dst_h < dst_height;dst_h++){
                unsigned int dst_ind = dst_h*dst_width+dst_w;
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
						if(dst_maskdata[dst_ind] == 255){
							dcloud->points[dst_ind].r = 255;
							dcloud->points[dst_ind].g = 000;
							dcloud->points[dst_ind].b = 255;
						}
					}
				}
			}
		}
		viewer->removeAllPointClouds();
		//printf("%i showing results\n",__LINE__);
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
		viewer->spin();
		viewer->removeAllPointClouds();

		for(unsigned int dst_w = 0; dst_w < dst_width; dst_w++){
			for(unsigned int dst_h = 0; dst_h < dst_height;dst_h++){
				int dst_ind = dst_h*dst_width+dst_w;
				dcloud->points[dst_ind].x = 0;
				dcloud->points[dst_ind].y = 0;
				dcloud->points[dst_ind].z = 0;
			}
		}


		for(unsigned int i = 0; i < residuals.size(); i++){
			float r = residuals[i];
			float weight = W(i);
			float ocl = 0;
			if(r > 0){ocl += 1-weight;}
			if(debugg){
				int w = ws[i];
				int h = hs[i];
				unsigned int src_ind = h * src_width + w;
				if(ocl > 0.01 || weight > 0.01){
					scloud->points[src_ind].r = 255.0*ocl;
					scloud->points[src_ind].g = 255.0*weight;
					scloud->points[src_ind].b = 255.0*0;
				}else{
					scloud->points[src_ind].x = 0;
					scloud->points[src_ind].y = 0;
					scloud->points[src_ind].z = 0;
				}
			}
		}

		viewer->removeAllPointClouds();
		//printf("%i showing results\n",__LINE__);
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
		viewer->spin();
		viewer->removeAllPointClouds();

		for(unsigned int src_w = 0; src_w < src_width-1; src_w++){
			for(unsigned int src_h = 0; src_h < src_height;src_h++){
				int src_ind = src_h*src_width+src_w;
				if(src_maskdata[src_ind] != 255){continue;}
				float z = src_idepth*float(src_depthdata[src_ind]);
				if(z > 0){
					float x = (float(src_w) - src_cx) * z * src_ifx;
					float y = (float(src_h) - src_cy) * z * src_ify;
					float tx	= m00*x + m01*y + m02*z + m03;
					float ty	= m10*x + m11*y + m12*z + m13;
					float tz	= m20*x + m21*y + m22*z + m23;
					scloud->points[src_ind].x = tx;
					scloud->points[src_ind].y = ty;
					scloud->points[src_ind].z = tz+2;
					scloud->points[src_ind].r = 0;
					scloud->points[src_ind].g = 0;
					scloud->points[src_ind].b = 255;
				}
			}
		}
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
						if(dst_maskdata[dst_ind] == 255){
							dcloud->points[dst_ind].r = 255;
							dcloud->points[dst_ind].g = 000;
							dcloud->points[dst_ind].b = 255;
						}
					}
				}
			}
		}
		for(unsigned int i = 0; i < residuals.size(); i++){
			float r = residuals[i];
			float weight = W(i);
			float ocl = 0;
			if(r > 0){ocl += 1-weight;}
			if(debugg){
				int w = ws[i];
				int h = hs[i];
				unsigned int src_ind = h * src_width + w;
				if(ocl > 0.01 || weight > 0.01){
					scloud->points[src_ind].r = 255.0*ocl;//*weights.at(i);
					scloud->points[src_ind].g = 255.0*weight;//*weights.at(i);
					scloud->points[src_ind].b = 0;
				}else{
					scloud->points[src_ind].x = 0;
					scloud->points[src_ind].y = 0;
					scloud->points[src_ind].z = 0;
				}
			}
		}

		viewer->removeAllPointClouds();

		//printf("%i showing results\n",__LINE__);
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
		viewer->spin();
		viewer->removeAllPointClouds();
	}
/*
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
	*/
	return oc;
}

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

vector<vector < OcclusionScore > > ModelUpdater::getOcclusionScores(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames, std::vector<ModelMask*> current_modelmasks, bool debugg_scores, double speedup){
	//printf("getOcclusionScores\n");

	long total_points = 0;
	for(unsigned int i = 0; i < current_frames.size(); i++){total_points+=current_modelmasks[i]->testw.size();}
	int step = std::max(long(1),long(speedup*total_points*long(current_frames.size()))/long(50000000));

//	printf("total_points: %i\n",total_points);
//	printf("current_frames.size(): %i\n",current_frames.size());
//	printf("ratio: %f\n",double(total_points*long(current_frames.size()))/double(50000000));
	printf("step: %i\n",step);

	vector<vector < OcclusionScore > > occlusionScores;
	occlusionScores.resize(current_frames.size());
    for(unsigned int i = 0; i < current_frames.size(); i++){occlusionScores[i].resize(current_frames.size());}

	int max_points = step;//100000.0/double(current_frames.size()*(current_frames.size()-1));
    //float occlusion_penalty = 10.0f;
	std::vector<std::vector < float > > scores;
	scores.resize(occlusionScores.size());
    for(unsigned int i = 0; i < occlusionScores.size(); i++){scores[i].resize(occlusionScores.size());}

	bool lock = true;
    for(unsigned int i = 0; i < current_frames.size(); i++){
		scores[i][i] = 0;
        for(unsigned int j = i+1; j < current_frames.size(); j++){
			//printf("scores %i %i\n",i,j);
			if(lock && current_modelmasks[j]->sweepid == current_modelmasks[i]->sweepid && current_modelmasks[j]->sweepid != -1){
				occlusionScores[i][j].score = 999999;
				occlusionScores[i][j].occlusions = 0;
				occlusionScores[j][i].score = 999999;
				occlusionScores[j][i].occlusions = 0;
			}else{
				Eigen::Matrix4d relative_pose = current_poses[i].inverse() * current_poses[j];
				occlusionScores[j][i]		= computeOcclusionScore(current_frames[j], current_modelmasks[j],current_frames[i], current_modelmasks[i], relative_pose,max_points,debugg_scores);
				occlusionScores[i][j]		= computeOcclusionScore(current_frames[i], current_modelmasks[i],current_frames[j], current_modelmasks[j], relative_pose.inverse(),max_points,debugg_scores);
				//printf("scores: %i %i -> occlusion_penalty: %f -> (%f %f) and (%f %f) -> %f \n",i,j,occlusion_penalty,occlusionScores[i][j].score,occlusionScores[i][j].occlusions,occlusionScores[j][i].score,occlusionScores[j][i].occlusions,occlusionScores[i][j].score+occlusionScores[j][i].score - occlusion_penalty*(occlusionScores[i][j].occlusions+occlusionScores[j][i].occlusions));
			}
			scores[i][j] = occlusionScores[i][j].score+occlusionScores[j][i].score - occlusion_penalty*(occlusionScores[i][j].occlusions+occlusionScores[j][i].occlusions);
			scores[j][i] = scores[i][j];
		}
	}
	return occlusionScores;
}

CloudData * ModelUpdater::getCD(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames,std::vector<cv::Mat> current_masks, int step){return 0;}

void ModelUpdater::computeMassRegistration(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames,std::vector<cv::Mat> current_masks){}

std::vector<std::vector < float > > ModelUpdater::getScores(std::vector<std::vector < OcclusionScore > > occlusionScores){//, float occlusion_penalty){
	std::vector<std::vector < float > > scores;
	scores.resize(occlusionScores.size());
    for(unsigned int i = 0; i < occlusionScores.size(); i++){scores[i].resize(occlusionScores.size());}
    for(unsigned int i = 0; i < scores.size(); i++){
		scores[i][i] = 0;
        for(unsigned int j = i+1; j < scores.size(); j++){
			scores[i][j] = occlusionScores[i][j].score+occlusionScores[j][i].score - occlusion_penalty*(occlusionScores[i][j].occlusions+occlusionScores[j][i].occlusions);
			scores[j][i] = scores[i][j];
		}
	}
	return scores;
}

}
