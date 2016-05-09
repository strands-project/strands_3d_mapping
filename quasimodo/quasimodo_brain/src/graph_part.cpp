#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <sys/time.h>

#include <emmintrin.h>

double getTime(){
    struct timeval start1;
    gettimeofday(&start1, NULL);
    return double(start1.tv_sec+(start1.tv_usec/1000000.0));
}

float score5(int * part,int nr_data, float * scores, int padding){
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
	
	
	   /*
   printf("scores\n");
   for(int i = 0; i < nr_data; i++){
        for(int j = 0; j < nr_data; j++){
            printf("%f ",scores[dataAndPadding*i+j]);
        }
        printf("\n");
    }

    printf("part:");for(int i = 0; i < nr_data; i++){printf("%i",part[i]);}printf("\n");
    printf("returnval: %f\n",2*ssum);
    */
    /*
    float sum = 0;
    for(int i = 0; i < nr_data; i++){
        int parti = part[i];
        int muli = (nr_data+padding)*i;
        for(int j = i+1; j < nr_data; j++){
            sum += scores[muli+j] * (parti == part[j]);// = rand()%1000 - 500;
        }
    }
    
    if(ssum != sum){
    	printf("WTF\n");
    	printf("simd: %f normal: %f\n",ssum,sum);
    	
    	sum = 0;
    	__m128 simd_sum2 {0,0,0,0};
		for(int i = 0; i < nr_data; i++){
		    int parti = part[i];
		    int muli = (dataAndPadding)*i;
		    printf("nomral muli: %i\n",muli);
		    for(int j = i+1; j < nr_data; j++){
		    	float add = scores[muli+j] * (parti == part[j]);
		    	//printf("normal: %i %i -> add: %f sum: %f \n",i,j,add,sum);
		    	printf("normal: %i %i -> %f\n",i,j,add);
		        sum += scores[muli+j] * (parti == part[j]);// = rand()%1000 - 500;        
		    }
		    
		    muli = (dataAndPadding*i)/4;
		    printf("simd muli: %i\n",muli);
		    __m128i simd_parti = _mm_set_epi32(parti,parti,parti,parti);
		    for(int j = (i+1)/4; j < dataAndPadding/4; j++){
		    	__m128 simd_add= _mm_and_ps(scores_ptr[muli+j],(__m128)_mm_cmpeq_epi32(simd_parti, part_ptr[j]));
		    	
		        simd_sum2 = _mm_add_ps(simd_sum2,_mm_and_ps(scores_ptr[muli+j],(__m128)_mm_cmpeq_epi32(simd_parti, part_ptr[j])));// = rand()%1000 - 500;

				float * simd_sum_vec2 = (float*)(&simd_sum2);
				
				float * simd_add_vec2 = (float*)(&simd_add);
				for(int k = 0; k < 4; k++){
					printf("simd:   %i %i -> %f\n",i,j*4+k,simd_add_vec2[k]);
				}
				//printf("add: %f\nadd: %f\nadd: %f\nadd: %f\n",simd_add_vec2[0],simd_add_vec2[1],simd_add_vec2[2],simd_add_vec2[3]);
				
				float add = 0;
				float ssum2 = simd_sum_vec2[0] + simd_sum_vec2[1] + simd_sum_vec2[2] + simd_sum_vec2[3];
		        //printf("simd:   %i %i -> add: %f sum: %f\n",i,j,add,ssum2);
		    }
		    
		    float * simd_sum_vec2 = (float*)(&simd_sum2);
			float ssum2 = simd_sum_vec2[0] + simd_sum_vec2[1] + simd_sum_vec2[2] + simd_sum_vec2[3];
		    
		    printf("%i sum: %f simd_sum: %f\n",i,sum,ssum2);
			if(sum != ssum2){break;}

		}
   	
    	exit(0);
    }

    return 2.0*sum;
    */
}

float score4(int * part,int nr_data, float * scores, int padding){
    float sum = 0;
    for(int i = 0; i < nr_data; i++){
        int parti = part[i];
        int muli = (nr_data+padding)*i;
        for(int j = i+1; j < nr_data; j++){
            sum += scores[muli+j] * (parti == part[j]);// = rand()%1000 - 500;
        }
    }
    return 2.0*sum;
}

float score3(int * part,int nr_data, float * scores, int padding){

    float sum = 0;
    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            sum += scores[(nr_data+padding)*i+j] * (part[i] == part[j]);// = rand()%1000 - 500;
        }
    }
    return 2.0*sum;
}

float score2(std::vector< int > part, std::vector< std::vector< float > > scores){
    int nr_data = scores.size();
    float sum = 0;
    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            sum += scores[i][j] * (part[i] == part[j]);// = rand()%1000 - 500;

        }
    }
    return 2.0*sum;
}

float score1(std::vector< int > part, std::vector< std::vector< float > > scores){
    int nr_data = scores.size();
    float sum = 0;
    for(int i = 0; i < nr_data; i++){
        for(int j = 0; j < nr_data; j++){
            sum += scores[i][j] * (part[i] == part[j]);// = rand()%1000 - 500;

        }
    }
    return sum;
}

float score0(std::vector< int > part, std::vector< std::vector< float > > scores){
    int nr_data = scores.size();
    float sum = 0;
    for(int i = 0; i < nr_data; i++){
        for(int j = 0; j < nr_data; j++){
        	if(part[i] == part[j]){
            	sum += scores[i][j];
            }
        }
    }
    return sum;
}



void build_part(std::vector< std::vector< int > > * get_all_part, std::vector< int > part, int todo){
    if(todo > 0){
        part.push_back(0);
        build_part(get_all_part, part, todo-1);
        part.back() = 1;
        build_part(get_all_part, part, todo-1);
        part.pop_back();
    }else{
        part.push_back(0);
        get_all_part->push_back(part);
        part.back() = 1;
        get_all_part->push_back(part);
        part.pop_back();
    }
}

float build_part_fast(int * part,int nr_data, float * scores, int padding, int todo){
	int indtoset = nr_data-todo-1;
	float a,b;
    if(todo > 0){
        part[indtoset] = 0;
        a = build_part_fast(part,nr_data,scores, padding, todo-1);
        part[indtoset] = 1;
        b = build_part_fast(part,nr_data,scores, padding, todo-1);
    }else{
        part[indtoset] = 0;
        a = score5(part,nr_data,scores, padding);
        part[indtoset] = 1;
        b = score5(part,nr_data,scores, padding);
    }
    return std::max(a,b);
}

float build_part_var_fast(int dims, float* best_score, int* best_part, int * var, int * part,int nr_data, float * scores, int padding, int todo){
	//int indtoset = nr_data-todo-1;
	int indtoset = var[todo];
	float best = -99999999999;
    if(todo > 0){
    	for(int i = 0; i < dims; i++){
		    part[indtoset] = i;
		    float current = build_part_var_fast(dims,best_score, best_part,var, part,nr_data,scores, padding, todo-1);
		    best = std::max(best,current);
        }
    }else{
    	for(int i = 0; i < dims; i++){
		    part[indtoset] = i;
        	float current = score5(part,nr_data,scores, padding);
        	best = std::max(best,current);
		    if(current > best_score[0]){
		    	best_score[0] = current;
		    	for(int i = 0; i < nr_data+padding; i++){best_part[i] = part[i];}
		    }
		}
    }
    return best;
}

void partition_aprox(std::vector< std::vector< float > > & scores, int dims = 2, int nr_todo = 5, double timelimit = 2){

    int nr_data = scores.size();
    int padding = 4-(nr_data-4*(nr_data/4));
        
    float* scorespadded;
	int dummy1 = posix_memalign((void**)&scorespadded, 16,  (nr_data+padding) *nr_data * sizeof(float));

    for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            scorespadded[(nr_data+padding)*i+j] = scores[i][j];
        }
    }
    
    
    int * var = new int[nr_data];
    for(int i = 0; i < nr_data; i++){var[i] = i;}
    
    int* partpadded;
	int dummy2 = posix_memalign((void**)&partpadded, 16,  (nr_data+padding) * sizeof(int));
    for(int i = 0; i < nr_data+padding; i++){partpadded[i] = 0;}
    
    float* best_score = new float[1]; 
    int* best_partpadded;
	int dummy3 = posix_memalign((void**)&best_partpadded, 16,  (nr_data+padding) * sizeof(int));
    for(int i = 0; i < nr_data+padding; i++){best_partpadded[i] = partpadded[i];}
    best_score[0] = score5(best_partpadded,nr_data,scorespadded, padding);
    
	double start = getTime();
    for(int it = 0; (getTime()-start) < timelimit; it++){
		for(int i = 0; i < nr_todo; i++){//Randomize order to select variables for changes
			int ind = rand()%nr_data;
			int tmp = var[ind];
			var[ind] = var[i];
			var[i] = tmp;
		}
		
		for(int i = 0; i < nr_data; i++){partpadded[i] = best_partpadded[i];}
		float best = build_part_var_fast(dims,best_score, best_partpadded, var,partpadded,nr_data, scorespadded,padding, nr_todo);
	}
	printf("best: %f ",best_score[0]/float(nr_data*nr_data));
    delete[] best_score;
    delete[] best_partpadded;
    delete[] var;
	delete[] partpadded;
	delete[] scorespadded;
}

void partition_brute8(std::vector< std::vector< float > > & scores){

    int nr_data = scores.size();
    int padding = 4-(nr_data-4*(nr_data/4));
    
    float* scorespadded;
	int dummy1 = posix_memalign((void**)&scorespadded, 16,  (nr_data+padding) *nr_data * sizeof(float));

    for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            scorespadded[(nr_data+padding)*i+j] = scores[i][j];
        }
    }
    
    std::vector<float> res;
    res.resize(8);
    #pragma omp parallel for num_threads(8)
	for(int i = 0; i < 8; i++){
		int* partpadded;
		int dummy2 = posix_memalign((void**)&partpadded, 16,  (nr_data+padding) * sizeof(int));
		for(int i = 0; i < nr_data+padding; i++){partpadded[i] = 0;}
		
		partpadded[0] = 0;
		partpadded[1] = (i&(1<<2)) > 0;
		partpadded[2] = (i&(1<<1)) > 0;
		partpadded[3] = (i&(1<<0)) > 0;
		
		//printf("%i %i %i %i\n",0,(i&(1<<2)),(i&(1<<1)),(i&(1<<0)));
		float best = build_part_fast(partpadded,nr_data, scorespadded,padding, nr_data-2-3);
		res[i] = best;
		//printf("%i%i%i%i best: %f\n",partpadded[0],partpadded[1],partpadded[2],partpadded[3],best);
		
		delete[] partpadded;
	}
	delete[] scorespadded;
	
	float best = -99999999999;
	for(unsigned int i = 0; i < res.size(); i++){
		best = std::max(best,res[i]);
	}
	printf("best: %f ",best/float(nr_data*nr_data));
}

void partition_brute7(std::vector< std::vector< float > > & scores){
/*
    int nr_data = scores.size();
    int padding = (1+(nr_data/4))-nr_data;
    float* scorespadded;
	int dummy123 = posix_memalign((void**)&scorespadded, 16,  (nr_data+padding) *nr_data * sizeof(float));
    int* partpadded;
	int dummy123 = posix_memalign((void**)&partpadded, 16,  (nr_data+padding) * sizeof(int));

    for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
    //int c = 0;
    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            scorespadded[(nr_data+padding)*i+j] = scores[i][j];
        }
    }
    
    for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            scorespadded[(nr_data+padding)*i+j] = scores[i][j];
        }
    }
*/

    int nr_data = scores.size();
    int padding = 4-(nr_data-4*(nr_data/4));
    
    float* scorespadded;
	int dummy1 = posix_memalign((void**)&scorespadded, 16,  (nr_data+padding) *nr_data * sizeof(float));

    for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
    //int c = 0;
    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            scorespadded[(nr_data+padding)*i+j] = scores[i][j];
        }
    }

    int* partpadded;
	int dummy2 = posix_memalign((void**)&partpadded, 16,  (nr_data+padding) * sizeof(int));
    for(int i = 0; i < nr_data+padding; i++){partpadded[i] = 0;}
    
    float best = build_part_fast(partpadded,nr_data, scorespadded,padding, nr_data-2);
	 printf("best: %f ",best/float(nr_data*nr_data));
}

void partition_brute6(std::vector< std::vector< float > > & scores){
    int nr_data = scores.size();
    //int padding = (1+(nr_data/4))-nr_data;//int padding = nr_data % 4;
    //int div4 = nr_data/4;
    //int diff = nr_data-4*div4;
    int padding = 4-(nr_data-4*(nr_data/4));
    

    float* scorespadded;
	int dummy1 = posix_memalign((void**)&scorespadded, 16,  (nr_data+padding) *nr_data * sizeof(float));
    int* partpadded;
	int dummy2 = posix_memalign((void**)&partpadded, 16,  (nr_data+padding) * sizeof(int));

    for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
    int c = 0;
    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            scorespadded[(nr_data+padding)*i+j] = scores[i][j];
        }
    }

    std::vector< std::vector< int > > * get_all_part = new std::vector< std::vector< int > >();
    std::vector< int > part;
    float best = -999999999999999999;
    part.push_back(0);

    double start = getTime();
    build_part(get_all_part, part, nr_data-2);
    //printf("build_part: %10.10f\n",getTime()-start);
	for(unsigned int i = 0; i < get_all_part->size(); i++){
        std::vector< int > & parttmp = get_all_part->at(i);
		for(unsigned int j = 0; j < get_all_part->at(i).size(); j++){partpadded[j] = parttmp[j];}
        float score = score5(partpadded,nr_data, scorespadded, padding);
        if(score > best){
            best = score;
            part = parttmp;
        }
    }
    printf("best: %f ",best/float(nr_data*nr_data));
}

void partition_brute5(std::vector< std::vector< float > > & scores){
    int nr_data = scores.size();
    int padding = nr_data % 4;
    float * scorespadded = new float[(nr_data+padding)*nr_data];
    for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
    int * partpadded = new int[nr_data+padding];

    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            scorespadded[(nr_data+padding)*i+j] = scores[i][j];// = rand()%1000 - 500;
        }
    }

    std::vector< std::vector< int > > * get_all_part = new std::vector< std::vector< int > >();
    std::vector< int > part;
    float best = -999999999999999999;
    part.push_back(0);

    double start = getTime();
    build_part(get_all_part, part, nr_data-2);
    //printf("build_part: %10.10f\n",getTime()-start);
	for(unsigned int i = 0; i < get_all_part->size(); i++){
        std::vector< int > & parttmp = get_all_part->at(i);
		for(unsigned int j = 0; j < get_all_part->at(i).size(); j++){
            partpadded[j] = parttmp[j];
        }

        float score = score4(partpadded,nr_data, scorespadded, padding);
        if(score > best){
            best = score;
            part = parttmp;
        }
    }
    printf("best: %f ",best/float(nr_data*nr_data));
}

void partition_brute4(std::vector< std::vector< float > > & scores){
    int nr_data = scores.size();
    int padding = nr_data % 4;
    float * scorespadded = new float[(nr_data+padding)*nr_data];
    for(int i = 0; i < (nr_data+padding)*nr_data; i++){scorespadded[i] = 0;}
    int * partpadded = new int[nr_data+padding];

    for(int i = 0; i < nr_data; i++){
        for(int j = i+1; j < nr_data; j++){
            scorespadded[(nr_data+padding)*i+j] = scores[i][j];// = rand()%1000 - 500;
        }
    }

    std::vector< std::vector< int > > * get_all_part = new std::vector< std::vector< int > >();
    std::vector< int > part;
    float best = -999999999999999999;
    part.push_back(0);

    double start = getTime();
    build_part(get_all_part, part, nr_data-2);
    //printf("build_part: %10.10f\n",getTime()-start);
	for(unsigned int i = 0; i < get_all_part->size(); i++){
        std::vector< int > & parttmp = get_all_part->at(i);
		for(unsigned int j = 0; j < get_all_part->at(i).size(); j++){
            partpadded[j] = parttmp[j];
        }

        float score = score3(partpadded,nr_data, scorespadded, padding);
        if(score > best){
            best = score;
            part = parttmp;
        }
    }
    printf("best: %f ",best/float(nr_data*nr_data));
}

void partition_brute3(std::vector< std::vector< float > > & scores){
    int nr_data = scores.size();
    std::vector< std::vector< int > > * get_all_part = new std::vector< std::vector< int > >();
    std::vector< int > part;
    float best = -999999999999999999;
    part.push_back(0);

    double start = getTime();
    build_part(get_all_part, part, nr_data-2);
    //printf("build_part: %10.10f\n",getTime()-start);
	for(unsigned int i = 0; i < get_all_part->size(); i++){
        float score = score2(get_all_part->at(i),scores);
        if(score > best){
            best = score;
            part = get_all_part->at(i);
        }
    }
    printf("best: %f ",best/float(nr_data*nr_data));
}

void partition_brute2(std::vector< std::vector< float > > & scores){
    int nr_data = scores.size();
    std::vector< std::vector< int > > * get_all_part = new std::vector< std::vector< int > >();
    std::vector< int > part;
    float best = -999999999999999999;
    part.push_back(0);
    double start = getTime();
    build_part(get_all_part, part, nr_data-2);
    //printf("build_part: %10.10f\n",getTime()-start);
	for(unsigned int i = 0; i < get_all_part->size(); i++){
        float score = score1(get_all_part->at(i),scores);
        if(score > best){
            best = score;
            part = get_all_part->at(i);
        }
    }
    printf("best: %f ",best/float(nr_data*nr_data));
}

void partition_brute1(std::vector< std::vector< float > > & scores){
    int nr_data = scores.size();
    std::vector< std::vector< int > > * get_all_part = new std::vector< std::vector< int > >();
    std::vector< int > part;
    float best = -999999999999999999;

    double start = getTime();
    build_part(get_all_part, part, nr_data-1);
    //printf("build_part: %10.10f\n",getTime()-start);

	for(unsigned int i = 0; i < get_all_part->size(); i++){
        float score = score1(get_all_part->at(i),scores);
        if(score > best){
            best = score;
            part = get_all_part->at(i);
        }
    }
    printf("best: %f ",best/float(nr_data*nr_data));
}

void partition_brute0(std::vector< std::vector< float > > & scores){
    int nr_data = scores.size();
    std::vector< std::vector< int > > * get_all_part = new std::vector< std::vector< int > >();
    std::vector< int > part;
    float best = -999999999999999999;

    double start = getTime();
    build_part(get_all_part, part, nr_data-1);
	for(unsigned int i = 0; i < get_all_part->size(); i++){
        float score = score0(get_all_part->at(i),scores);
        if(score > best){
            best = score;
            part = get_all_part->at(i);
        }
    }
    printf("best: %f ",best/float(nr_data*nr_data));
}

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

void partition_graph(std::vector< std::vector< float > > & scores){
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

	int * part = new int[nr_data];
	for(unsigned int i = 0; i < graphinds_out->size(); i++){
		for(unsigned int j = 0; j < graphinds_out->at(i).size(); j++){
			part[graphinds_out->at(i).at(j)] = i;
		}
	}
/*
	printf("best: %f\n",best/float(nr_data*nr_data));
	for(unsigned int i = 0; i < graphinds_out->size(); i++){
		printf("%i -> ",i);
		for(unsigned int j = 0; j < graphinds_out->at(i).size(); j++){printf("%i ",graphinds_out->at(i).at(j));}
		printf("\n");
	}
	for(int i = 0; i < nr_data; i++){printf("%i ",part[i]);}printf("\n");
*/
}

int main(int argc, char **argv)
{
	printf("starting partition software\n");
	double start;
	std::vector< std::vector< float > > scores;
    //for(int nr_data = 5; nr_data <= 25; nr_data += 2){
	int gr = 30;
	int nr_data = gr*gr;
	scores.resize(nr_data);
	for(int i = 0; i < nr_data; i++){scores[i].resize(nr_data);}
	/*
	for(int i = 0; i < nr_data; i++){
		for(int j = i+1; j < nr_data; j++){
			//if(((rand() % 1) == 0)){
			if(((rand() % 1) == 0)){
				scores[i][j] = rand()%1000 - 500;
			}else{
				scores[i][j] = 0;
			}
			scores[j][i] = scores[i][j];
		}
	}
	*/

	for(int i = 0; i < gr-1; i++){
		for(int j = 0; j < gr-1; j++){
			int ind = i*gr+j;
			scores[ind][ind+1] = rand()%1000 - 500;
			scores[ind+1][ind] = scores[ind][ind+1];

			scores[ind][ind+gr] = rand()%1000 - 500;
			scores[ind+gr][ind] = scores[ind][ind+gr];
		}
	}

	float sum = 0;
	for(int i = 0; i < nr_data; i++){
		for(int j = i+1; j < nr_data; j++){
			float weight = scores[i][j];
			sum += 2*weight;
		}
	}
	printf("best no part: %f\n",sum/float(nr_data*nr_data));


	start = getTime();
	partition_graph(scores);
	printf("partition_brute9: %10.10f\n",getTime()-start);
/*
		//for(int i = 4; i < 6; i++){
			for(double t = 0.01; t <= 800.0; t*=1.3){
				start = getTime();
				partition_aprox(scores,5,7,t);
				printf("partition_aprox: %10.10f\n",getTime()-start);
			}
		//}
*/
		start = getTime();
		partition_brute8(scores);
		printf("partition_brute8: %10.10f\n",getTime()-start);
exit(0);
        start = getTime();
        partition_brute7(scores);
        printf("partition_brute7: %10.10f\n",getTime()-start);

        start = getTime();
        partition_brute6(scores);
        printf("partition_brute6: %10.10f\n",getTime()-start);

        start = getTime();
        partition_brute5(scores);
        printf("partition_brute5: %10.10f\n",getTime()-start);

        start = getTime();
        partition_brute4(scores);
        printf("partition_brute4: %10.10f\n",getTime()-start);

        start = getTime();
        partition_brute3(scores);
        printf("partition_brute3: %10.10f\n",getTime()-start);

        start = getTime();
        partition_brute2(scores);
        printf("partition_brute2: %10.10f\n",getTime()-start);

        start = getTime();
        partition_brute1(scores);
        printf("partition_brute1: %10.10f\n",getTime()-start);

        start = getTime();
        partition_brute0(scores);
        printf("partition_brute0: %10.10f\n",getTime()-start);

	return 0;
}
