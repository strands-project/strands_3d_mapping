#include "ModelDatabaseRetrieval.h"

#include <dynamic_object_retrieval/dynamic_retrieval.h>
#include <dynamic_object_retrieval/visualize.h>
#include <object_3d_benchmark/benchmark_retrieval.h>

#include <dynamic_object_retrieval/extract_surfel_features.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

ModelDatabaseRetrieval::ModelDatabaseRetrieval(std::string vpath) : vt_features()
{
    // actually, maybe we should just add some features to the vocabulary so it's not empty, like load an old vocabulary?
    // good idea
	boost::filesystem::path vocabulary_path(vpath);
    dynamic_object_retrieval::load_vocabulary(vt, vocabulary_path);
    vt.set_min_match_depth(3);
    vt.compute_normalizing_constants();
    training_indices = vt.max_ind();
}

ModelDatabaseRetrieval::~ModelDatabaseRetrieval(){}

void ModelDatabaseRetrieval::add(reglib::Model * model){
	models.push_back(model);
	printf("number of models: %i\n",models.size());
}

bool ModelDatabaseRetrieval::remove(reglib::Model * model){
	for(unsigned int i = 0; i < models.size(); i++){
		if(models[i] == model){
			models[i] = models.back();
			models.pop_back();
			return true;
		}
	}
	return false;
}

std::vector<reglib::Model *> ModelDatabaseRetrieval::search(reglib::Model * model, int number_of_matches){
	std::vector<reglib::Model *> ret;
		for(unsigned int i = 0; i < models.size(); i++){
		if(models[i] != model){
			ret.push_back(models[i]);
		}
		if(ret.size() == number_of_matches){break;}
	}
	return ret;
}


//Add pointcloud to database, return index number in database, weight is the bias of the system to perfer this object when searching
int ModelDatabaseRetrieval::add(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, double weight)
{
    CloudT::Ptr points(new CloudT);
    NormalCloudT::Ptr normals(new NormalCloudT);
    for (const pcl::PointXYZRGBNormal& pn : cloud->points) {
        points->push_back(PointT());
        points->back().getVector3fMap() = pn.getVector3fMap();
        points->back().getRGBVector4i() = pn.getRGBVector4i();
        normals->push_back(NormalT());
        normals->back().getNormalVector3fMap() = pn.getNormalVector3fMap();
    }

    HistCloudT::Ptr features(new HistCloudT);
    CloudT::Ptr keypoints(new CloudT);
    dynamic_object_retrieval::compute_features(features, keypoints, points, normals, false, true);

    cout << "Got features!" << endl;

    vt_features.push_back(HistCloudT::Ptr(new HistCloudT(*features)));

    int old_index = vt.max_ind();

    cout << "Trying to append cloud" << endl;

    vector<int> indices(features->size(), old_index);

    cout << "Features cloud size: " << features->size() << endl;
    vt.append_cloud(features, indices, false);

    cout << "Got to append the cloud!" << endl;

    int new_index = vt.max_ind()-1;

    added_indices.insert(make_pair(new_index, added_indices.size()));

    cout << "Added cloud with index: " << new_index << endl;

    return added_indices.size()-1;
}

// return true if successfull
// return false if fail
bool ModelDatabaseRetrieval::remove(int index)
{
    // don't do this for now
    removed_indices.insert(index);

    return false;
}

//Find the number_of_matches closest matches in dabase to the pointcloud for index
std::vector<int> ModelDatabaseRetrieval::search(int index, int number_of_matches)
{
    using result_type = vocabulary_tree<HistT, 8>::result_type;
    vector<result_type> scores;

    cout << "Trying to search with index: " << index << endl;
    cout << "Number clouds added: " << vt_features.size() << endl;

    vt.query_vocabulary(scores, vt_features[index], 0);
    vector<int> rtn;

    cout << "Finished querying, got " << scores.size() << " results!" << endl;

    for (const result_type& r : scores) {
        if (rtn.size() >= number_of_matches) {
            break;
        }
        if (r.index < training_indices) {
            continue;
        }
        cout << "Accessing added cloud at vt index: " << r.index << endl;
        int return_index = added_indices[r.index];
        if (removed_indices.count(return_index) == 0) {
            rtn.push_back(return_index);
        }
    }

    cout << "Filtered out only the added cloud, got " << rtn.size() << " clouds!" << endl;

    return rtn;
}
