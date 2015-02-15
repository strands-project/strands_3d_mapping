#include <iostream>
#include <fstream>
#include <stdint.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/vector.hpp>

#include <Eigen/Dense>

#include "object_3d_retrieval/object_retrieval.h"

using namespace std;

using PointT = object_retrieval::PointT;
using CloudT = object_retrieval::CloudT;
using HistT = object_retrieval::HistT;
using HistCloudT = object_retrieval::HistCloudT;
using NormalT = object_retrieval::NormalT;
using NormalCloudT = object_retrieval::NormalCloudT;

using index_score = object_retrieval::index_score;

struct annotation {
    string category;
    int instance;
    int top;
    int bottom;
    int left;
    int right;
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(category, instance, top, bottom, left, right);
    }
};

struct bboxes {
    // all of the annotations for one scene type (e.g. desk_1) then all for one image
    vector<vector<annotation> > values;
    void read_json(const string& file)
    {
        //ifstream in("/home/nbore/Data/rgbd-scenes/desk/try.txt", ios::binary);
        ifstream in(file);
        //cereal::BinaryInputArchive archive_i(in);
        cereal::JSONInputArchive archive_i(in);
        archive_i(values); // Is reading of the K matrix slowing this down?
    }
};

void read_annotations(map<string, bboxes>& annotations)
{
    boost::filesystem::path p1 = "/home/nbore/Data/rgbd-scenes/";
    typedef vector<boost::filesystem::path> vec;
    vec v1; // so we can sort them later
    copy(boost::filesystem::directory_iterator(p1), boost::filesystem::directory_iterator(), back_inserter(v1));
    sort(v1.begin(), v1.end()); // sort, since directory iteration
    cout << "Opening path " << p1.string() << endl;
    for (const boost::filesystem::path& p2 : v1) {
        if (!boost::filesystem::is_directory(p2) || p2.stem().string() == "background") {
            continue;
        }
        vec v2; // so we can sort them later
        copy(boost::filesystem::directory_iterator(p2), boost::filesystem::directory_iterator(), back_inserter(v2));
        sort(v2.begin(), v2.end()); // sort, since directory iteration
        cout << "Opening path " << p2.string() << endl;
        for (const boost::filesystem::path& p3 : v2) {
            if (!boost::filesystem::is_regular_file(p3) || p3.extension().string() != ".json") {
                continue;
            }
            cout << "Reading file " << p3.stem().string() << endl;
            bboxes bb;
            bb.read_json(p3.string());
            annotations.insert(make_pair(p3.stem().string(), bb));
        }

    }
}

void print_annotations(bboxes& bb)
{
    for (vector<annotation>& v1 : bb.values) {
        cout << "Reading vector" << endl;
        for (annotation& a : v1) {
            cout << "category: " << a.category << endl;
            cout << "instance: " << a.instance << endl;
            cout << "top: " << a.top << endl;
            cout << "bottom: " << a.bottom << endl;
            cout << "left: " << a.left << endl;
            cout << "right: " << a.right << endl;
        }
    }
}

void get_test_segments(vector<size_t>& indices)
{
    for (size_t i = 0; i < 12000; i += 50) {
        indices.push_back(i);
    }
}

void scene_annotations_for_metadata(vector<annotation>& rtn, map<string, bboxes> annotations, const string& metadata)
{
    boost::filesystem::path path(metadata);
    string name = path.stem().string();
    string folder = path.parent_path().stem().string();
    cout << folder << endl;
    bboxes& bb = annotations.at(folder);
    vector<string> strs;
    boost::split(strs, name, boost::is_any_of("_"));
    int ind = stoi(strs.back())-1;
    cout << "ind: " << ind << endl;
    rtn = bb.values[ind];
}

void get_object_instance(string& object, int& instance, vector<annotation>& scene_annotations, CloudT::Ptr& cloud, const Eigen::Matrix3f& K)
{
    for (annotation& a : scene_annotations) {
        // if cloud is sufficiently within bounding box, return object instance
        size_t counter = 0;
        for (PointT& p : cloud->points) {
            Eigen::Vector3f pe = p.getVector3fMap();
            pe = K*pe;
            pe = pe/pe(2);
            if (pe(0) < a.right && pe(0) > a.left && pe(1) < a.bottom && pe(1) > a.top) {
                ++counter;
            }
        }
        if (float(counter)/float(cloud->size()) > 0.75) {
            object = a.category;
            instance = a.instance;
            return;
        }
    }
    object = "";
    instance = 0;
}

size_t count_hits(string cobject, int cinstance, vector<CloudT::Ptr>& segments, vector<string>& metadatas,
                  Eigen::Matrix3f& K, map<string, bboxes>& annotations)
{
    size_t hits = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        vector<annotation> scene_annotations;
        scene_annotations_for_metadata(scene_annotations, annotations, metadatas[i]);
        string object;
        int instance;
        get_object_instance(object, instance, scene_annotations, segments[i], K);
        if (object == cobject && instance == cinstance) {
            ++hits;
        }
        cout << "Comparing to " << object << " instance " << instance << endl;
    }
    return hits;
}

void get_all_annotations(map<pair<string, int> , vector<int> >& instance_segments,
                         map<string, bboxes>& annotations, const string& segments_path,
                         object_retrieval& obr)
{
    boost::filesystem::path path(segments_path);
    boost::filesystem::path annotations_file = path.parent_path() / "annotations.cereal";

    if (boost::filesystem::is_regular_file(annotations_file)) {
        ifstream in(annotations_file.string(), std::ios::binary);
        cereal::BinaryInputArchive archive_i(in);
        archive_i(instance_segments);
        return;
    }

    for (size_t i = 0; ; ++i) {
        CloudT::Ptr segment(new CloudT);
        NormalCloudT::Ptr normal(new NormalCloudT);
        CloudT::Ptr hd_segment(new CloudT);
        string metadata;
        Eigen::Matrix3f K;
        if (!obr.read_segment(segment, normal, hd_segment, K, metadata, i)) {
            break;
        }
        vector<annotation> scene_annotations;
        scene_annotations_for_metadata(scene_annotations, annotations, metadata);
        string object;
        int instance;
        get_object_instance(object, instance, scene_annotations, segment, K);
        if (instance == 0) {
            continue;
        }
        pair<string, int> instance_id = make_pair(object, instance);
        if (instance_segments.count(instance_id) == 0) {
            instance_segments.insert(make_pair(instance_id, vector<int>({ i })));
        }
        else {
            instance_segments.at(instance_id).push_back(i);
        }
    }

    {
        ofstream out(annotations_file.string(), std::ios::binary);
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(instance_segments);
    }
}

vector<size_t> sample_without_replacement(size_t upper, size_t number)
{
    random_device device;
    mt19937 generator(device());

    // Generate a vector with all possible numbers and shuffle it.
    vector<size_t> result;
    for (size_t i = 0; i < upper; ++i) {
        result.push_back(i);
    }
    shuffle(result.begin(), result.end(), generator);

    // Truncate to the requested size.
    result.resize(number);

    return result;
}

void calculate_exclude_set(set<int>& exclude_set, map<pair<string, int>, vector<int> >& annotations, int exclude_number)
{
    for (pair<const pair<string, int>, vector<int> >& instance_set : annotations) {
        vector<size_t> sampled = sample_without_replacement(instance_set.second.size(), exclude_number);
        for (size_t i : sampled) {
            exclude_set.insert(instance_set.second[i]);
        }
    }
}

void read_segments_from_scores(vector<CloudT::Ptr>& segments, vector<string>& metadatas, vector<index_score>& scores, object_retrieval& obr)
{
    for (index_score& i : scores) {
        CloudT::Ptr segment(new CloudT);
        NormalCloudT::Ptr normal(new NormalCloudT);
        CloudT::Ptr hd_segment(new CloudT);
        string metadata;
        Eigen::Matrix3f K;
        obr.read_segment(segment, normal, hd_segment, K, metadata, i.first);
        segments.push_back(hd_segment);
        metadatas.push_back(metadata);
    }
}

size_t instances_in_annotations(string object, int instance, map<string, bboxes>& annotations)
{
    size_t counter = 0;
    for (pair<const string, bboxes>& bb : annotations) {
        for (vector<annotation>& v : bb.second.values) {
            for (annotation& a : v) {
                if (a.category == object && a.instance == instance) {
                    ++counter;
                }
            }
        }
    }
    return counter;
}

void compute_precision_recall(map<pair<string, int>, vector<int> >& annotations,
                              map<pair<string, int>, int>& matched_instances, int nbr_query, int exclude_number)
{
    // also, remember that exclude_number of every instance is not in the data set

    for (pair<const pair<string, int>, vector<int> >& a : annotations) {
        pair<string, int> instance = a.first;
        int number_in_dataset = a.second.size();

        // first, compute the number of true positives
        // tp simply the number of correctly matched instances, matched_instances
        int tp = matched_instances.at(instance);

        // then, the number of false positives
        // fp simply the rest of the query vector i.e. nbr_query*exclude_number - matched_instances
        int fp = nbr_query*exclude_number - tp;

        // then, the number of true negatives
        // all_examples - all_of_instance
        int tn = number_in_dataset - exclude_number;

        // then, the number of false negatives
        // all_of_instance - matched_instances
        //int fn = 0;

        // generate matlab code
    }
}

int main(int argc, char** argv)
{
    map<string, bboxes> annotations;
    read_annotations(annotations);

    string segments_path = "/home/nbore/Data/rgbd-scenes/object_segments";
    object_retrieval obr(segments_path);

    map<pair<string, int>, vector<int> > instance_segments;
    get_all_annotations(instance_segments, annotations, segments_path, obr);

    set<int> exclude_set;
    int exclude_number = 20; // the number of examples of each instance to query
    calculate_exclude_set(exclude_set, instance_segments, exclude_number);
    obr.set_exclude_set(exclude_set);

    obr.train_vocabulary_incremental(5000, false);

    int nbr_query = 10;
    map<pair<string, int>, int> first_matched_instances;
    map<pair<string, int>, int> reweight_matched_instances;
    for (const pair<pair<string, int>, vector<int> >& i : instance_segments) {
        if (i.first.second == 0) {
            cout << "WTF? background in here" << endl;
            exit(0);
        }
        first_matched_instances.insert(make_pair(i.first, 0));
        reweight_matched_instances.insert(make_pair(i.first, 0));
    }

    for (int i : exclude_set) {
        CloudT::Ptr segment(new CloudT);
        NormalCloudT::Ptr normal(new NormalCloudT);
        CloudT::Ptr hd_segment(new CloudT);
        Eigen::Matrix3f K;
        string metadata;
        vector<annotation> scene_annotations;
        // read in all information for segment
        obr.read_segment(segment, normal, hd_segment, K, metadata, i);

        // get the annotations corresponding to this scene
        scene_annotations_for_metadata(scene_annotations, annotations, metadata);

        // determine if segment correspondes to any object in scene
        pair<string, int> instance;
        get_object_instance(instance.first, instance.second, scene_annotations, segment, K);
        if (instance.second == 0) {
            cout << "WTF? background in here" << endl;
            exit(0);
        }

        vector<index_score> first_scores;
        vector<index_score> reweight_scores;
        obr.query_reweight_vocabulary(first_scores, reweight_scores, i, nbr_query); // query as many as there are instances
        vector<CloudT::Ptr> first_segments;
        vector<string> first_metadatas;
        read_segments_from_scores(first_segments, first_metadatas, first_scores, obr);
        vector<CloudT::Ptr> reweight_segments;
        vector<string> reweight_metadatas;
        read_segments_from_scores(reweight_segments, reweight_metadatas, reweight_scores, obr);

        int first_hits = count_hits(instance.first, instance.second, first_segments, first_metadatas, K, annotations);
        int reweight_hits = count_hits(instance.first, instance.second, reweight_segments, reweight_metadatas, K, annotations);
        first_matched_instances.at(instance) += first_hits;
        reweight_matched_instances.at(instance) += reweight_hits;
    }

    for (pair<const pair<string, int>, int> m : first_matched_instances) {
        float first_ratio = float(m.second)/float(nbr_query*exclude_number);
        float reweight_ratio = float(reweight_matched_instances.at(m.first))/float(nbr_query*exclude_number);
        cout << m.first.first << "_" << m.first.second << " = [ " << first_ratio << ", " << reweight_ratio << " ];" << endl;
    }

    return 0;
}
