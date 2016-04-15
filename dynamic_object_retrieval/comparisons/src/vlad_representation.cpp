#include <vlad/vlad_representation.h>

// VLAD PFHRGB feature struct (numCenters * 250)
POINT_CLOUD_REGISTER_POINT_STRUCT (VladT,
                                   (float[V], histogram, histogram)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

using namespace std;

namespace vlad_representation {

void build_vlad_representation(const boost::filesystem::path& data_path,
                               vl_size numData, vlad_repr& repr)
{
    dynamic_object_retrieval::convex_feature_cloud_map features(data_path);

    Eigen::Matrix<float, 250, Eigen::Dynamic, Eigen::ColMajor> data_matrix(250, numData);

    size_t counter = 0;
    for (HistCloudT::Ptr& f : features) {
        if (counter >= numData) {
            break;
        }
        for (const HistT& h : f->points) {
            if (counter >= numData) {
                break;
            }
            if (pcl_hist_inf(h)) {
                continue;
            }
            data_matrix.col(counter) = Eigen::Map<const Eigen::Matrix<float, 250, 1> >(h.histogram);
            ++counter;
        }
    }

    cout << "Got all of the data" << endl;

    cout << "Allocating k-means" << endl;

    // create a KMeans object and run clustering to get vocabulary words (centers)
    repr.kmeans = vl_kmeans_new(VL_TYPE_FLOAT, VlDistanceL2) ;

    cout << "Starting k-means" << endl;

    vl_kmeans_cluster (repr.kmeans,
                       data_matrix.data(),
                       repr.dimension,
                       numData,
                       repr.numCenters) ;

    Eigen::Map<Eigen::Matrix<float, 250, nbr_centers, Eigen::ColMajor> > center_map((float*)vl_kmeans_get_centers(repr.kmeans));
    HistCloudT::Ptr ccloud(new HistCloudT);
    for (int i = 0; i < repr.numCenters; i++) {
        ccloud->push_back(HistT());
        Eigen::Map<Eigen::Matrix<float, 250, 1> >(ccloud->back().histogram) = center_map.col(i);
    }
    pcl::io::savePCDFileBinary("cluster_centers.pcd", *ccloud);
}

void encode_vlad_representation(const boost::filesystem::path& data_path,
                                vlad_repr& repr)
{
    dynamic_object_retrieval::convex_feature_cloud_map features(data_path);

    // allocate space for vlad encoding
    //float * enc = (float*)vl_malloc(sizeof(VL_TYPE_FLOAT) * repr.dimension * repr.numCenters);

    vl_uint32 * indexes = (vl_uint32*)vl_malloc(sizeof(vl_uint32) * 20000);
    float * assignments = (float*)vl_malloc(sizeof(float) * 20000 * repr.numCenters);
    float * distances = (float*)vl_malloc(sizeof(float) * 20000);

    VladCloudT::Ptr vcloud(new VladCloudT);
    for (HistCloudT::Ptr& f : features) {
        Eigen::Matrix<float, 250, Eigen::Dynamic, Eigen::ColMajor> data_matrix(250, f->size());
        size_t counter = 0;
        for (HistT& h : f->points) {
            //h.histogram[249] = 0;
            if (pcl_hist_inf(h)) {
                continue;
            }
            data_matrix.col(counter) = Eigen::Map<const Eigen::Matrix<float, 250, 1> >(h.histogram);
            ++counter;
        }
        data_matrix.conservativeResize(250, counter);

        //cout << f->size() << endl;

        //vl_uint32 * indexes;
        //float * assignments;
        //void* distances; // distances from centers to training points?

        //cout << "Allocating assignment" << endl;
        // find nearest cliuster centers for the data that should be encoded
        //indexes = (vl_uint32*)vl_malloc(sizeof(vl_uint32) * f->size());
        //distances = vl_malloc(sizeof(float) * f->size());

        //cout << "Starting assignment" << endl;

        vl_kmeans_quantize(repr.kmeans,indexes,distances,data_matrix.data(),counter);
        // convert indexes array to assignments array,
        // which can be processed by vl_vlad_encode

        //cout << "Setting up assignment conversion" << endl;

        //assignments = (float*)vl_malloc(sizeof(float) * f->size() * repr.numCenters);

        //cout << "Finished assignment, doing something with data?" << endl;

        memset(assignments, 0, sizeof(float) * counter * repr.numCenters);
        for (int i = 0; i < counter; i++) {
            assignments[i * repr.numCenters + indexes[i]] = 1.;
        }

        //cout << "Allocating for encoding" << endl;
        // do the encoding job

        //#define VL_VLAD_FLAG_NORMALIZE_COMPONENTS (0x1 << 0)
        //#define VL_VLAD_FLAG_SQUARE_ROOT          (0x1 << 1)
        //#define VL_VLAD_FLAG_UNNORMALIZED         (0x1 << 2)
        //#define VL_VLAD_FLAG_NORMALIZE_MASS       (0x1 << 3)
        vcloud->push_back(VladT());
        vl_vlad_encode (vcloud->back().histogram, VL_TYPE_FLOAT,
                        vl_kmeans_get_centers(repr.kmeans), repr.dimension, repr.numCenters,
                        data_matrix.data(), counter,
                        assignments,
                        VL_VLAD_FLAG_UNNORMALIZED);

        /*
        Eigen::Map<Eigen::Matrix<float, 250, 16, Eigen::ColMajor> > center_map((float*)vl_kmeans_get_centers(repr.kmeans));
        for (int i = 0; i < 16; i++) {
            cout << center_map.col(i).transpose() << endl;
        }

        Eigen::Matrix<float, 250, 16, Eigen::ColMajor> vlad_manual;
        vlad_manual.setZero();

        for (const HistT& h : f->points) {
            Eigen::Map<const Eigen::Matrix<float, 250, 1> > point(h.histogram);
            Eigen::Matrix<float, 1, 16>::Index index;
            (center_map.colwise() - point).colwise().squaredNorm().minCoeff(&index);
            vlad_manual.col(index) += point - center_map.col(index);
        }

        Eigen::Map<Eigen::Matrix<float, V, 1> > vlad_map(vlad_manual.data());
        vlad_map.normalize();
        Eigen::Map<Eigen::Matrix<float, V, 1> > vlad_given(vcloud->back().histogram);

        cout << (vlad_map - vlad_given).squaredNorm() << endl;
        */

        //vl_free(indexes);
        //vl_free(distances);
        //vl_free(assignments);

        //Eigen::Map<Eigen::Matrix<float, V, 1> >(vcloud->back().histogram) =
        //    Eigen::Map<Eigen::Matrix<float, V, 1> >(enc);
    }

    vl_free(indexes);
    vl_free(distances);
    vl_free(assignments);

    pcl::io::savePCDFileBinary("vlad_features.pcd", *vcloud);
}

void vlad_sqrt_normalization(VladT& v)
{
    for (float& f : v.histogram) {
        f = float(copysign(1, f))*sqrt(fabs(f));
    }
}
void vlad_l2_normalization(VladT& v)
{
    Eigen::Map<Eigen::Matrix<float, V, 1> > vlad_map(v.histogram);
    float n = vlad_map.norm();
    if (n == 0.0f) {
        vlad_map.setOnes();
        vlad_map.normalize();
    }
    else {
        vlad_map *= 1.0f/n;
    }
}

void vlad_intranormalization(VladT& v)
{
    Eigen::Map<Eigen::Matrix<float, 250, nbr_centers, Eigen::ColMajor> > vlad_map(v.histogram);
    float n;
    for (int i = 0; i < nbr_centers; ++i) {
        n = vlad_map.col(i).norm();
        if (n == 0.0f) {
            vlad_map.col(i).setOnes();
            vlad_map.col(i).normalize();
        }
        else {
            vlad_map.col(i) *= 1.0f/n;
        }
    }
}

VladT encode_vlad_point(HistCloudT::Ptr& features)
{
    HistCloudT::Ptr ccloud(new HistCloudT);
    pcl::io::loadPCDFile("cluster_centers.pcd", *ccloud);

    Eigen::Matrix<float, 250, nbr_centers, Eigen::ColMajor> centers(250, nbr_centers);
    for (int i = 0; i < nbr_centers; ++i) {
        centers.col(i) = Eigen::Map<const Eigen::Matrix<float, 250, 1> >(ccloud->at(i).histogram);
    }

    VladT vlad_point;
    Eigen::Map<Eigen::Matrix<float, 250, nbr_centers, Eigen::ColMajor> > vlad_map(vlad_point.histogram);
    vlad_map.setZero();

    for (const HistT& h : features->points) {
        if (pcl_hist_inf(h)) {
            continue;
        }
        Eigen::Map<const Eigen::Matrix<float, 250, 1> > point(h.histogram);
        Eigen::Matrix<float, 1, nbr_centers>::Index index;
        (centers.colwise() - point).colwise().squaredNorm().minCoeff(&index);
        vlad_map.col(index) += point - centers.col(index);
    }

    return vlad_point;
}

vector<pair<float, string> > query_vlad_representation(VladCloudT::Ptr& vcloud, pcl::KdTreeFLANN<VladT>& kdtree,
                                                       const dynamic_object_retrieval::data_summary& summary,
                                                       HistCloudT::Ptr& fcloud)
{
    //VladCloudT::Ptr vcloud(new VladCloudT);
    if (vcloud->empty()) {
        pcl::io::loadPCDFile("vlad_features.pcd", *vcloud);

        size_t inf_before_norm = 0;
        size_t inf_after_norm = 0;
        size_t are_zero[nbr_centers] = {0};
        for (VladT& v : vcloud->points) {
            //cout << Eigen::Map<Eigen::Matrix<float, V, 1> >(v.histogram).norm() << endl;
            //Eigen::Map<Eigen::Matrix<float, V, 1> >(v.histogram).normalize();
            Eigen::Map<Eigen::Matrix<float, 250, nbr_centers, Eigen::ColMajor> > vlad_map(v.histogram);

            if (pcl_hist_inf(v)) {
                cout << "Was inf before norm!" << endl;
                vlad_map.setZero();
                inf_before_norm++;
                continue;
                //exit(-1);
            }

            //vlad_sqrt_normalization(v);
            //vlad_intranormalization(v);
            vlad_l2_normalization(v);

            Eigen::Map<Eigen::Matrix<float, V, 1> > point(v.histogram);
            point.normalize();
            //point *= 1.0f/point.lpNorm<2>();

            if (pcl_hist_inf(v)) {
                cout << "Was inf after norm!" << endl;
                vlad_map.setZero();
                inf_after_norm++;
                continue;
            }
        }

        cout << "number inf before: " << inf_before_norm << endl;
        cout << "number inf after: " << inf_after_norm << endl;
        for (int i = 0; i < nbr_centers; ++i) {
            cout << are_zero[i] << " ";
        }
        cout << endl;

        kdtree.setInputCloud(vcloud);
    }

    VladT vpoint = encode_vlad_point(fcloud);
    //vlad_sqrt_normalization(vpoint);
    //vlad_intranormalization(vpoint);
    vlad_l2_normalization(vpoint);

    //pcl::KdTreeFLANN<VladT> kdtree;

    int K = 15;

    cout << "Querying:" << endl;
    cout << Eigen::Map<Eigen::Matrix<float, V, 1> >(vpoint.histogram).transpose() << endl;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if (kdtree.nearestKSearch(vpoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) == 0) {
        cout << "No matches found!" << endl;
        exit(0);
    }

    vector<pair<float, string> > results;
    for (int i = 0; i < K; i++) {
        string cpath = summary.index_convex_segment_paths[pointIdxNKNSearch[i]];
        cout << pointIdxNKNSearch[i] << endl;
        cout << cpath << endl;
        cout << pointNKNSquaredDistance[i] << endl;
        Eigen::Map<Eigen::Matrix<float, nbr_centers, 1> > vlad_map(vcloud->at(pointIdxNKNSearch[i]).histogram);
        cout << vlad_map.transpose() << endl;
        results.push_back(make_pair(pointNKNSquaredDistance[i], summary.index_convex_segment_paths[pointIdxNKNSearch[i]]));
    }

    return results;
}

} // namespace vlad_representation
