#include <vlad/bow_representation.h>

// BOW PFHRGB feature struct (numCenters * 250)
POINT_CLOUD_REGISTER_POINT_STRUCT (BowT,
                                   (float[nbr_centers], histogram, histogram)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

using namespace std;

namespace bow_representation {

BowT encode_bow_point(HistCloudT::Ptr& features)
{
    HistCloudT::Ptr ccloud(new HistCloudT);
    pcl::io::loadPCDFile("cluster_centers.pcd", *ccloud);
    Eigen::Matrix<float, 250, nbr_centers, Eigen::ColMajor> centers(250, nbr_centers);
    for (int i = 0; i < nbr_centers; ++i) {
        centers.col(i) = Eigen::Map<const Eigen::Matrix<float, 250, 1> >(ccloud->at(i).histogram);
    }

    BowT bow_point;
    Eigen::Map<Eigen::Matrix<float, nbr_centers, 1> > bow_map(bow_point.histogram);
    bow_map.setZero();
    for (const HistT& h : features->points) {
        Eigen::Map<const Eigen::Matrix<float, 250, 1> > point(h.histogram);
        Eigen::Matrix<float, 1, 16>::Index index;
        (centers.colwise() - point).colwise().squaredNorm().minCoeff(&index);
        bow_map(index) += 1;
    }

    return bow_point;
}

void encode_bow_representation(const boost::filesystem::path& data_path)
{
    dynamic_object_retrieval::convex_feature_cloud_map features(data_path);

    BowCloudT::Ptr bcloud(new BowCloudT);
    for (HistCloudT::Ptr& f : features) {
        BowT b = encode_bow_point(f);
        bcloud->push_back(b);
    }

    pcl::io::savePCDFileBinary("bow_features.pcd", *bcloud);
}

vector<pair<float, string> > query_bow_representation(const dynamic_object_retrieval::data_summary& summary,
                                                      HistCloudT::Ptr& fcloud)
{
    //dynamic_object_retrieval::data_summary summary;
    //summary.load(data_path);
    cout << "number features: " << summary.nbr_convex_segments << endl;

    BowCloudT::Ptr bcloud(new BowCloudT);
    pcl::io::loadPCDFile("bow_features.pcd", *bcloud);

    for (BowT& b : bcloud->points) {
        Eigen::Map<Eigen::Matrix<float, nbr_centers, 1> > bow_map(b.histogram);
        bow_map.normalize();
    }

    BowT bpoint = encode_bow_point(fcloud);
    Eigen::Map<Eigen::Matrix<float, nbr_centers, 1> >(bpoint.histogram).normalize();

    // switch to pcl::search::FlannSearch from pcl 1.8 instead
    pcl::KdTreeFLANN<BowT, flann::L1<float> > kdtree;
    kdtree.setInputCloud(bcloud);

    int K = 10;
    //cout << "Querying:" << endl;
    //cout << Eigen::Map<Eigen::Matrix<float, nbr_centers, 1> >(bcloud->at(i).histogram).transpose() << endl;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if (kdtree.nearestKSearch(bpoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) == 0) {
        cout << "No matches found!" << endl;
        exit(0);
    }

    vector<pair<float, string> > results;
    for (int i = 0; i < K; i++) {
        string cpath = summary.index_convex_segment_paths[pointIdxNKNSearch[i]];
        cout << pointIdxNKNSearch[i] << endl;
        cout << cpath << endl;
        cout << pointNKNSquaredDistance[i] << endl;
        Eigen::Map<Eigen::Matrix<float, nbr_centers, 1> > bow_map(bcloud->at(pointIdxNKNSearch[i]).histogram);
        cout << bow_map.transpose() << endl;
        /*CloudT::Ptr c(new CloudT);
        pcl::io::loadPCDFile(cpath, *c);
        dynamic_object_retrieval::visualize(c);*/

        results.push_back(make_pair(pointNKNSquaredDistance[i], summary.index_convex_segment_paths[pointIdxNKNSearch[i]]));
    }

    return results;
}

} // namespace bow_representation
