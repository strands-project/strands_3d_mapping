#include "object_3d_retrieval/train_metric.h"

using namespace std;

train_metric::train_metric()
{
}

void train_metric::set_training_scores(PointT& query, CloudPtrT& cloud, const vector<float>& scores)
{
    
}

float train_metric::compute_distance(const PointT& p1, const PointT& p2) const
{
    // generalized euclidian metric:
    //Eigen::MatrixXf W(133, 133);
    //return (eig(p1) - eig(p2)).transpose()*W*(eig(p1)-eig(p2));
}
