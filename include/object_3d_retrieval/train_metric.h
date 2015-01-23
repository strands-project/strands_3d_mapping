#ifndef TRAIN_METRIC_H
#define TRAIN_METRIC_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class train_metric
{
private:
    using PointT = pcl::Histogram<131>;
    using CloudT = pcl::PointCloud<PointT>;
    using CloudPtrT = CloudT::Ptr;
public:
    void set_training_scores(PointT& query, CloudPtrT& cloud, const std::vector<float>& scores);
    train_metric();
};

#endif // TRAIN_METRIC_H
