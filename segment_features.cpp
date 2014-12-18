#include "segment_features.h"

#include <pcl/common/centroid.h>

segment_features::segment_features()
{
}

void segment_features::calculate_features(Eigen::VectorXf& feature, PointCloudT::Ptr segment) const
{
    Eigen::Matrix<float, 4, 1> centroid;
    Eigen::Matrix3f covariance_matrix;
    //pcl::compute3DCentroid(*segment, centroid);
    //pcl::computeCovarianceMatrixNormalized(*segment, centroid, covariance_matrix);
    pcl::computeMeanAndCovarianceMatrix(*segment, covariance_matrix, centroid);
    std::cout << covariance_matrix << std::endl;
    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
    //std::cout << es.eigenvectors() << std::endl;
    //std::cout << es.eigenvalues() << std::endl;

    Eigen::JacobiSVD<Eigen::Matrix3f> svd;
    svd.compute(covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //Eigen::Vector3f T = svd.singularValues().array().abs();//(svd.matrixU()*svd.singularValues()).array().abs();
    Eigen::Vector3f T = es.eigenvalues().array().abs();
    std::sort(T.data(), T.data() + T.size(), [](float f1, float f2) { return f1 > f2; });
    std::cout << T << std::endl;
    /*std::cout << svd.matrixU() << std::endl;
    std::cout << svd.singularValues() << std::endl;*/

    feature.resize(3);
    feature(0) = fabs(T(1) / T(0));
    feature(1) = fabs(T(2) / T(0));
    feature(2) = segment->size();
}
