#ifndef RegistrationRefinementColor_H
#define RegistrationRefinementColor_H

#include "Registration.h"
#include <time.h>

#include "../core/Util.h"

#include "RegistrationPPR.h"
#include "ICP.h"
//#include "GoICP_V1.3/src/jly_goicp.h"

#include <pcl/registration/lum.h>

namespace reglib
{

// This is an exampleof a custom data set class

//typedef KDTreeSingleIndexAdaptor< L2_Simple_Adaptor<double, PointCloud<double> > , PointCloud<double>, 6 > my_kd_tree_t;
	//my_kd_tree_t   index(6 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );

    class RegistrationRefinementColor : public Registration
    {
        public:

		//PointCloudXYZRGB<double> cloud;
		//nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor<double, PointCloudXYZRGB<double> > , PointCloudXYZRGB<double>, 6 > * xyzrgb_tree;

        MatchType type;
        bool use_PPR_weight;
        bool use_features;
        bool normalize_matchweights;

        Eigen::Matrix<double, 3, Eigen::Dynamic> Y;
        Eigen::Matrix<double, 3, Eigen::Dynamic> C;
        Eigen::Matrix<double, 3, Eigen::Dynamic> N;
        std::vector<double> total_dweight;
        unsigned int ycols;

        Eigen::VectorXd DST_INORMATION;
        //nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> kdtree;
		nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> * tree;

        std::vector<int> feature_start;//Dimension of data a specific feature starts, if the feature is RGB this should be 3
        std::vector<int> feature_end;//Dimension of data a specific feature ends, if the feature is RGB this should be 5
        std::vector< DistanceWeightFunction2 * > feature_func;

        DistanceWeightFunction2PPR2 * func;
        DistanceWeightFunction2PPR2 * funcR;
        DistanceWeightFunction2PPR2 * funcG;
        DistanceWeightFunction2PPR2 * funcB;

		pcl::registration::LUM<pcl::PointXYZ> lum;

        RegistrationRefinementColor();
        ~RegistrationRefinementColor();

        void setDst(CloudData * dst_);

        FusionResults getTransform(Eigen::MatrixXd guess);
    };
}

#endif
