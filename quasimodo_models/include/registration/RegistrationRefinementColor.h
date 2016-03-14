#ifndef RegistrationRefinementColor_H
#define RegistrationRefinementColor_H

#include "Registration.h"
#include <time.h>

#include "RegistrationPPR.h"
#include "ICP.h"
//#include "GoICP_V1.3/src/jly_goicp.h"

namespace reglib
{

// This is an exampleof a custom data set class
template <typename T>
struct PointCloudXYZRGB
{
	struct Point{
		T  x,y,z,r,g,b;
	};

	T w0, w1, w2, w3, w4, w5;
	std::vector<Point>  pts;
	mutable T count;
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	inline void printCount(){printf("count: %i\n",int(count));}

	inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const{
		count++;
		const T d0=p1[0]-pts[idx_p2].x;
		const T d1=p1[1]-pts[idx_p2].y;
		const T d2=p1[2]-pts[idx_p2].z;

		const T d3=p1[3]-pts[idx_p2].r;
		const T d4=p1[4]-pts[idx_p2].g;
		const T d5=p1[5]-pts[idx_p2].b;
		return w0*d0*d0+w1*d1*d1+w2*d2*d2 + w3*d3*d3+ w4*d4*d4+w5*d5*d5;
	}

	inline T kdtree_get_pt(const size_t idx, int dim) const{
		if		(dim==0) return pts[idx].x;
		else if (dim==1) return pts[idx].y;
		else if (dim==2) return pts[idx].z;
		else if (dim==3) return pts[idx].r;
		else if (dim==4) return pts[idx].g;
		else return pts[idx].b;
	}
	template <class BBOX> bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

//typedef KDTreeSingleIndexAdaptor< L2_Simple_Adaptor<double, PointCloud<double> > , PointCloud<double>, 6 > my_kd_tree_t;
	//my_kd_tree_t   index(6 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );

    class RegistrationRefinementColor : public Registration
    {
        public:


		nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor<double, PointCloudXYZRGB<double> > , PointCloudXYZRGB<double>, 6 > * xyzrgb_tree;

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

        RegistrationRefinementColor();
        ~RegistrationRefinementColor();

        void setDst(CloudData * dst_);

        FusionResults getTransform(Eigen::MatrixXd guess);
    };
}

#endif
