#ifndef MassRegistrationPPR_H
#define MassRegistrationPPR_H

#include "MassRegistration.h"
#include <time.h>

namespace reglib
{

	class MassRegistrationPPR : public MassRegistration
	{
		public:

		MatchType type;

		bool fast_opt;
		bool use_PPR_weight;
		bool use_features;
		bool normalize_matchweights;

		double stopval;
		unsigned steps;

		std::vector<int> nr_datas;

		std::vector< bool > is_ok;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > points;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > colors;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > normals;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > transformed_points;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > transformed_normals;
		std::vector< Eigen::VectorXd > informations;

        std::vector< int > nr_arraypoints;
		std::vector< double * > arraypoints;
		std::vector< Tree3d * > trees3d;
		std::vector< ArrayData3D<double> * > a3dv;

		std::vector<int> nr_matches;
		std::vector< std::vector< std::vector<int> > > matchids;

		std::vector<int> sweepids;
		std::vector<int> background_nr_datas;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_points;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_colors;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_normals;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_transformed_points;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_transformed_normals;
//		std::vector< Eigen::VectorXd > background_informations;
//		std::vector< nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> * > background_trees;
//		std::vector<int> background_nr_matches;
//		std::vector< std::vector< std::vector<int> > > background_matchids;


//		std::vector<int> feature_start;//Dimension of data a specific feature starts, if the feature is RGB this should be 3
//		std::vector<int> feature_end;//Dimension of data a specific feature ends, if the feature is RGB this should be 5
//		std::vector< DistanceWeightFunction2 * > feature_func;

		DistanceWeightFunction2PPR2 * func;

		MassRegistrationPPR(double startreg = 0.05, bool visualize = false);
		~MassRegistrationPPR();
		
		MassFusionResults getTransforms(std::vector<Eigen::Matrix4d> guess);
		void setData(std::vector<RGBDFrame*> frames_, std::vector<ModelMask *> mmasks);
		void setData(std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > all_clouds);

	};

}

#endif
