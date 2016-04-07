#ifndef MassRegistrationPPRColor_H
#define MassRegistrationPPRColor_H

#include "MassRegistration.h"
#include "../core/Util.h"

#include "lum2.h"

//#include "DistanceWeightFunction2.h"
//#include "ICP.h"
namespace reglib
{
//	typedef nanoflann2::KDTreeEigenMatrixAdaptor< Eigen::MatrixXd,6,nanoflann2::metric_L2_Simple> KDTree;
	typedef nanoflann2::KDTreeEigenMatrixAdaptor< Eigen::Matrix<double,Dynamic,Dynamic>, 6,nanoflann2::metric_L2_Simple>  KDTree;
	typedef nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor<double, ArrayData3D<double> > , ArrayData3D<double>, 3 > KDTree2;

	class PointMatch
	{
		public:
		size_t src;
		std::vector<size_t> dst;
		PointMatch(size_t src_, std::vector<size_t> & dst_);
		~PointMatch();
	};

	class MassRegistrationPPRColor : public MassRegistration
	{
		public:

		MatchType type;

		bool use_PPR_weight;
		bool use_features;
		bool normalize_matchweights;

		double stopval;
		unsigned steps;

		int max_matches;

		pcl::registration::LUM2<pcl::PointXYZ> * LUM2;
		std::vector<pcl::registration::LUM2< pcl::PointXYZ >::Vertex> verts;

		DistanceWeightFunction2PPR2 * func;
		DistanceWeightFunction2PPR2 * funcR;
		DistanceWeightFunction2PPR2 * funcG;
		DistanceWeightFunction2PPR2 * funcB;

		std::vector<int> nr_datas;

		nanoflann2::SearchParams sp;

//		std::vector< Eigen::MatrixXd > clouds;
		std::vector< Eigen::MatrixXd > noises;
        //std::vector< KDTree * > trees;

		std::vector< int > nrdatas;
		std::vector< double* > clouddatas;
		std::vector< double* > normaldatas;
		std::vector< bool* > depthedges;
		std::vector< unsigned char * > rgbdatas;
		std::vector< ArrayData3D<double> * > ads;
		std::vector< KDTree2 * > treedatas;

		std::vector<std::pair <int,int> > connections;

		std::vector< std::vector< std::vector< PointMatch > > > all_matches;

        std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > points;
        std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > colors;
        std::vector< Eigen::Matrix<double, 7, Eigen::Dynamic> > pointscolors;
        std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > normals;
        std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > transformed_points;
        std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > transformed_normals;
        std::vector< Eigen::VectorXd > informations;
        std::vector< nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> * > trees;
        std::vector< nanoflann::KDTreeAdaptor<Eigen::Matrix<double, 6, Eigen::Dynamic>, 6, nanoflann::metric_L2_Simple> * > trees6d;
        std::vector<int> nr_matches;
        std::vector< std::vector< std::vector<int> > > matchids;

//		std::vector<int> feature_start;//Dimension of data a specific feature starts, if the feature is RGB this should be 3
//		std::vector<int> feature_end;//Dimension of data a specific feature ends, if the feature is RGB this should be 5
//		std::vector< DistanceWeightFunction2 * > feature_func;

//		DistanceWeightFunction2PPR2 * func;

		MassRegistrationPPRColor(double startreg = 0.05, bool visualize = false);
		~MassRegistrationPPRColor();
		
		void setData(std::vector<RGBDFrame*> frames_, std::vector<ModelMask *> mmasks_);
		void preprocessData(int index);
		void rematchAll(std::vector<Eigen::MatrixXd> poses);
		void dorematch(std::vector<Eigen::Matrix4d> poses);
		void recomputeFunctions(std::vector<Eigen::MatrixXd> poses);
		std::vector<Eigen::MatrixXd> refinePoses(std::vector<Eigen::MatrixXd> poses);
		std::vector< PointMatch > rematch(int i, int j, Eigen::MatrixXd pose);
		MassFusionResults getTransforms(std::vector<Eigen::Matrix4d> guess);
		void showMatches(int i, int j, Eigen::MatrixXd pose);
		void show(std::vector<Eigen::MatrixXd> guess, bool color = false);
		void showTP();
		pcl::CorrespondencesPtr getCorrs(int i, int j, Eigen::MatrixXd pose);
		std::vector<CostFunction*> getCostFunctions(int i, int j, Eigen::MatrixXd pose);
	};

}

#endif
