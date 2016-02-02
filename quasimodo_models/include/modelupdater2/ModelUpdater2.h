#ifndef reglibModelUpdater2_H
#define reglibModelUpdater2_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>

#include <string.h>

#include <Eigen/Dense>
#include "../core/RGBDFrame.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
//#include "g2o/stuff/macros.h"
//#include "g2o/stuff/command_args.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
//#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
//#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
//#include "g2o/solvers/structure_only/structure_only_solver.h"


//#include "../model/Model.h"
//#include "../registration/Registration.h"
//#include <pcl/visualization/pcl_visualizer.h>

namespace reglib
{
	class ModelUpdater2{
		public:
		g2o::SparseOptimizer* g;

		std::vector<unsigned char  *>	d_rgbs;
		std::vector<unsigned short *>	d_depths;

		std::vector<cv::Mat>			masks;
		std::vector<RGBDFrame *>		frames;
		std::vector<Eigen::Matrix4d>	poses;
		std::vector<bool>				iskeyframe;
		std::vector<int>				keyframe_index;
		std::vector<std::vector<int>>	matching;

		std::vector<int>				keyframes;
		std::vector<float>				kfscores;

		float bias;
		float * d_scalingmat;
		float * d_scalingmatweight;

		int histogram_size;
		float trans_maxd;
		int * trans_diff_hist;

		ModelUpdater2();
		~ModelUpdater2();

		void saveToFB(std::string path = "testoutput.txt");
/*
		Model * model;




		virtual void fuse(Model * model2, Eigen::Matrix4d guess = Eigen::Matrix4d::Identity(), double uncertanity = -1);
		virtual void fuseData(FusionResults * f, Model * model1,Model * model2);
		virtual void pruneOcclusions();
*/

		virtual void addFrame(RGBDFrame * frame , cv::Mat mask);
		virtual void refine();
	};

}

#endif
