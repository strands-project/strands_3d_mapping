#include <iostream>
#include <fstream>

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"

#include "g2o/core/factory.h"

//#include "g2o/config.h"
//#include "g2o/core/base_vertex.h"
//#include "g2o/core/hyper_graph_action.h"
//#include "g2o/types/slam3d/isometry3d_mappings.h"
//#include "g2o/types/slam3d/g2o_types_slam3d_api.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "g2o/core/eigen_types.h"

using namespace Eigen;
using namespace g2o;
	
class PointcloudSE3 : public VertexSE3
{
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	int nr_points;
	double * points;
	double * normals;

	PointcloudSE3(){
		nr_points = 0;
		points = 0;
		normals = 0;

		setToOriginImpl();
		updateCache();
	}
      
	PointcloudSE3(Eigen::Matrix3Xd points_, Eigen::Matrix3Xd normals_){
		nr_points = points_.cols();
		points = new double[nr_points*3];
		normals = new double[nr_points*3];

		for(unsigned int i = 0; i < nr_points; i++){
			points[3*i +0] = points_(0,i);
			points[3*i +1] = points_(1,i);
			points[3*i +2] = points_(2,i);
			normals[3*i+0] = normals_(0,i);
			normals[3*i+1] = normals_(1,i);
			normals[3*i+2] = normals_(2,i);
		}

		setToOriginImpl();
		updateCache();
	}
};

class EdgePointcloudSE3 : public BaseBinaryEdge<1, Eigen::Matrix<double,1,1,Eigen::ColMajor>, PointcloudSE3, PointcloudSE3> {
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
	int nr_matches;
	int * matches;
	double * weights;
	
	double maxd;
	int nr_bins;
	double * slope;
	double * cost;
	
	EdgePointcloudSE3();
	
	void init(){
		nr_matches = 0;
		matches = 0;
		weights = 0;
	}

	void setMatches(std::vector<int> & matchidi, std::vector<int> & matchidj){
		unsigned int nr_matchesi = matchidi.size();

		unsigned int count = 0;
		for(unsigned int i = 0; i < nr_matchesi; i++){
			int ki = matchidi[i];
			int kj = matchidj[ki];
			if(kj == i){count++;}
		}
		
		if(count == 0){return ;}
		
		if(nr_matches > 0){
			delete[] matches;
			delete[] weights;
		}
		
		nr_matches	= count;
		matches		= new int 		[2*nr_matches];
		weights		= new double 	[nr_matches];
		
		count = 0;
		for(unsigned int i = 0; i < nr_matchesi; i++){
			int ki = matchidi[i];
			int kj = matchidj[ki];
			if(kj == i){
				matches[2*count + 0] = ki;
				matches[2*count + 1] = kj;
				count++;
			}
		}
	}
	
		
	double getCost(double d){
		if(d >= maxd){return cost[nr_bins];}
		int binid = double(nr_bins)*d/maxd;
		return slope[binid]*d*d+cost[binid];
	}
/*
	double getCost(double d){
		return d;//*d;
	}
*/
	void setCostFunction(double maxd_, int nr_bins_, double * slope_, double * cost_){
		maxd = maxd_;
		nr_bins = nr_bins_;
		slope = slope_;
		cost = cost_;
	}
	
	void computeError(){
		PointcloudSE3 * A		= static_cast<PointcloudSE3*>(_vertices[0]);
		PointcloudSE3 * B		= static_cast<PointcloudSE3*>(_vertices[1]);
		
		double * A_points		= A->points;
		double * A_normals		= A->normals;

		double * B_points		= B->points;
		double * B_normals		= B->normals;
		
		Isometry3D diff			= A->estimate().inverse() * B->estimate();
		Eigen::Affine3d p (diff);
		double m00 = p(0,0);	double m01 = p(0,1);	double m02 = p(0,2);	double m03 = p(0,3);
		double m10 = p(1,0);	double m11 = p(1,1);	double m12 = p(1,2);	double m13 = p(1,3);
		double m20 = p(2,0);	double m21 = p(2,1);	double m22 = p(2,2);	double m23 = p(2,3);

		//printf("-------------------------------------\n");
		//printf("%9.9f %9.9f %9.9f %9.9f\n",m00,m01,m02,m03);		
		//printf("%9.9f %9.9f %9.9f %9.9f\n",m10,m11,m12,m13);		
		//printf("%9.9f %9.9f %9.9f %9.9f\n",m10,m21,m22,m23);		
		//printf("-------------------------------------\n");

		double e = 0;
		
		for(unsigned int count = 0; count < nr_matches; count++){
			int ki = matches[2*count + 0];
			int kj = matches[2*count + 1];
			

			
			double ax	= A_points [3*ki+0];
			double ay	= A_points [3*ki+1];
			double az	= A_points [3*ki+2];
			double anx	= A_normals[3*ki+0];
			double any	= A_normals[3*ki+1];
			double anz	= A_normals[3*ki+2];
			
			double bx	= B_points [3*kj+0];
			double by	= B_points [3*kj+1];
			double bz	= B_points [3*kj+2];
			double bnx	= B_normals[3*kj+0];
			double bny	= B_normals[3*kj+1];
			double bnz	= B_normals[3*kj+2];
					
			double tbx	= m00*bx + m01*by + m02*bz + m03;
			double tby	= m10*bx + m11*by + m12*bz + m13;
			double tbz	= m20*bx + m21*by + m22*bz + m23;
			
			double tbnx	= m00*bnx + m01*bny + m02*bnz;
			double tbny	= m10*bnx + m11*bny + m12*bnz;
			double tbnz	= m20*bnx + m21*bny + m22*bnz;

			double dx = ax-tbx;
			double dy = ay-tby;
			double dz = az-tbz;
			
			double diff_ab = anx*dx + any*dy + anz*dz;
			double diff_ba = tbnx*dx + tbny*dy + tbnz*dz;
		
			//double cost = diff_ab*diff_ab + diff_ba*diff_ba;
			double cost = getCost(fabs(diff_ab) + fabs(diff_ba));
			
			e += cost;
/*
			if(count == 0){
				//printf("-------------\n");
				//printf("match:%i %i\n",ki,kj); 
				//printf("a  point: %9.9f %9.9f %9.9f , normal: %9.9f %9.9f %9.9f\n",ax,ay,az,anx,any,anz);
				//printf("b  point: %9.9f %9.9f %9.9f , normal: %9.9f %9.9f %9.9f\n",bx,by,bz,bnx,bny,bnz);
				//printf("tb point: %9.9f %9.9f %9.9f , normal: %9.9f %9.9f %9.9f",tbx,tby,tbz,tbnx,tbny,tbnz);
				
				//printf("b  point: %19.19f\n",bx);
				//printf("tb point: %19.19f\n",tbx);
				//printf("diff:     %19.19f\n",tbx-bx); 
				
				printf("diff_ab: %19.19f * (%19.19f - %19.19f) -> %19.19f\n",anx,ax,tbx,anx*dx);
				
				//printf("d: %15.15f %15.15f %15.15f ",dx,dy,dz);
				//printf("diff_ab: %9.9f diff_ba\n",diff_ab,diff_ba);
				//printf("cost: %20.20f ",cost);
			}
*/
		}
		//printf("error: %20.20f\n",e);
		//exit(0);
		_error[0] = sqrt(e);
		//_error[1] = 0;
		//_error[2] = 0;
	}
	
	void linearizeOplus();
	bool read(std::istream& is) 			{return true;}
	bool write(std::ostream& os) const 		{return true;}
};

EdgePointcloudSE3::EdgePointcloudSE3() : BaseBinaryEdge<1, Eigen::Matrix<double,1,1,Eigen::ColMajor>, PointcloudSE3, PointcloudSE3>(){
	information().setIdentity();
}

void EdgePointcloudSE3::linearizeOplus(){
	//printf("linearizeOplus()\n");

  VertexXiType* vi = static_cast<VertexXiType*>(_vertices[0]);
  VertexXjType* vj = static_cast<VertexXjType*>(_vertices[1]);

  bool iNotFixed = !(vi->fixed());
  bool jNotFixed = !(vj->fixed());

  if (!iNotFixed && !jNotFixed)
    return;
/*
	PointcloudSE3 * A		= static_cast<PointcloudSE3*>(_vertices[0]);
	PointcloudSE3 * B		= static_cast<PointcloudSE3*>(_vertices[1]);
		
	double * A_points		= A->points;
	double * A_normals		= A->normals;

	double * B_points		= B->points;
	double * B_normals		= B->normals;
*/
  const double delta = 1e-9;
  const double scalar = 1.0 / (2*delta);
  ErrorVector errorBak;
  ErrorVector errorBeforeNumeric = _error;

  if (iNotFixed) {
  
	//printf("iNotFixed\n");
    double add_vi[VertexXiType::Dimension];
    std::fill(add_vi, add_vi + VertexXiType::Dimension, 0.0);
    for (int d = 0; d < VertexXiType::Dimension; ++d) {
      vi->push();
      add_vi[d] = delta;
      vi->oplus(add_vi);
      computeError();
      errorBak = _error;
      vi->pop();
      vi->push();
      add_vi[d] = -delta;
      vi->oplus(add_vi);
      computeError();
      errorBak -= _error;
      vi->pop();
      add_vi[d] = 0.0;
      _jacobianOplusXi.col(d) = scalar * errorBak;
    }
    
    
  }

if (jNotFixed) {
	//printf("jNotFixed\n");
    double add_vj[VertexXjType::Dimension];
    std::fill(add_vj, add_vj + VertexXjType::Dimension, 0.0);
    for (int d = 0; d < VertexXjType::Dimension; ++d) {
      vj->push();
      add_vj[d] = delta;
      vj->oplus(add_vj);
      computeError();
      errorBak = _error;
      vj->pop();
      vj->push();
      add_vj[d] = -delta;
      vj->oplus(add_vj);
      computeError();
      errorBak -= _error;
      vj->pop();
      add_vj[d] = 0.0;

      _jacobianOplusXj.col(d) = scalar * errorBak;
    }
  }

  _error = errorBeforeNumeric;
}

/*
void EdgePointcloudSE3::linearizeOplus(){
	//printf("linearizeOplus()\n");

  VertexXiType* vi = static_cast<VertexXiType*>(_vertices[0]);
  VertexXjType* vj = static_cast<VertexXjType*>(_vertices[1]);

  bool iNotFixed = !(vi->fixed());
  bool jNotFixed = !(vj->fixed());

  if (!iNotFixed && !jNotFixed)
    return;

  const double delta = 1e-9;
  const double scalar = 1.0 / (2*delta);
  ErrorVector errorBak;
  ErrorVector errorBeforeNumeric = _error;

  if (iNotFixed) {
  
	//printf("iNotFixed\n");
    double add_vi[VertexXiType::Dimension];
    std::fill(add_vi, add_vi + VertexXiType::Dimension, 0.0);
    for (int d = 0; d < VertexXiType::Dimension; ++d) {
      vi->push();
      add_vi[d] = delta;
      vi->oplus(add_vi);
      computeError();
      errorBak = _error;
      vi->pop();
      vi->push();
      add_vi[d] = -delta;
      vi->oplus(add_vi);
      computeError();
      errorBak -= _error;
      vi->pop();
      add_vi[d] = 0.0;
      _jacobianOplusXi.col(d) = scalar * errorBak;
    }
  }

if (jNotFixed) {
	//printf("jNotFixed\n");
    double add_vj[VertexXjType::Dimension];
    std::fill(add_vj, add_vj + VertexXjType::Dimension, 0.0);
    for (int d = 0; d < VertexXjType::Dimension; ++d) {
      vj->push();
      add_vj[d] = delta;
      vj->oplus(add_vj);
      computeError();
      errorBak = _error;
      vj->pop();
      vj->push();
      add_vj[d] = -delta;
      vj->oplus(add_vj);
      computeError();
      errorBak -= _error;
      vj->pop();
      add_vj[d] = 0.0;

      _jacobianOplusXj.col(d) = scalar * errorBak;
    }
  }
  //std::cout << _jacobianOplusXi.row(0) << std::endl;
  //std::cout << _jacobianOplusXj.row(0) << std::endl;

  _error = errorBeforeNumeric;
#ifdef G2O_OPENMP
  vj->unlockQuadraticForm();
  vi->unlockQuadraticForm();
#endif
}

*/

double getCost(double maxd, int nr_bins, double * slope, double * cost, double d){
	if(d >= maxd){return cost[nr_bins];}
	int binid = double(nr_bins)*d/maxd;
	return slope[binid]*d*d+cost[binid];
}


void getCostFromSlope(double maxd, int nr_bins, double * slope, double * cost){
	cost[0] = 0;
	for(int i = 0; i < nr_bins; i++){
		double before = double(i)*maxd/double(nr_bins);
		double after = double(i+1)*maxd/double(nr_bins);
		double diff = slope[i]*(after*after - before*before);
		cost[i+1] = cost[i] + diff; 
	}
	
	for(int i = 0; i < nr_bins; i++){
		double before = double(i)*maxd/double(nr_bins);
		cost[i] -= slope[i]*before*before;
	}
}

using namespace g2o;
void testMinimizer(){
	double step = 0.001;
	
	double maxd = 1.0;
	int nr_bins = 1000;
	double * slope = new double[nr_bins+1];
	double * cost = new double[nr_bins+1];
	
	slope[nr_bins] = 0;
	for(int i = 0; i < nr_bins; i++){slope[i] = 1;}//0.001*double(rand()%1000);}
	//for(int i = 0; i < nr_bins; i++){
	//	slope[i] = exp(-0.5*pow(5.0*double(i)/double(nr_bins),2));
	//}
	getCostFromSlope(maxd,nr_bins,slope,cost);
/*	
	cost[0] = 0;
	for(int i = 0; i < nr_bins; i++){
		double before = double(i)*maxd/double(nr_bins);
		double after = double(i+1)*maxd/double(nr_bins);
		
		double diff = slope[i]*(after*after - before*before);

		cost[i+1] = cost[i] + diff; 
		printf("before: %f after: %f\n",before,after);
	}
	
	printf("X = [");for(int i = 0; i < nr_bins+1; i++){printf("%f ",double(i)*maxd/double(nr_bins));}printf("];\n");
	printf("slope = [");for(int i = 0; i < nr_bins; i++){printf("%5.5f ",slope[i]);}printf("];\n");
	printf("cost = [");for(int i = 0; i < nr_bins+1; i++){printf("%5.5f ",cost[i]);}printf("];\n");
	
	for(int i = 0; i < nr_bins; i++){
		double before = double(i)*maxd/double(nr_bins);
		cost[i] -= slope[i]*before*before;
	}

	printf("X2 = [");for(double d = 0; d < 2.0*maxd; d+= step){printf("%f ",d);}printf("];\n");
	printf("cost2 = [");for(double d = 0; d < 2.0*maxd; d+= step){printf("%5.5f ",getCost(maxd,nr_bins,slope,cost,d));}printf("];\n");
*/

//	printf("X3 = [");for(double d = 0; d <= 1.25*maxd; d+= step){printf("%f ",d);}printf("];\n");
//	printf("cost3 = [");for(double d = 0; d <= 1.25*maxd; d+= step){printf("%5.5f ",getCost(maxd,nr_bins,slope,cost,d));}printf("];\n");
//	for(double d = 0; d < 1.0*maxd; d+= 0.1){
//		printf("%3.3f -> %5.5f\n",d,getCost(maxd,nr_bins,slope,cost,d));
//	}


	int nr_pts = 10000;
//GT set up
//printf("setting up GT\n");

	double sum000 = 0;
	double sum010 = 0;
	double sum100 = 0;
	double sum110 = 0;

	double sum001 = 0;
	double sum011 = 0;
	double sum101 = 0;
	double sum111 = 0;

	double sum002 = 0;
	double sum012 = 0;
	double sum102 = 0;
	double sum112 = 0;

		Eigen::VectorXd gt_w		(nr_pts);

		Eigen::Matrix3Xd gt_point	= Eigen::Matrix3Xd::Zero(3,nr_pts);
		Eigen::Matrix3Xd gt_normal	= Eigen::Matrix3Xd::Zero(3,nr_pts);
		for(int i = 0; i < nr_pts; i++){
			gt_point(0,i) = 2.0*double(rand()%1000)*0.001 -1.0;
			gt_point(1,i) = 2.0*double(rand()%1000)*0.001 -1.0;
			gt_point(2,i) = 2.0*double(rand()%1000)*0.001 -1.0;

			float nx = 2.0*double(rand()%1000)*0.001 -1.0;
			float ny = 2.0*double(rand()%1000)*0.001 -1.0;
			float nz = 2.0*double(rand()%1000)*0.001 -1.0;
			float norm = sqrt(nx*nx+ny*ny+nz*nz);
			nx /= norm;
			ny /= norm;
			nz /= norm;
			gt_normal(0,i) = nx;
			gt_normal(1,i) = ny;
			gt_normal(2,i) = nz;
			gt_w(i) = 1;
		}

		std::default_random_engine generator;
		std::normal_distribution<double> distribution_points(0.0,0.1);
		std::normal_distribution<double> distribution_normals(0.0,0.0001);
		
	std::vector<Eigen::Matrix3Xd> X_points;
	std::vector<Eigen::Matrix3Xd> X_normals;
		
    for(int iter = 0; iter < 15; iter++){
    	double offset_x = 10.0*distribution_points(generator);
		double offset_y = 10.0*distribution_points(generator);
		double offset_z = 10.0*distribution_points(generator);
		Eigen::Matrix3Xd X_point = gt_point;
		Eigen::Matrix3Xd X_normal = gt_normal;
		for(int i = 0; i < nr_pts; i++){
			X_point(0,i) += offset_x;
			X_point(1,i) += offset_y;
			X_point(2,i) += offset_z;

			double px = distribution_points(generator);
			double py = distribution_points(generator);
			double pz = distribution_points(generator);

			X_point(0,i) += px;
			X_point(1,i) += py;
			X_point(2,i) += pz;

			double nx = X_normal(0,i)+distribution_normals(generator);
			double ny = X_normal(1,i)+distribution_normals(generator);
			double nz = X_normal(2,i)+distribution_normals(generator);
			float norm = sqrt(nx*nx+ny*ny+nz*nz);
			nx /= norm;
			ny /= norm;
			nz /= norm;
			X_normal(0,i) = nx;
			X_normal(1,i) = ny;
			X_normal(2,i) = nz;
		}
		X_points.push_back(X_point);
		X_normals.push_back(X_normal);
	}
	
	BlockSolverX::LinearSolverType * linearSolver = new LinearSolverPCG<BlockSolverX::PoseMatrixType>();
	BlockSolverX* blockSolver = new BlockSolverX(linearSolver);
	OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);
	SparseOptimizer optimizer;
	optimizer.setVerbose(true);
	optimizer.setAlgorithm(optimizationAlgorithm);
	
	std::vector<int> matchidi;
	for(int i = 0; i < nr_pts; i++){matchidi.push_back(i);}

	std::vector<PointcloudSE3 *> verts;
    for(int i = 0; i < X_points.size(); i++){
        PointcloudSE3 * v = new PointcloudSE3(X_points[i],X_normals[i]);
		v->setId(i);
		verts.push_back(v);
    	optimizer.addVertex(verts[i]);
    }

    for(int i = 0; i < X_points.size(); i++){
		for(int j = 0; j < i; j++){
			EdgePointcloudSE3 * edge = new EdgePointcloudSE3();
			edge->vertices()[0] = verts[j];//dst;
			edge->vertices()[1] = verts[i];//src;
			edge->init();
			edge->setMatches(matchidi,matchidi);
			edge->setCostFunction(maxd, nr_bins, slope, cost);
			optimizer.addEdge(edge);
    	}
    }   
	optimizer.initializeOptimization();
	optimizer.optimize(3000);

}


int main(int argc, char **argv){
	testMinimizer();
	return 0;
}
