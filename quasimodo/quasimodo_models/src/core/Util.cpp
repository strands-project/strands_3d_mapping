#include "core/Util.h"
namespace reglib{

	double getTime(){
		struct timeval start1;
		gettimeofday(&start1, NULL);
		return double(start1.tv_sec+(start1.tv_usec/1000000.0));
	}

	Eigen::Matrix4d getMatTest(const double * const camera, int mode){
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		double rr [9];
		ceres::AngleAxisToRotationMatrix(camera,rr);

		ret(0,0) = rr[0];
		ret(1,0) = rr[1];
		ret(2,0) = rr[2];

		ret(0,1) = rr[3];
		ret(1,1) = rr[4];
		ret(2,1) = rr[5];

		ret(0,2) = rr[6];
		ret(1,2) = rr[7];
		ret(2,2) = rr[8];

		ret(0,3) = camera[3];
		ret(1,3) = camera[4];
		ret(2,3) = camera[5];
		return ret;
	}


	double * getCamera(Eigen::Matrix4d mat, int mode){
		double * camera = new double[6];
		double rr [9];
		rr[0] = mat(0,0);
		rr[1] = mat(1,0);
		rr[2] = mat(2,0);

		rr[3] = mat(0,1);
		rr[4] = mat(1,1);
		rr[5] = mat(2,1);

		rr[6] = mat(0,2);
		rr[7] = mat(1,2);
		rr[8] = mat(2,2);
		ceres::RotationMatrixToAngleAxis(rr,camera);

		camera[3] = mat(0,3);
		camera[4] = mat(1,3);
		camera[5] = mat(2,3);
		return camera;
	}

	Eigen::Matrix4d constructTransformationMatrix (const double & alpha, const double & beta, const double & gamma, const double & tx,    const double & ty,   const double & tz){
		// Construct the transformation matrix from rotation and translation
		Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Zero ();
		transformation_matrix (0, 0) =  cos (gamma) * cos (beta);
		transformation_matrix (0, 1) = -sin (gamma) * cos (alpha) + cos (gamma) * sin (beta) * sin (alpha);
		transformation_matrix (0, 2) =  sin (gamma) * sin (alpha) + cos (gamma) * sin (beta) * cos (alpha);
		transformation_matrix (1, 0) =  sin (gamma) * cos (beta);
		transformation_matrix (1, 1) =  cos (gamma) * cos (alpha) + sin (gamma) * sin (beta) * sin (alpha);
		transformation_matrix (1, 2) = -cos (gamma) * sin (alpha) + sin (gamma) * sin (beta) * cos (alpha);
		transformation_matrix (2, 0) = -sin (beta);
		transformation_matrix (2, 1) =  cos (beta) * sin (alpha);
		transformation_matrix (2, 2) =  cos (beta) * cos (alpha);

		transformation_matrix (0, 3) = tx;
		transformation_matrix (1, 3) = ty;
		transformation_matrix (2, 3) = tz;
		transformation_matrix (3, 3) = 1;
		return transformation_matrix;
	}

	void point_to_plane2(		Eigen::Matrix<double, 3, Eigen::Dynamic> & X,
								Eigen::Matrix<double, 3, Eigen::Dynamic> & Xn,
								Eigen::Matrix<double, 3, Eigen::Dynamic> & Y,
								Eigen::Matrix<double, 3, Eigen::Dynamic> & Yn,
								Eigen::VectorXd & W){
		typedef Eigen::Matrix<double, 6, 1> Vector6d;
		typedef Eigen::Matrix<double, 6, 6> Matrix6d;

		Matrix6d ATA;
		Vector6d ATb;
		ATA.setZero ();
		ATb.setZero ();

		unsigned int xcols = X.cols();
		for(unsigned int i=0; i < xcols; i++) {
			const double & sx = X(0,i);
			const double & sy = X(1,i);
			const double & sz = X(2,i);
			const double & dx = Y(0,i);
			const double & dy = Y(1,i);
			const double & dz = Y(2,i);
			const double & nx = Xn(0,i);
			const double & ny = Xn(1,i);
			const double & nz = Xn(2,i);
			const double & weight = W(i);

			double a = nz*sy - ny*sz;
			double b = nx*sz - nz*sx;
			double c = ny*sx - nx*sy;

			ATA.coeffRef (0) += weight * a * a;
			ATA.coeffRef (1) += weight * a * b;
			ATA.coeffRef (2) += weight * a * c;
			ATA.coeffRef (3) += weight * a * nx;
			ATA.coeffRef (4) += weight * a * ny;
			ATA.coeffRef (5) += weight * a * nz;
			ATA.coeffRef (7) += weight * b * b;
			ATA.coeffRef (8) += weight * b * c;
			ATA.coeffRef (9) += weight * b * nx;
			ATA.coeffRef (10) += weight * b * ny;
			ATA.coeffRef (11) += weight * b * nz;
			ATA.coeffRef (14) += weight * c * c;
			ATA.coeffRef (15) += weight * c * nx;
			ATA.coeffRef (16) += weight * c * ny;
			ATA.coeffRef (17) += weight * c * nz;
			ATA.coeffRef (21) += weight * nx * nx;
			ATA.coeffRef (22) += weight * nx * ny;
			ATA.coeffRef (23) += weight * nx * nz;
			ATA.coeffRef (28) += weight * ny * ny;
			ATA.coeffRef (29) += weight * ny * nz;
			ATA.coeffRef (35) += weight * nz * nz;

			double d = weight * (nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz);

			ATb.coeffRef (0) += a * d;
			ATb.coeffRef (1) += b * d;
			ATb.coeffRef (2) += c * d;
			ATb.coeffRef (3) += nx * d;
			ATb.coeffRef (4) += ny * d;
			ATb.coeffRef (5) += nz * d;
		}

		ATA.coeffRef (6) = ATA.coeff (1);
		ATA.coeffRef (12) = ATA.coeff (2);
		ATA.coeffRef (13) = ATA.coeff (8);
		ATA.coeffRef (18) = ATA.coeff (3);
		ATA.coeffRef (19) = ATA.coeff (9);
		ATA.coeffRef (20) = ATA.coeff (15);
		ATA.coeffRef (24) = ATA.coeff (4);
		ATA.coeffRef (25) = ATA.coeff (10);
		ATA.coeffRef (26) = ATA.coeff (16);
		ATA.coeffRef (27) = ATA.coeff (22);
		ATA.coeffRef (30) = ATA.coeff (5);
		ATA.coeffRef (31) = ATA.coeff (11);
		ATA.coeffRef (32) = ATA.coeff (17);
		ATA.coeffRef (33) = ATA.coeff (23);
		ATA.coeffRef (34) = ATA.coeff (29);

		// Solve A*x = b
		Vector6d x = static_cast<Vector6d> (ATA.inverse () * ATb);
		Eigen::Affine3d transformation = Eigen::Affine3d(constructTransformationMatrix(x(0,0),x(1,0),x(2,0),x(3,0),x(4,0),x(5,0)));

		X = transformation*X;
		transformation(0,3) = 0;
		transformation(1,3) = 0;
		transformation(2,3) = 0;
		Xn = transformation*Xn;
	}

	bool isconverged(std::vector<Eigen::Matrix4d> before, std::vector<Eigen::Matrix4d> after, double stopvalr, double stopvalt){
		double change_trans = 0;
		double change_rot = 0;
		unsigned int nr_frames = after.size();
		for(unsigned int i = 0; i < nr_frames; i++){
			for(unsigned int j = i+1; j < nr_frames; j++){
				Eigen::Matrix4d diff_before = after[i].inverse()*after[j];
				Eigen::Matrix4d diff_after	= before[i].inverse()*before[j];
				Eigen::Matrix4d diff = diff_before.inverse()*diff_after;
				double dt = 0;
				for(unsigned int k = 0; k < 3; k++){
					dt += diff(k,3)*diff(k,3);
					for(unsigned int l = 0; l < 3; l++){
						if(k == l){ change_rot += fabs(1-diff(k,l));}
						else{		change_rot += fabs(diff(k,l));}
					}
				}
				change_trans += sqrt(dt);
			}
		}

		change_trans /= double(nr_frames*(nr_frames-1));
		change_rot	 /= double(nr_frames*(nr_frames-1));

		if(change_trans < stopvalt && change_rot < stopvalr){return true;}
		else{return false;}
	}
}
