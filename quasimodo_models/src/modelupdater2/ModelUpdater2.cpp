#include "ModelUpdater2.h"

#include <cuda_runtime_api.h>
#include <iostream>
#include <fstream>

#include "ICPOdometry.h"
#include "PPRregistration.h"

namespace reglib
{

void ModelUpdater2::addFrame(reglib::RGBDFrame * frame , cv::Mat mask){
	int threads = 128;
	int blocks = 96;

	unsigned char  * d_rgb;
	unsigned short * d_depth;

	unsigned int nr_pixels = frame->camera->width*frame->camera->height;

	cudaMalloc((void**)(&(d_rgb)),			sizeof(unsigned char)*3*nr_pixels);
	cudaMemcpy(d_rgb, frame->rgb.data,		sizeof(unsigned char)*3*nr_pixels, cudaMemcpyHostToDevice);

	cudaMalloc((void**)(&(d_depth)),		sizeof(unsigned short)*nr_pixels);
	cudaMemcpy(d_depth, frame->depth.data,	sizeof(unsigned short)*nr_pixels, cudaMemcpyHostToDevice);

	matching.push_back(std::vector<int>());
	if(frames.size() == 0){
		poses.push_back(Eigen::Matrix4d::Identity());
		keyframes.push_back(0);
		kfscores.push_back(0);
		iskeyframe.push_back(true);
		keyframe_index.push_back(0);
	}else{
		iskeyframe.push_back(false);
		keyframe_index.push_back(-1);

		Eigen::Matrix4f p = poses.back().cast<float>();
		Eigen::Matrix4f guess = p;
		Eigen::Matrix4d v = Eigen::Matrix4d::Identity();
		if(frames.size() > 1){
			v = poses[frames.size()-2].inverse()*poses.back();
			guess = p*v.cast<float>();
		}

		std::vector<float *> scalemats;
		scalemats.push_back(d_scalingmat);

		PPRregistration ppr;
		//printf("VO: %5.5i to %5.5i\n",d_depths.size(),d_depths.size()-1);
		std::vector<PPRPyramid *> dst_pyramids = ppr.build_pyramids(scalemats, d_rgbs.back(),	d_depths.back(),640,480,1);
		std::vector<PPRPyramid *> src_pyramids = ppr.build_pyramids(scalemats, d_rgb,			d_depth,		640,480,1);
		ppr.fx = frame->camera->fx;
		ppr.fy = frame->camera->fy;
		ppr.cx = frame->camera->cx;
		ppr.cy = frame->camera->cy;
		ppr.ds = 0.001;

		//Eigen::Affine3f pprvel = ppr.registerPyramids(src_pyramids,dst_pyramids,Eigen::Affine3f::Identity());
		Eigen::Affine3f pprvel = ppr.registerPyramids(src_pyramids,dst_pyramids,Eigen::Affine3f(v.cast<float>()));//Eigen::Affine3f(v));
		Eigen::Matrix4f pprpose = p*pprvel.matrix();
		for(unsigned int i = 0; i < src_pyramids.size(); i++){delete src_pyramids[i];}
		for(unsigned int i = 0; i < dst_pyramids.size(); i++){delete dst_pyramids[i];}

		PPRregistration kfppr;
		std::vector<PPRPyramid *> kfdst_pyramids = kfppr.build_pyramids(scalemats, d_rgbs[keyframes.back()],	d_depths[keyframes.back()],640,480,1);
		std::vector<PPRPyramid *> kfsrc_pyramids = kfppr.build_pyramids(scalemats, d_rgb,			d_depth,		640,480,1);
		kfppr.fx = frame->camera->fx;
		kfppr.fy = frame->camera->fy;
		kfppr.cx = frame->camera->cx;
		kfppr.cy = frame->camera->cy;
		kfppr.ds = 0.001;

		//Eigen::Matrix4f pprvel = ppr.registerPyramids(src_pyramids,dst_pyramids,Eigen::Affine3f::Identity());
		//Eigen::Affine3f kfguess  = Eigen::Affine3f(poses[keyframes.back()].inverse().cast<float>()*guess);
		Eigen::Affine3f kfguess  = Eigen::Affine3f(poses[keyframes.back()].inverse().cast<float>()*p)*pprvel;
		Eigen::Affine3f kfpprvel = kfppr.registerPyramids(kfsrc_pyramids,kfdst_pyramids,kfguess);//Eigen::Affine3f(v));
		Eigen::Matrix4f kfpprpose = poses[keyframes.back()].cast<float>()*kfpprvel.matrix();
		for(unsigned int i = 0; i < kfsrc_pyramids.size(); i++){delete kfsrc_pyramids[i];}
		for(unsigned int i = 0; i < kfdst_pyramids.size(); i++){delete kfdst_pyramids[i];}

		Eigen::Vector3f trans = p.topRightCorner(3, 1);
		Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = p.topLeftCorner(3, 3);

/*
		ICPOdometry icpOdom2 (640, 480, frame->camera->cx, frame->camera->cy, frame->camera->fx, frame->camera->fy);//, 0.01, sin(10.f * 3.14159254f / 180.f));

		trans = poses[keyframes.back()].cast<float>().topRightCorner(3, 1);
		rot = poses[keyframes.back()].cast<float>().topLeftCorner(3, 3);
		icpOdom2.initICP(		(unsigned short *)frame->depth.data,			20.0f);
		icpOdom2.initICPModel(	(unsigned short *)frames[keyframes.back()]->depth.data, 	20.0f, poses[keyframes.back()].cast<float>());
		icpOdom2.getIncrementalTransformation(trans, rot, threads, blocks, Eigen::Affine3f(poses.back().cast<float>()));//Eigen::Affine3f(guess));//poses.back().cast<float>());//guess);
		guess.topLeftCorner(3, 3) = rot;
		guess.topRightCorner(3, 1) = trans;

		double score = log(icpOdom2.getCovariance().determinant());
		if(frames.size() == 1){kfscores[0] = score;}

		trans = poses.back().cast<float>().topRightCorner(3, 1);
		rot = poses.back().cast<float>().topLeftCorner(3, 3);
		Eigen::Matrix4f local_guess = (poses.back()*v).cast<float>();
		icpOdom2.initICPModel(	(unsigned short *)frames.back()->depth.data, 	20.0f, poses.back().cast<float>());
        icpOdom2.getIncrementalTransformation(trans, rot, threads, blocks, Eigen::Affine3f(local_guess));//poses.back().cast<float>());//guess);
		local_guess.topLeftCorner(3, 3) = rot;
		local_guess.topRightCorner(3, 1) = trans;
*/
		Eigen::Matrix4d velocity_global = ((poses.back().inverse())*kfpprpose.cast<double>());
		Eigen::Matrix4d velocity_local = ((poses.back().inverse())*pprpose.cast<double>());

		//std::cout << "========================================" << std::endl;
		//std::cout << velocity_local << std::endl << std::endl;
		//std::cout << pprvel.matrix() << std::endl << std::endl;


		Eigen::Matrix4d acc = v.inverse()*velocity_global;

		Eigen::Matrix4d diff = velocity_local.inverse()*velocity_global;
		double trans_acc = sqrt(acc(0,3)*acc(0,3)+acc(1,3)*acc(1,3)+acc(2,3)*acc(2,3));
		double trans_diff = sqrt(diff(0,3)*diff(0,3)+diff(1,3)*diff(1,3)+diff(2,3)*diff(2,3));
		double rot_acc = 0;
		double rot_diff = 0;
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				if(i == j){
					rot_acc += fabs(1-acc(i,j));
					rot_diff += fabs(1-diff(i,j));
				}else{
					rot_acc += fabs(acc(i,j));
					rot_diff += fabs(diff(i,j));
				}
			}
		}

		for(int i = 0; i < 3; i++){
			float d = fabs(diff(0,3));
			if(d < trans_maxd){
				int binid = float(histogram_size)*(d/trans_maxd);
				trans_diff_hist[binid]++;
			}
		}
//		printf("%5.5i -> trans_acc:%5.5f rot_acc:%5.5f trans_diff: %5.5f rot_diff: %5.5f\n",frames.size(),trans_acc,rot_acc,trans_diff,rot_diff);
//		printf("hist: ");for(int i = 0; i < histogram_size; i++){printf("%i ",trans_diff_hist[i]);}printf("\n");

		matching.back().push_back(frames.size()-1);
		//printf("ODOMETRY: adding match from %i to %i\n",frames.size(),frames.size()-1);
		//if acceleration too big, set new keyframe

		//printf("%5.5i -> trans_acc:%5.5f rot_acc:%5.5f trans_diff: %5.5f rot_diff: %5.5f\n",frames.size(),trans_acc,rot_acc,trans_diff,rot_diff);

		int last = keyframes.back();
		if((frames.size() - last) > 30000 || rot_acc > 0.5 || trans_acc > 0.05 || trans_diff > 0.01 || rot_diff > 0.1 ){

			int current = frames.size()-1;
			int mid = (last+current)/2;

			for(int i = last+1; i < current; i++){matching.at(i).push_back(current);}//All nodes match to keyframe behind and infront
/*
			if(last != mid && current != mid){
				keyframes.push_back(mid);
				iskeyframe[mid] = true;
				keyframe_index[mid] = keyframes.size()-1;
			}
*/
			keyframes.push_back(current);
			iskeyframe[current] = true;
			keyframe_index[current] = keyframes.size()-1;
			//guess = local_guess;
			//kfscores.push_back(log(icpOdom2.getCovariance().determinant()));

			kfpprpose = pprpose;

			for(unsigned int i = 0; i < last; i++){
				if(true || !iskeyframe[i]){
					if(d_depths[i] != 0){
						cudaFree(d_depths[i]);
						d_depths[i] = 0;
					}
					if(d_rgbs[i] != 0){
						cudaFree(d_rgbs[i]);
						d_rgbs[i] = 0;
					}
				}
			}
			printf("keyframe found: %5.5i last keyframe now %5.5i\n",frames.size(),keyframes.back());
		}else{
			matching.back().push_back(keyframes.back());
		}
		printf("keyframe: %5.5i frame %5.5i\n",keyframes.back(),frames.size());

		p = kfpprpose;
		poses.push_back(p.cast<double>());
		for(unsigned int id = last; true && id < poses.size()-1; id++){
			Eigen::Affine3f rp = Eigen::Affine3f((poses.at(id).inverse()*poses.back()).cast<float>());
			Eigen::Affine3f irp = rp.inverse();
			Eigen::Vector3f tcurr	= rp.translation();
			Eigen::Vector3f tinv	= irp.translation();
			Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Rcurr = rp.rotation();
			Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Rinv = irp.rotation();
			Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);
			float3& device_tcurr = device_cast<float3>(tcurr);
			Mat33&  device_Rinv = device_cast<Mat33> (Rinv);
			float3& device_tinv = device_cast<float3>(tinv);

scalemat(
	device_Rcurr, device_tcurr, device_Rinv, device_tinv,
	frame->camera->fx, frame->camera->fy,frame->camera->cx,frame->camera->cy,
	d_scalingmat,d_scalingmatweight,
	640, 480,d_depth, d_depths[id],
	threads,blocks);
			//if(id % 1 == 0 || id == poses.size()-1){normalizeDS(d_scalingmat, 640*480, 0.001 * 640.0 * 480.0, threads, blocks);}
		}
	}


	d_rgbs.push_back(d_rgb);
	d_depths.push_back(d_depth);

	frames.push_back(frame);
	masks.push_back(mask);

}
using namespace g2o;

Eigen::Matrix4d getMat(Eigen::Isometry3d p){
	return (Eigen::Affine3d(p)).matrix();
}

Eigen::Isometry3d getIso(Eigen::Matrix4d p){
	Eigen::Affine3d aff (p);
	Eigen::Isometry3d iso;
	iso = aff.rotation();//.rotation();
	iso.translation() = aff.translation();
	return iso;
}

void ModelUpdater2::refine(){
	int threads = 128;
	int blocks = 96;

	Eigen::Vector3f trans;
	Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot;

	if(keyframes.back() != frames.size()-1){
		keyframes.push_back(frames.size()-1);
		iskeyframe[keyframes.back()] = true;
		keyframe_index[keyframes.back()] = keyframes.size()-1;
	}

	ICPOdometry icpOdom2 (640, 480, frames[0]->camera->cx, frames[0]->camera->cy, frames[0]->camera->fx, frames[0]->camera->fy);//, 0.01, sin(10.f * 3.14159254f / 180.f));
	std::vector<VertexSE3 *> verts;
/*
	for(unsigned int i = 0; i < keyframes.size(); i++){
		int id = keyframes[i];

		VertexSE3 *vc = new VertexSE3();
		vc->setEstimate(getIso(poses.at(id)));
		vc->setId(i);      // vertex id
		g->addVertex(vc);
		if (i==0){vc->setFixed(true);}
		verts.push_back(vc);

		//icpOdom2.initICPModel((unsigned short *)frames[id]->depth.data, 	20.0f, poses[id].cast<float>());

		Eigen::Matrix4f start = poses[id].cast<float>();
		icpOdom2.initICPModel((unsigned short *)frames[id]->depth.data, 	20.0f, start);

		printf("%i -> %i\n",i,id);
		printf("-----------------\n");
		for(unsigned int j = 0; j < matching.at(i).size(); j++){
			int match_id = matching.at(id).at(j);
			bool kf = iskeyframe.at(match_id);
			if(kf){
				printf("current frame %i to old frame %i (keyframe)\n",id,match_id,kf);

				Eigen::Matrix4f stop = poses[match_id].cast<float>();

				icpOdom2.initICP(	(unsigned short *)frames.at(match_id)->depth.data,			20.0f);
				trans	= start.topRightCorner(3, 1);
				rot		= start.topLeftCorner(3, 3);

				icpOdom2.getIncrementalTransformation(trans, rot, threads, blocks, stop);//poses.back().cast<float>());//guess);
				Eigen::Matrix4f res			= Eigen::Matrix4f::Identity();
				res.topLeftCorner(3, 3)		= rot;
				res.topRightCorner(3, 1)	= trans;

				Eigen::Matrix4f motion	= start.inverse()*res;
				Eigen::Matrix4f diff	= res.inverse()*stop;

				Eigen::Matrix<double, 6,6> m = Eigen::Matrix<double, 6,6>::Identity();

				EdgeSE3 * e = new EdgeSE3();

				Eigen::Isometry3d iso = getIso(motion.cast<double>());
				e->setMeasurement(iso);
				e->setInformation(m);
				//delete e;
				e->vertices()[0]=verts[keyframe_index[match_id]];
				e->vertices()[1]=vc;
				g->addEdge(e);
				//std::cout << stop		<< std::endl << std::endl << res	<< std::endl;
				//std::cout << motion	<< std::endl << std::endl << diff	<< std::endl;
			}else{
				printf("current frame %i to old frame %i \n",id,match_id);
			}
		}
	}
*/

	for(unsigned int i = 0; i < frames.size(); i++){
		VertexSE3 *vc = new VertexSE3();
		vc->setEstimate(getIso(poses.at(i)));
		vc->setId(i);      // vertex id
		g->addVertex(vc);
		if (i==0){vc->setFixed(true);}
		verts.push_back(vc);
	}

	for(unsigned int i = 0; i < frames.size(); i++){
		int id = i;
		printf("id %i\n",id);

		printf("number matching: %i\n",matching.at(i).size());

		Eigen::Matrix4f start = poses[id].cast<float>();
		icpOdom2.initICPModel((unsigned short *)frames[id]->depth.data, 	20.0f, start);

		for(unsigned int j = 0; j < matching.at(i).size(); j++){
			int match_id = matching.at(id).at(j);
			//if(abs(match_id - id) == 1){continue;}
			printf("current frame %i to %i frame\n",id,match_id);

			Eigen::Matrix4f stop = poses[match_id].cast<float>();

			icpOdom2.initICP(	(unsigned short *)frames.at(match_id)->depth.data,			20.0f);
			trans	= start.topRightCorner(3, 1);
			rot		= start.topLeftCorner(3, 3);

            icpOdom2.getIncrementalTransformation(trans, rot, threads, blocks, Eigen::Affine3f(stop));//poses.back().cast<float>());//guess);
			Eigen::Matrix4f res			= Eigen::Matrix4f::Identity();
			res.topLeftCorner(3, 3)		= rot;
			res.topRightCorner(3, 1)	= trans;

			int count = icpOdom2.lastICPCount;


			Eigen::Matrix4f motion	= start.inverse()*res;
			Eigen::Matrix4f diff	= res.inverse()*stop;
			Eigen::Matrix<double, 6,6> m = Eigen::Matrix<double, 6,6>::Identity();

			double trans_diff = sqrt(diff(0,3)*diff(0,3)+diff(1,3)*diff(1,3)+diff(2,3)*diff(2,3));
			double rot_diff = 0;
			for(int i = 0; i < 3; i++){
				for(int j = 0; j < 3; j++){
					if(i == j){	rot_diff += fabs(1-diff(i,j));}
					else{		rot_diff += fabs(  diff(i,j));}
				}
			}

			//if(trans_diff < 0.02 && rot_diff < 0.1){
				EdgeSE3 * e = new EdgeSE3();
				Eigen::Isometry3d iso = getIso(motion.cast<double>().inverse());
				e->setMeasurement(iso);
				e->setInformation(m);
				e->vertices()[0]=verts[match_id];
				e->vertices()[1]=verts[id];
				g->addEdge(e);
			//}
			//std::cout << stop		<< std::endl << std::endl << res	<< std::endl;
			//std::cout << motion	<< std::endl << std::endl << diff	<< std::endl;
		}
	}

	for(int i = 0; i < keyframes.size(); i++){
		int id = keyframes[i];
		VertexSE3 *vc = verts[id];

		Eigen::Matrix4f start = poses[id].cast<float>();
		icpOdom2.initICPModel((unsigned short *)frames[id]->depth.data, 	20.0f, start);

		for(int j = i-2; j >= 0; j--){
			int match_id = keyframes[j];

			Eigen::Matrix4f stop = poses[match_id].cast<float>();

			icpOdom2.initICP(	(unsigned short *)frames.at(match_id)->depth.data,			20.0f);
			trans	= start.topRightCorner(3, 1);
			rot		= start.topLeftCorner(3, 3);

            icpOdom2.getIncrementalTransformation(trans, rot, threads, blocks, Eigen::Affine3f(stop));//poses.back().cast<float>());//guess);
			Eigen::Matrix4f res			= Eigen::Matrix4f::Identity();
			res.topLeftCorner(3, 3)		= rot;
			res.topRightCorner(3, 1)	= trans;

			int count = icpOdom2.lastICPCount;
			printf("%i %i current frame %i to %i frame count: %i\n",i,j,id,match_id,count);
			Eigen::Matrix4f motion	= start.inverse()*res;
			Eigen::Matrix4f diff	= res.inverse()*stop;
/*
			if(count > 60000){
				Eigen::Matrix<double, 6,6> m = Eigen::Matrix<double, 6,6>::Identity();
				EdgeSE3 * e = new EdgeSE3();
				Eigen::Isometry3d iso = getIso(motion.cast<double>().inverse());
				e->setMeasurement(iso);
				e->setInformation(m);
				e->vertices()[0]=verts[match_id];
				e->vertices()[1]=vc;
				g->addEdge(e);
			}
*/
		}
	}


	g->setVerbose(true);
	g->initializeOptimization();
	printf("nr verts: %i\n",g->activeVertices().size());
	printf("nr edges: %i\n",g->activeEdges().size());
	g->optimize(100);
	for(unsigned int i = 0; i < poses.size(); i++){
		poses[i] = getMat(verts[i]->estimate());
	}
	g->clear();
}

ModelUpdater2::ModelUpdater2(){
	histogram_size = 50;
	trans_maxd = 0.02;
	trans_diff_hist = new int[histogram_size];
	for(int i = 0; i < histogram_size; i++){trans_diff_hist[i] = 0;}

	std::string strSolver = "lm_var";

	g = new g2o::SparseOptimizer();
	g2o::ParameterSE3Offset * odomOffset=new g2o::ParameterSE3Offset();
	odomOffset->setId(0);
	g->addParameter(odomOffset);

	BlockSolverX::LinearSolverType * linearSolver = new LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();
	BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);
	OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(solver_ptr);

	g->setAlgorithm(solver);

	unsigned int nr_pixels = 640*480;
	bias = 1000;

	cudaMalloc((void**)(&(d_scalingmat)),		sizeof(float)*nr_pixels);
	//cudaMemset(d_scalingmat, 0.001f,			sizeof(float)*nr_pixels);
	setFloat(d_scalingmat, nr_pixels, 0.001, 512,96);

	cudaMalloc((void**)(&(d_scalingmatweight)),	sizeof(float)*nr_pixels);
	setFloat(d_scalingmatweight, nr_pixels, bias, 512,96);
}

ModelUpdater2::~ModelUpdater2(){}

void ModelUpdater2::saveToFB(std::string path){
	printf("Saving in: %s\n",path.c_str());

	std::ofstream myfile;
	myfile.open(path.c_str());

	char buf[1000];
	for(int i = 0; i < poses.size(); i++){
		//printf("pose: %i\n",i);

		Eigen::Affine3d a(poses[i]);
		Eigen::Quaterniond qr(a.rotation());
		double timestamp = frames[i]->capturetime;//frames.at(i)->input->rgb_timestamp;
		float tx = a(0,3);
		float ty = a(1,3);
		float tz = a(2,3);
		float qx = qr.x();
		float qy = qr.y();
		float qz = qr.z();
		float qw = qr.w();
		int n = sprintf(buf,"%f %f %f %f %f %f %f %f\n",timestamp,tx,ty,tz,qx,qy,qz,qw);
		myfile << buf;
	}
	myfile.close();

	path+=".kf";
	std::ofstream myfile2;
	myfile2.open(path.c_str());

	for(int i = 0; i < keyframes.size(); i++){
		//printf("pose: %i\n",i);

		Eigen::Affine3d a(poses[keyframes[i]]);
		Eigen::Quaterniond qr(a.rotation());
		double timestamp = frames[keyframes[i]]->capturetime;//frames.at(i)->input->rgb_timestamp;
		float tx = a(0,3);
		float ty = a(1,3);
		float tz = a(2,3);
		float qx = qr.x();
		float qy = qr.y();
		float qz = qr.z();
		float qw = qr.w();
		int n = sprintf(buf,"%f %f %f %f %f %f %f %f\n",timestamp,tx,ty,tz,qx,qy,qz,qw);
		myfile2 << buf;
	}
	myfile2.close();
}
/*
void ModelUpdater2::fuse(Model * model2, Eigen::Matrix4d guess, double uncertanity){
	//Blind addidtion of new model to old model...
	for(unsigned int i = 0; i < model2->frames.size();i++){
		model->frames.push_back(model2->frames[i]);
		model->masks.push_back(model2->masks[i]);
		model->relativeposes.push_back(guess*model2->relativeposes[i]);
	}
}

void ModelUpdater2::fuseData(FusionResults * f, Model * model1,Model * model2){}

void ModelUpdater2::refine(){}//No refinement behaviour added yet

void ModelUpdater2::show(bool stop){
	//printf("void ModelUpdater2::show()\n");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = model->getPCLcloud(1,false);
	viewer->removeAllPointClouds();
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud), "cloud");
	if(stop){
		viewer->spin();
	}else{
		viewer->spinOnce();
	}
	viewer->removeAllPointClouds();
}

//Backproject and prune occlusions
void ModelUpdater2::pruneOcclusions(){}
*/
}
