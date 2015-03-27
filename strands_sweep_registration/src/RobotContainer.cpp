#include "strands_sweep_registration/RobotContainer.h"

//#include "simple_xml_parser.h"
//#include "simple_summary_parser.h"
//#include <semantic_map/room_xml_parser.h>
#include <tf_conversions/tf_eigen.h>
#include <load_utilities.h>

#include "strands_sweep_registration/pair3DError.h"

typedef pcl::PointXYZRGB PointType;
typedef typename SimpleSummaryParser::EntityStruct Entities;

void RobotContainer::saveAllSweeps(std::string savePath)
{
    for (auto sweep: sweeps)
    {
        saveSweep(sweep, savePath);
    }
}

void RobotContainer::saveSweep(Sweep * sweep, std::string savePath){//std::vector<tf::StampedTransform> transforms)
   /* std::string sweep_xml = sweep->xmlpath;
    std::vector<Eigen::Matrix4f> poses = sweep->getPoseVector();

    std::cout<<"Sweep xml "<<sweep_xml<<std::endl;
    std::cout<<"No poses "<<poses.size()<<std::endl;

    auto room = SemanticRoomXMLParser<PointType>::loadRoomFromXML(sweep_xml, true);
    room.clearIntermediateCloudRegisteredTransforms();
    room.clearIntermediateCloudCameraParametersCorrected();
    auto original_transforms = room.getIntermediateCloudTransforms();
    auto intClouds = room.getIntermediateClouds();
    auto original_params = room.getIntermediateCloudCameraParameters();

    static tf::StampedTransform firstTransform = original_transforms[0];

    Camera * camera = sweep->frames[0][0]->camera;
    for (unsigned int i=0; i<poses.size(); i++){
        Eigen::Matrix4f pose = poses[i];
        // add reg transform and corrected camera parameters to sweep
        auto transform = original_transforms[i];
        tf::Transform tfTr;
        const Eigen::Affine3d eigenTr(pose.cast<double>());
        tf::transformEigenToTF(eigenTr, tfTr);
        tf::Transform combinedTransform = firstTransform * tfTr;

        sensor_msgs::CameraInfo camInfo;
        camInfo.P = {camera->fx, 0.0, camera->cx, 0.0, 0.0, camera->fy, camera->cy, 0.0,0.0, 0.0, 1.0,0.0};
        camInfo.D = {0,0,0,0,0};
        image_geometry::PinholeCameraModel aCameraModel;
        aCameraModel.fromCameraInfo(camInfo);
        room.addIntermediateCloudCameraParametersCorrected(aCameraModel);

        transform.setOrigin(tfTr.getOrigin());
        transform.setBasis(tfTr.getBasis());
        room.addIntermediateRoomCloudRegisteredTransform(transform);
    }

    SemanticRoomXMLParser<PointType> parser(savePath);
    parser.saveRoomAsXML(room);
    */
}

RobotContainer::RobotContainer(unsigned int gx_,unsigned int todox_,unsigned int gy_,unsigned int todoy_){
    gx = gx_;
    todox = todox_;
    gy = gy_;
    todoy = todoy_;

    inds = new unsigned int*[todox];
    for(unsigned int x = 0; x < todox; x++){inds[x] = new unsigned int[todoy];}

    for(unsigned int y = 0; y < gy; y++){//Camera moving forward
        for (unsigned int x = 0; x < gx ; x++){inds[x][y] = 0;}
        if(y % 2 == 0){
            for (unsigned int x = 0; x < gx ; x++){inds[x][y] = y*gx+x;}
        }else{
            for (unsigned int x = 0; x < gx ; x++){inds[x][y] = y*gx+gx-x-1;}
        }
    }

    poses = new double**[todox];
    for(unsigned int x = 0; x < todox; x++){
        poses[x] = new double*[todoy];
        for(unsigned int y = 0; y < todoy; y++){
            poses[x][y] = new double[6];
            for(unsigned int k = 0; k < 6; k++){poses[x][y][k] = 0;}
        }
    }

    width = 0;
    height = 0;

    shared_params	= new double[5];
    camera			= 0;
    rgb				= 0;
    depth			= 0;
}

RobotContainer::~RobotContainer(){
    for(unsigned int x = 0; x < todox; x++){delete[] inds[x];}
    delete[] inds;

    for(unsigned int s = 0; s < sweeps.size(); s++){delete sweeps.at(s);}
    if(camera != 0){		delete camera;}
    if(shared_params != 0){	delete shared_params;}
    if(rgb != 0){			delete rgb;}
    if(depth != 0){			delete depth;}

    for(unsigned int x = 0; x < todox; x++){
        for(unsigned int y = 0; y < todoy; y++){
            delete[] poses[x][y];
        }
        delete[] poses[x];
    }
    delete[] poses;
};

void RobotContainer::initializeCamera(double fx, double fy, double cx, double cy, unsigned int w, unsigned int h)
{
    std::cout<<"Initializing camera"<<std::endl;
    width = w;
    height = h;

    if (camera != 0)
    {
        delete camera;
    }

    camera = new Camera(fx, fy, cx, cy, width,	height);
    camera->version = 1;

    rgb		=	new float[3*width*height];
    depth	=	new float[	width*height];

    shared_params[0] = 1.0/fx;		//invfx
    shared_params[1] = 1.0/fy;		//invfy
    shared_params[2] = cx;
    shared_params[3] = cy;
    shared_params[4] = 0.1;
}

bool RobotContainer::addToTrainingORBFeatures(std::string path)
{
    std::cout<<"Adding ORB features to training from sweep "<<path<<std::endl;

    //    typedef semantic_map_registration_features::RegistrationFeatures RegFeatures;

    //    // load precomputed orb features
    std::vector<semantic_map_registration_features::RegistrationFeatures> features = semantic_map_registration_features::loadRegistrationFeaturesFromSingleSweep(path, false);
    if (features.size() == 0)
    {
        return false;
    }

    std::vector<Frame *> sweep_frames;
    sweep_frames.resize(todox*todoy);
    for(unsigned int y = 0; y < todoy; y++){
        for (unsigned int x = 0; x < todox ; x++){
            int index = inds[x][y];
            sweep_frames.at(y*todox+x) = new Frame(camera,features[index].keypoints, features[index].depths, features[index].descriptors);
            sweep_frames.at(y*todox+x)->framepos = inds[x][y];
        }
    }

    sweeps.push_back(new Sweep(todox, todoy, sweep_frames));
    alignedSweep.push_back(false);
    sweeps.back()->xmlpath = path;

    return true;

}

void RobotContainer::addToTraining(std::string path){
    printf("Not supported. Add to training using precomputed ORB features. \n");

/*

    SimpleXMLParser<PointType> simple_parser;
    SimpleXMLParser<PointType>::RoomData roomData = simple_parser.loadRoomFromXML(path);
    if(roomData.vIntermediateRoomClouds.size() < todox*todoy){return;}

    //	float fx = roomData.vIntermediateRoomCloudCamParams[0].fx();
    //	float fy = roomData.vIntermediateRoomCloudCamParams[0].fy();
    //	float cx = roomData.vIntermediateRoomCloudCamParams[0].cx();
    //	float cy = roomData.vIntermediateRoomCloudCamParams[0].cy();
    float fx = 540.0;//roomData.vIntermediateRoomCloudCamParams[i].fx();
    float fy = 540.0;//roomData.vIntermediateRoomCloudCamParams[i].fy();
    float cx = 319.5;//roomData.vIntermediateRoomCloudCamParams[i].cx();
    float cy = 239.5;//roomData.vIntermediateRoomCloudCamParams[i].cy();

    cv::Size res	= roomData.vIntermediateRoomCloudCamParams[0].fullResolution();
    height			= res.height;
    width			= res.width;

    if(camera == 0){
        camera = new Camera(fx, fy, cx, cy, width,	height);
        camera->version = 1;
        rgb		=	new float[3*width*height];
        depth	=	new float[	width*height];

        shared_params[0] = 1.0/fx;		//invfx
        shared_params[1] = 1.0/fy;		//invfy
        shared_params[2] = cx;
        shared_params[3] = cy;
        shared_params[4] = 0.1;
    }

    std::vector<Frame *> sweep_frames;
    sweep_frames.resize(todox*todoy);
    for(unsigned int y = 0; y < todoy; y++){
        for (unsigned int x = 0; x < todox ; x++){
            pcl::PointCloud<PointType>::Ptr clouddata = roomData.vIntermediateRoomClouds[inds[x][y]];
            for(unsigned int w = 0; w < width; w++){
                for(unsigned int h = 0; h < height; h++){
                    unsigned int ind = h*width+w;
                    unsigned int ind3 = 3*ind;
                    rgb[ind3+0] = clouddata->points.at(ind).r;
                    rgb[ind3+1] = clouddata->points.at(ind).g;
                    rgb[ind3+2] = clouddata->points.at(ind).b;
                    depth[ind] = clouddata->points.at(ind).z;
                }
            }
            sweep_frames.at(y*todox+x) = new Frame(camera,rgb,depth);
            sweep_frames.at(y*todox+x)->framepos = inds[x][y];
        }
    }

    sweeps.push_back(new Sweep(todox, todoy, sweep_frames));
    alignedSweep.push_back(false);
    sweeps.back()->idtag = roomData.roomWaypointId;
    sweeps.back()->xmlpath = path;
    */
}

std::vector< CostFunction * > getMatchesRansac(std::vector< ProblemFrameConnection * > & pc_vec, float weight = 1, float threshold = 0.005, int ransac_iter = 200000, int nr_points = 3){
    std::vector<int> owner;
    std::vector<int> match_id;
    std::vector< Eigen::Vector3f > src_points;
    std::vector< Eigen::Vector3f > dst_points;

    for(unsigned int i = 0; i < pc_vec.size(); i++){
        for(unsigned int j = 0; j < pc_vec.at(i)->src_points.size(); j++){
            owner.push_back(i);
            match_id.push_back(j);
            src_points.push_back(pc_vec.at(i)->src_points.at(j));
            dst_points.push_back(pc_vec.at(i)->dst_points.at(j));
        }
    }

    int nr_kp = src_points.size();
    std::vector< CostFunction * > errors;
    if(nr_kp == 0){return errors;}
    int nr_consistent = 0;


    Eigen::Matrix4f retpose;
    std::vector<int> bestmatching;
    for(int it = 0; it < ransac_iter; it++){
        //Sample points
        std::vector< int > indexes;
        std::vector< Eigen::Vector3f > src_samples;
        std::vector< Eigen::Vector3f > dst_samples;

        for(int j = 0; j < nr_points; j++){
            int ind;
            bool failed = true;
            while(failed){
                ind = rand()%nr_kp;
                failed = false;
                for(int k = 0; k < j; k++){
                    if(ind == indexes.at(k)){failed = true;}
                }
            }
            indexes.push_back(ind);
            src_samples.push_back(src_points.at(ind));
            dst_samples.push_back(dst_points.at(ind));
        }

        //Check consistency
        bool consistent = true;

        for(int j = 0; j < nr_points; j++){
            for(int k = j+1; k < nr_points; k++){
                float src_distance = (src_samples.at(j)-src_samples.at(k)).norm();
                float dst_distance = (dst_samples.at(j)-dst_samples.at(k)).norm();
                if(fabs(src_distance-dst_distance) > 0.02){consistent = false; break;break;}
            }
        }

        //Check inliers
        nr_consistent += consistent;
        if(consistent){
            pcl::TransformationFromCorrespondences tfc;
            for(int i = 0; i < nr_points; i++){tfc.add(dst_samples.at(i),src_samples.at(i),1);}
            Eigen::Affine3f ret = tfc.getTransformation();

            Eigen::Matrix4f pose = ret.matrix().inverse();

            std::vector<int> matching;
            for(int j = 0; j < nr_kp; j++){
                Eigen::Vector3f src_data = src_points.at(j);
                Eigen::Vector3f dst_data = dst_points.at(j);

                Eigen::Vector4f sd = pose*Eigen::Vector4f(src_data(0),src_data(1),src_data(2),1);
                Eigen::Vector4f dd = Eigen::Vector4f(dst_data(0),dst_data(1),dst_data(2),1);
                float t = threshold;//0.1*0.005*(dst_data(2)*dst_data(2)+src_data(2)*src_data(2));
                if((sd-dd).norm() < t){matching.push_back(j);}
            }

            //save if best
            if(matching.size() > bestmatching.size()){
                bestmatching = matching;
                retpose = pose;
                //printf("%i -> %i\n",it,matching.size());
            }
            if(nr_consistent == 30000){printf("break at %i\n",it);break;}
        }
    }

    printf("nr matches best: %i / %i consistent: %i / %i\n",int(bestmatching.size()),nr_kp,nr_consistent,ransac_iter);

    //getMatchesICP(pc_vec,retpose);

    for(int iter = 0; iter < 200; iter++){
        bestmatching.clear();
        pcl::TransformationFromCorrespondences tfc;
        int matching = 0;
        for(int j = 0; j < nr_kp; j++){
            Eigen::Vector3f src_data = src_points.at(j);
            Eigen::Vector3f dst_data = dst_points.at(j);

            Eigen::Vector4f sd = retpose*Eigen::Vector4f(src_data(0),src_data(1),src_data(2),1);
            Eigen::Vector4f dd = Eigen::Vector4f(dst_data(0),dst_data(1),dst_data(2),1);

            if((sd-dd).norm() < threshold){
                bestmatching.push_back(j);
                tfc.add(dst_data.head<3>(),src_data.head<3>(),1);
                matching++;
            }
        }
        if(iter % 50 == 0){printf("iteration: %i matches: %i\n",iter,matching);}

        Eigen::Affine3f ret = tfc.getTransformation();
        retpose = ret.matrix().inverse();
    }
    printf("nr matches best: %i / %i \n",int(bestmatching.size()),nr_kp);
    //exit(0);
    if(bestmatching.size() < 15){return errors;}
    for(unsigned int i = 0; i < pc_vec.size(); i++){
        pc_vec.at(i)->src_matches.clear();
        pc_vec.at(i)->dst_matches.clear();
    }

    for(unsigned int i = 0; i < bestmatching.size() && i < 1000; i++){
        ProblemFrameConnection * pc = pc_vec.at(owner.at(bestmatching.at(i)));
        int id = match_id.at(bestmatching.at(i));
        //printf("id: %i %i %i\n ",id,pc->src_matches.size(),pc->dst_matches.size());
        int src_kp_id = pc->src_possible_matches_id.at(id);
        int dst_kp_id = pc->dst_possible_matches_id.at(id);
        pc->src_matches.push_back(src_kp_id);
        pc->dst_matches.push_back(dst_kp_id);
        cv::KeyPoint src_kp = pc->src->keypoints.at(src_kp_id);
        cv::KeyPoint dst_kp = pc->dst->keypoints.at(dst_kp_id);
        double sz	= pc->src->keypoint_depth.at(src_kp_id);
        double dz	= pc->dst->keypoint_depth.at(dst_kp_id);
        CostFunction* err = new pair3DError(src_kp.pt.x,src_kp.pt.y,sz,dst_kp.pt.x,dst_kp.pt.y,dz,weight);
        errors.push_back(err);
    }

    return errors;
}

template <typename T> Eigen::Matrix<T,4,4> getMat(const T* const camera, int mode = 0){
    Eigen::Matrix<T,4,4> ret = Eigen::Matrix<T,4,4>::Identity();
    if(mode == 0){//yaw pitch roll tx ty tz
        Eigen::AngleAxis<T> yawAngle(camera[0], Eigen::Matrix<T,3,1>::UnitY());
        Eigen::AngleAxis<T> pitchAngle(camera[1], Eigen::Matrix<T,3,1>::UnitX());
        Eigen::AngleAxis<T> rollAngle(camera[2], Eigen::Matrix<T,3,1>::UnitZ());
        Eigen::Quaternion<T> q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix<T,3,3> rotationMatrix = q.matrix();
        ret.block(0,0,3,3) = rotationMatrix;
        ret(0,3) = camera[3];
        ret(1,3) = camera[4];
        ret(2,3) = camera[5];
    }
    return ret;
}

std::vector<Eigen::Matrix4f> RobotContainer::runInitialTraining(){
    ceres::Problem problem;
    Solver::Options options;
    options.max_num_iterations = 1500;
    options.minimizer_progress_to_stdout = true;
    options.num_linear_solver_threads = 11;
    options.num_threads = 11;
    Solver::Summary summary;

    //    double *** poses = new double**[todox];
    //    for(unsigned int x = 0; x < todox; x++){
    //        poses[x] = new double*[todoy];
    //        for(unsigned int y = 0; y < todoy; y++){
    //            poses[x][y] = new double[6];
    //            for(unsigned int k = 0; k < 6; k++){poses[x][y][k] = 0;}
    //        }
    //    }


    //1st forward X loop
    std::vector< std::vector< ProblemFrameConnection * > > x1_vec;
    x1_vec.resize(todoy);
    for(unsigned int s = 0; s < sweeps.size(); s++){
        printf("1st forward X loop: %i\n",s);
        Sweep * sweep = sweeps.at(s);
        for(unsigned int x = 0; x < todox-1; x++){
            for(unsigned int y = 0; y < todoy; y++){
                ProblemFrameConnection * pc = new ProblemFrameConnection(problem, sweep->frames[x][y],sweep->frames[x+1][y], shared_params, poses[x][y], poses[x+1][y]);
                x1_vec.at(y).push_back(pc);
            }
        }
    }

    for(unsigned int y = 0; y < todoy; y++){
        std::vector< CostFunction * > matches = getMatchesRansac(x1_vec.at(y));
        for(unsigned int i = 0; i < x1_vec.at(y).size(); i++){
            x1_vec.at(y).at(i)->addMatchesToProblem(problem, matches);
        }
    }

    Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";

    //1st forward Y loop
    std::vector< std::vector< ProblemFrameConnection * > > y1_vec;
    y1_vec.resize(todox);
    for(unsigned int s = 0; s < sweeps.size(); s++){
        printf("1st forward Y loop: %i\n",s);
        Sweep * sweep = sweeps.at(s);
        for(unsigned int x = 0; x < todox; x++){
            for(unsigned int y = 0; y < todoy-1; y++){
                ProblemFrameConnection * pc = new ProblemFrameConnection(problem, sweep->frames[x][y],sweep->frames[x][y+1], shared_params, poses[x][y], poses[x][y+1]);
                y1_vec.at(x).push_back(pc);
            }
        }
    }

    for(unsigned int x = 0; x < todox; x++){
        std::vector< CostFunction * > matches = getMatchesRansac(y1_vec.at(x));
        for(unsigned int i = 0; i < y1_vec.at(x).size(); i++){
            y1_vec.at(x).at(i)->addMatchesToProblem(problem, matches);
        }
    }

    Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";

    //2nd forward X loop
    std::vector< std::vector< ProblemFrameConnection * > > x2_vec;
    x2_vec.resize(todoy);
    for(unsigned int s = 0; s < sweeps.size(); s++){
        printf("2t forward X loop: %i\n",s);
        Sweep * sweep = sweeps.at(s);
        for(unsigned int x = 0; x < todox-2; x++){
            for(unsigned int y = 0; y < todoy; y++){
                ProblemFrameConnection * pc = new ProblemFrameConnection(problem, sweep->frames[x][y],sweep->frames[x+2][y], shared_params, poses[x][y], poses[x+2][y]);
                x2_vec.at(y).push_back(pc);
            }
        }
    }

    for(unsigned int y = 0; y < todoy; y++){
        std::vector< CostFunction * > matches = getMatchesRansac(x2_vec.at(y));
        for(unsigned int i = 0; i < x2_vec.at(y).size(); i++){
            x2_vec.at(y).at(i)->addMatchesToProblem(problem, matches);
        }
    }

    Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";

    //	camera->fx = 1.0/shared_params[0];	camera->fy = 1.0/shared_params[1];	camera->cx = shared_params[2];		camera->cy = shared_params[3];
    //	camera->print();
    //	for(unsigned int s = 0; s < sweeps.size(); s++){
    //		for(unsigned int x = 0; x < todox; x++){
    //			for(unsigned int y = 0; y < todoy; y++){
    //				sweeps.at(s)->poses[x][y] = (getMat(poses[0][0]).inverse()*getMat(poses[x][y])).cast<float>();
    //			}
    //		}
    //	}

    //Loop closure
    std::vector< std::vector< ProblemFrameConnection * > > loop_vec;
    loop_vec.resize(todoy);
    for(unsigned int s = 0; s < sweeps.size(); s++){
        printf("Loop closure: %i\n",s);
        Sweep * sweep = sweeps.at(s);
        for(unsigned int y = 0; y < todoy; y++){
            ProblemFrameConnection * pc = new ProblemFrameConnection(problem, sweep->frames[0][y],sweep->frames[todox-1][y], shared_params, poses[0][y], poses[todox-1][y],1,false);
            loop_vec.at(y).push_back(pc);
        }
    }

    for(unsigned int y = 0; y < todoy; y++){
        std::vector< CostFunction * > matches = getMatchesRansac(loop_vec.at(y),x1_vec.at(y).size());
        for(unsigned int i = 0; i < loop_vec.at(y).size(); i++){
            loop_vec.at(y).at(i)->addMatchesToProblem(problem, matches);
        }
    }
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    //Optimize camera parameter
    //optimizeCameraParams = true;
    //Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";

    camera->fx = 1.0/shared_params[0];	camera->fy = 1.0/shared_params[1];	camera->cx = shared_params[2];		camera->cy = shared_params[3];
    camera->print();    for(unsigned int s = 0; s < sweeps.size(); s++){
        for(unsigned int x = 0; x < todox; x++){
            for(unsigned int y = 0; y < todoy; y++){
                sweeps.at(s)->poses[x][y] = (getMat(poses[0][0]).inverse()*getMat(poses[x][y])).cast<float>();
            }
        }
    }

    std::vector<Eigen::Matrix4f> registeredPoses = sweeps.at(0)->getPoseVector(); // all the sweeps have the same poses. Return the first one
    return registeredPoses;

}

void RobotContainer::refineTraining(){}

std::vector<Eigen::Matrix4f> RobotContainer::train(){
    return runInitialTraining();
    //	refineTraining();
}

bool RobotContainer::isCalibrated(){return true;}



std::vector<Eigen::Matrix4f> RobotContainer::alignAndStoreSweeps(){

    // aligns sweeps to the first one

    using namespace Eigen;

    std::vector<Eigen::Matrix4f> toRet;

    if (sweeps.size() == 0)
    {
        std::cout<<"No sweeps to register."<<std::endl;
        return toRet;
    }

//    for (size_t i=0; i<todox; i++){
//        for (size_t j=0; j<todoy;j++){
//            for (size_t k=0; k<6;k++)
//            {
//                std::cout<<poses[i][j][k]<<std::endl;
//            }
//        }
//    }


    std::cout<<"Initializing sweep camera positions."<<std::endl;
    // setup sweep transforms
    for(unsigned int s = 0; s < sweeps.size(); s++){
        for(unsigned int x = 0; x < todox; x++){
            for(unsigned int y = 0; y < todoy; y++){
                Eigen::Matrix4f mat = (getMat(poses[0][0]).inverse()*getMat(poses[x][y])).cast<float>();
                sweeps.at(s)->poses[x][y] = mat;
            }
        }
    }

    Sweep * sweep = sweeps.at(0);
    for(unsigned int i = 1; i < sweeps.size(); i++){
        Sweep * s = sweeps.at(i);
        Eigen::Matrix4f m = sweep->align(s);
        toRet.push_back(m);
    }
    return toRet;
}

