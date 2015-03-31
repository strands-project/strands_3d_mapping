#include <calibrate_sweeps/CalibrateSweepsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <boost/filesystem.hpp>
#include <tf_conversions/tf_eigen.h>

#include <semantic_map/room_xml_parser.h>
#include <semantic_map/reg_features.h>
#include <semantic_map/room_utilities.h>
#include <semantic_map/reg_transforms.h>
#include "load_utilities.h"
#include <strands_sweep_registration/RobotContainer.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef typename Cloud::Ptr CloudPtr;
using namespace std;

typedef actionlib::SimpleActionServer<calibrate_sweeps::CalibrateSweepsAction> Server;

void execute(const calibrate_sweeps::CalibrateSweepsGoalConstPtr& goal, Server* as)
{
    ROS_INFO_STREAM("Received calibrate message. Min/max sweeps: "<<goal->min_num_sweeps<<" "<<goal->max_num_sweeps);
    calibrate_sweeps::CalibrateSweepsResult res;

    std::string sweep_location;
    if (goal->sweep_location=="")
    {
        passwd* pw = getpwuid(getuid());
        std::string path(pw->pw_dir);

        path+="/.semanticMap/";
        sweep_location = path;
    } else {
        sweep_location = goal->sweep_location;
        sweep_location+="/";
    }
    if ( ! boost::filesystem::exists( sweep_location ) )
    {
        ROS_ERROR_STREAM("Could not find folder where to load sweeps from "+sweep_location);
        as->setAborted(res,"Could not find folder where to load sweeps from "+sweep_location);
        return;
    }

    ROS_INFO_STREAM("Sweeps will be read from "<<sweep_location);

    std::string save_folder;
    // default location
    passwd* pw = getpwuid(getuid());
    std::string path(pw->pw_dir);

    path+="/.ros/semanticMap/";
    save_folder = path;
    if ( ! boost::filesystem::exists( save_folder ) )
    {
        if (!boost::filesystem::create_directory(save_folder))
        {
            ROS_ERROR_STREAM("Could not create folder where to save calibration data "+save_folder);
             as->setAborted(res,"Could not create folder where to save calibration data "+save_folder);
             return;
        }
    }
    ROS_INFO_STREAM("Calibration data will be saved at: "<<save_folder);

    // Load sweeps
    vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmls<PointType>(sweep_location);

    if (matchingObservations.size() < goal->min_num_sweeps)
    {
        ROS_ERROR_STREAM("Not enough sweeps to perform calibration "<<matchingObservations.size());
         as->setAborted(res,"Not enough sweeps to perform calibration "+matchingObservations.size());
         return;
    }

    std::string saveLocation = goal->save_location;
    if (saveLocation == "")
    {
        saveLocation = sweep_location; // save in the same folder
    } else {
        saveLocation+="/";
        if ( ! boost::filesystem::exists( saveLocation ) )
        {
            if (!boost::filesystem::create_directory(saveLocation))
            {
                ROS_ERROR_STREAM("Could not create folder where to save calibration data "+saveLocation);
                 as->setAborted(res,"Could not create folder where to save calibration data "+saveLocation);
                 return;
            }
        }
    }
    ROS_INFO_STREAM("The registered sweeps will be saved at: "<<saveLocation);

    sort(matchingObservations.begin(), matchingObservations.end());
    reverse(matchingObservations.begin(), matchingObservations.end());

    // Initialize calibration class
    unsigned int gx = 17;
    unsigned int todox = 17;
    unsigned int gy = 3;
    unsigned int todoy = 3;
    RobotContainer * rc = new RobotContainer(gx,todox,gy,todoy);
    rc->initializeCamera(540.0, 540.0,319.5, 219.5, 640, 480);

    for (size_t i=0; i<goal->max_num_sweeps && i<matchingObservations.size(); i++)
    {
        // check if the orb features have already been computed
        std::vector<semantic_map_registration_features::RegistrationFeatures> features = semantic_map_registration_features::loadRegistrationFeaturesFromSingleSweep(matchingObservations[i], false);
        if (features.size() == 0)
        {
            // recompute orb
            SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(matchingObservations[i],true);
            unsigned found = matchingObservations[i].find_last_of("/");
            std::string base_path = matchingObservations[i].substr(0,found+1);
            RegistrationFeatures reg(false);
            reg.saveOrbFeatures<PointType>(aRoom,base_path);
        }
        rc->addToTrainingORBFeatures(matchingObservations[i]);
    }

    // perform calibration
    std::vector<Eigen::Matrix4f> cameraPoses = rc->train();
    std::vector<tf::StampedTransform> registeredPoses;

    for (auto eigenPose : cameraPoses)
    {
        tf::StampedTransform tfStamped;
        tfStamped.frame_id_ = "temp";
        tfStamped.child_frame_id_ = "temp";
        tf::Transform tfTr;
        const Eigen::Affine3d eigenTr(eigenPose.cast<double>());
        tf::transformEigenToTF(eigenTr, tfTr);
        tfStamped.setOrigin(tfTr.getOrigin());
        tfStamped.setBasis(tfTr.getBasis());
        registeredPoses.push_back(tfStamped);
    }
    std::string registeredPosesFile = semantic_map_registration_transforms::saveRegistrationTransforms(registeredPoses);
    registeredPoses.clear();
    registeredPoses = semantic_map_registration_transforms::loadRegistrationTransforms(registeredPosesFile);
    ROS_INFO_STREAM("Calibration poses saved at: "<<registeredPosesFile);

    double*** rawPoses = rc->poses;
    unsigned int x,y;
    std::string rawPosesFile = semantic_map_registration_transforms::saveRegistrationTransforms(rawPoses, rc->todox,rc->todoy);
    ROS_INFO_STREAM("Raw calibration data saved at: "<<rawPosesFile);
    res.calibration_file = registeredPosesFile;

    // correct used sweeps with the new transforms and camera parameters
    // create corrected cam params
    sensor_msgs::CameraInfo camInfo;
    camInfo.P = {rc->camera->fx, 0.0, rc->camera->cx, 0.0, 0.0, rc->camera->fy, rc->camera->cy, 0.0,0.0, 0.0, 1.0,0.0};
    camInfo.D = {0,0,0,0,0};
    image_geometry::PinholeCameraModel aCameraModel;
    aCameraModel.fromCameraInfo(camInfo);

    std::string camParamsFile = semantic_map_registration_transforms::saveCameraParameters(aCameraModel);
    ROS_INFO_STREAM("Camera parameters saved at: "<<camParamsFile);

    // update sweeps with new poses and new camera parameters

    SemanticRoomXMLParser<PointType> reg_parser(saveLocation);

    for (auto usedObs : matchingObservations)
    {
        SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(usedObs,true);
        auto origTransforms = aRoom.getIntermediateCloudTransforms();
        for (size_t i=0; i<origTransforms.size(); i++)
        {
            tf::StampedTransform transform = origTransforms[i];
            transform.setOrigin(registeredPoses[i].getOrigin());
            transform.setBasis(registeredPoses[i].getBasis());
            aRoom.addIntermediateCloudCameraParametersCorrected(aCameraModel);
            aRoom.addIntermediateRoomCloudRegisteredTransform(transform);
        }
        semantic_map_room_utilities::reprojectIntermediateCloudsUsingCorrectedParams<PointType>(aRoom);
        semantic_map_room_utilities::rebuildRegisteredCloud<PointType>(aRoom);
        // transform to global frame of reference
        tf::StampedTransform origin = origTransforms[0];
        CloudPtr completeCloud = aRoom.getCompleteRoomCloud();
        pcl_ros::transformPointCloud(*completeCloud, *completeCloud,origin);
        aRoom.setCompleteRoomCloud(completeCloud);
        string room_path = reg_parser.saveRoomAsXML(aRoom);
        ROS_INFO_STREAM("..done");
        // recompute ORB features
        unsigned found = room_path.find_last_of("/");
        std::string base_path = room_path.substr(0,found+1);
        RegistrationFeatures reg(false);
        reg.saveOrbFeatures<PointType>(aRoom,base_path);
    }

    delete rc;


    as->setSucceeded(res,"Done");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_sweeps_action_server");
  ros::NodeHandle n;
  Server server(n, "calibrate_sweeps", boost::bind(&execute, _1, &server), false);
  ROS_INFO_STREAM("Calibrate sweep action server initialized");
  server.start();
  ros::spin();
  return 0;
}
