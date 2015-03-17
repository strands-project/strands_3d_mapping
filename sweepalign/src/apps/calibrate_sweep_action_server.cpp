#include <strands_room/CalibrateSweepsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <boost/filesystem.hpp>


typedef actionlib::SimpleActionServer<strands_room::CalibrateSweepsAction> Server;

void execute(const strands_room::CalibrateSweepsGoalConstPtr& goal, Server* as)
{
    ROS_INFO_STREAM("Received calibrate message. Min/max sweeps: "<<goal->min_num_sweeps<<" "<<goal->max_num_sweeps);
    strands_room::CalibrateSweepsResult res;

    std::string sweep_location;
    if (goal->sweep_location=="")
    {
        passwd* pw = getpwuid(getuid());
        std::string path(pw->pw_dir);

        path+="/.semanticMap/";
        sweep_location = path;
    } else {
        sweep_location = goal->sweep_location;
    }
    if ( ! boost::filesystem::exists( sweep_location ) )
    {
        ROS_ERROR_STREAM("Could not find folder where to load sweeps from "+sweep_location);
        as->setAborted(res,"Could not find folder where to load sweeps from "+sweep_location);
        return;
    }

    ROS_INFO_STREAM("Sweeps will be read from "<<sweep_location);

    std::string save_folder;
    if (goal->calibration_folder=="")
    {
        // default location
        passwd* pw = getpwuid(getuid());
        std::string path(pw->pw_dir);

        path+="/.ros/semanticMap/";
        save_folder = path;

    } else {
        save_folder = goal->calibration_folder;
    }
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
  // Do lots of awesome groundbreaking robot stuff here
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
