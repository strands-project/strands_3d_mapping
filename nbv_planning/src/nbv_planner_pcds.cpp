/**
 * Runs the NBV algorithm on a supplied set of pointcloud files.
 *
 * Run this with a single command line argument:
 *
 * nbv_pcds path_to_yaml
 *
 * Where the YAML file supplied give the location of the pointclouds, target volume and sensor model. See test_files/out.yaml
 * for an example YAML file.
 *
 * The pointclouds supplied need to contain there position within the VIEWPOINT field. Compatible point clouds can be
 * captured using scripts/capture_some_clouds.py.
 *
 * The output of the program will be the view scores and the selected view. To visualise the progress in RViz, subscribe
 * to /nbv_planner/views, /nbv_planner/octomap, and /nbv_planner/volume.
 */
#include <ros/ros.h>
#include <nbv_planning/NBVFinderROS.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/common/eigen.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "nbv_planner");
    ros::NodeHandle n;
    YAML::Node config;
    boost::filesystem::path config_path;

    if (argc!=2){
        std::cout << "Must provide just a single argument, the path to the yaml file describing the PCDs.." << std::endl;
        return 1;
    } else {
        try {
            boost::filesystem::path p(argv[1]);
            if (!boost::filesystem::is_regular(p)) {
                std::cout << "Argument supplied is not a regular file!\n";
            }
            config_path = p.parent_path();
            config = YAML::LoadFile(argv[1]);
            // Do some checks here for the contents of the YAML :-)
        } catch (YAML::BadFile e) {
            std::cout << "Can't load specified yaml file, make sure first parameter is YAML description of PCDs." << std::endl;
            return 1;
        }
    }

    std::cout << "Loaded configuration from " << argv[1] << std::endl;
    std::cout << "Config path: " << config_path << std::endl;

    // Set up the camera parameters
    nbv_planning::SensorModel::ProjectionMatrix P;
    for(int i = 0; i < P.size(); ++i){
        P(i) = config["sensor_model"]["projection_matrix"][i].as<double>();
    }
    nbv_planning::SensorModel sensor_model(config["sensor_model"]["height"].as<int>(),
                                           config["sensor_model"]["width"].as<int>(),
                                           P,
                                           config["sensor_model"]["max_range"].as<float>(),
                                           config["sensor_model"]["min_range"].as<float>(),
                                           config["sensor_model"]["sub_sample"].as<int>());
    std::cout << "Loaded sensor model: " << sensor_model << std::endl;
    nbv_planning::NBVFinderROS planner(sensor_model, n);

    // Set the target volume
    Eigen::Vector3f origin, extents;
    for (unsigned i=0; i<3; i++){
        origin(i) = config["target_volume"]["origin"][i].as<float>();
        extents(i) = config["target_volume"]["extents"][i].as<float>();
    }
    nbv_planning::TargetVolume volume(config["target_volume"]["scale"].as<float>(),
                                      origin, extents);
    planner.set_target_volume(volume);
    planner.publish_volume_marker();
    std::cout << "Loded target volume: \n" << volume << std::endl;



    ROS_INFO("Loading clouds from  disk...");
    std::vector<Eigen::Affine3d> view_poses;
    std::vector<nbv_planning::NBVFinder::CloudPtr> view_clouds;
    for (YAML::const_iterator view=config["views"].begin();view!=config["views"].end();++view) {
        boost::filesystem::path pcd_path = config_path / view->as<std::string>();

        Eigen::Affine3d origin;
        nbv_planning::NBVFinder::CloudPtr cloud(new nbv_planning::NBVFinder::Cloud);

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(
                pcd_path.string(),
                *cloud) == -1) {
            PCL_ERROR ("Couldn't read file pcd file\n");
            return (-1);
        }

        origin = Eigen::Translation3d(cloud->sensor_origin_.block(0, 0, 3, 1).cast<double>()) *
                 cloud->sensor_orientation_.cast<double>();

        view_poses.push_back(origin);
        view_clouds.push_back(cloud);
    }


    // Setup planner and go through each view selecting the best each time
    planner.set_candidate_views(view_poses);
    planner.publish_views();
    bool got_view = true;
    double score;
//    unsigned int view =0;
    while (got_view) {
        unsigned int view;
        got_view = planner.choose_next_view(true, view, score);

        if (got_view) {
            ROS_INFO("Updating map");
            planner.update_current_volume(view_clouds[view], view_poses[view]);
            planner.publish_octomap();
            std::cout << "Number of unobserved cells=";
            std::flush(std::cout);
            std::cout << planner.count_unobserved_cells() << std::endl;
        }
//        view++;
//        if (view>view_clouds.size())
//            got_view=false;
    }

    std::cout << "Done, now spinning..." << std::endl;

    try {
        ros::spin();
    } catch (std::runtime_error &e) {
        ROS_ERROR("nbv_planner_server exception: %s", e.what());
        return -1;
    }

    return 0;
}
