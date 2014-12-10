Package for building local metric maps
==========================

# Dependencies

Make sure you have Qt installed on the robot by getting the rqt packages:
```bash
sudo apt-get install ros-hydro-qt-ros
sudo apt-get install ros-hydro-rqt
sudo apt-get install ros-hydro-qt-build
```

# Description 

The local metric map consists of a series of meta-rooms, each corresponding to a different location. A meta-room contains only those parts of the scene which are observed to be static, and it is created incrementally as the robot re-observes the same location over time.

Some data is stored on the disk, in the folder

```bash
~.semanticMap/ 
```

This contains room observations (with or without individual point clouds) and meta-room information, each with a corresponding xml file. Whenever a new observation is available, the appropriate meta-room is loaded and updated. A statistical analyzer checks a set of previous observations for data which has been removed from the meta-room by mistake due to misalignment errors. The system is based on the assumption that the robots conducts patrol runs, and the patrol run number (or name) is used to keep track of room observations, which are stored inside the corresponding patrol folders. Both the patrol number and the observation ID are computed automatically. 

Data is published on the following topics:

* /local_metric_map/depth/depth_filtered - averaged depth frames corresponding to an intermediate position
* /local_metric_map/depth/rgb_filtered - averaged RGB frames corresponding to an intermediate position
* /local_metric_map/intermediate_point_cloud - RGBD point cloud corresponding to an intermediate position
* /local_metric_map/merged_point_cloud - RGBD point cloud corresponding to a complete observation


# Local metric map nodes

To run the metric map nodes do the following:

```bash
roslaunch cloud_merge cloud_merge.launch
```

Launch parameters:
* save_intermediate (true/false)- whether to save the intermediate point clouds to disk; default true
* cleanup (true/false) - whether to remove previously saved data; default false
* generate_pointclouds (true/false) - generate point clouds from RGBD images or use the point clouds produced by the camera driver directly; default true
* log_to_db (true/false) - whether to log data to mongodb database
* voxel_size_table_top (double) - the cell size to downsample the merged point cloud to before being published for detecting table tops; default 0.01 m
* voxel_size_observation (double) - the cell size to downsample the merge point cloud to for visualisation purposes in rviz; default 0.03 m
* point_cutoff_distance (double) - maximum distance after which data should be discarded when constructing the merged point cloud; default 4.0 m
* max_instances (int) - how many instances of each observation to keep stored on disk; default 2
* input_cloud - name of the topic for the RGBD input point clouds (this is used when generate_pointclouds is false); default /depth_registered/points
* input_rgb - name of the topic for the RGB image (this is used when generate_pointclouds is true); default /head_xtion/rgb/image_color
* input_depth - name of the topic for the depth image (this is used when generate_pointclouds is true); default /head_xtion/depth_registered/image_rect 
* input_caminfo - name of the topic for the camera parameters (this is used when generate_pointclouds is true); default /head_xtion/rgb/camera_info


You can also run 

```bash
roslaunch cloud_merge mapping.launch
```
This will start both the `cloud_merge` node and the `semantic_map` node. 

## Start the pan tilt action server:
```bash
rosrun scitos_ptu ptu_action_server_metric_map.py
```

# How to run and check that everything is working

Start the mapping nodes on the machine where you have connected the head camera:

```bash
roslaunch cloud_merge mapping.launch
```

Start the ptu action server on the computer where you have access to the scitos header:

```bash
rosrun scitos_ptu ptu_action_server_metric_map.py 
```

Add a sweep task to the scheduler (python):

```bash
def create_3d_scan_task(waypoint_name):
    task = Task(start_node_id=waypoint_name, end_node_id=waypoint_name, action='ptu_pan_tilt_metric_map', max_duration=rospy.Duration(240))
    task_utils.add_int_argument(task, '-160')
    task_utils.add_int_argument(task, '20')
    task_utils.add_int_argument(task, '160')
    task_utils.add_int_argument(task, '-25')
    task_utils.add_int_argument(task, '25')
    task_utils.add_int_argument(task, '25')
    return task
```
 
Execute a few sweeps. Some data will be stored on the disk, at `~.semanticMap/`, in particular a fixed number of observations (or sweep data) at each waypoint, determined by the value set in the `max_instances` parameter. Old data will be deleted automatically. 

In addition, if the parameter `log_to_db` is set to `true`, sweep data will be logged to the mongodb database in the `metric_maps` database. 

After logging some data, you can extract if from the database and saved it to disk in a folder of your choice using:

```bash
rosrun semantic_map load_from_mongo /path/where/to/save/
```

After extracting data from the database, you can load all the recorded observations in appropriate datastructures (containing the waypoint_id, merged cloud, individual point clouds, individual rgb and depth images and camera parameters):

```bash
rosrun metaroom_xml_parser load_multiple_files /path/where/to/load/from/
```

(Note the `/` at the end of the path in the command above). The [file](  https://github.com/RaresAmbrus/strands_3d_mapping/blob/hydro-devel/metaroom_xml_parser/src/loadMultipleFilesMain.cpp)  should be a good starting point if you want to use the logged data.  
