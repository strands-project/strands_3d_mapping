Package for building local metric maps
==========================

# cloud_merge_node 

## Dependencies

Make sure you have Qt installed on the robot by getting the rqt packages:
```bash
sudo apt-get install ros-hydro-qt-ros
sudo apt-get install ros-hydro-rqt
sudo apt-get install ros-hydro-qt-build
```

## Description 

The `cloud_merge_node` acquires data from the RGBD sensor, as the PTU unit does a sweep, stopping at various positions as provided as input to the `scitos_ptu ptu_pan_tilt_metric_map.py` action server. (As an alternative, one can use the `do_sweep.py` action server from this package, which provides a higher level interface to doing a sweep). 

As the PTU stops at a position, a number of RGBD frames are collected and averaged, with the purpose of reducing noise. Each one of these frames are converted to the global frame of reference, and merged together to form an observation point cloud, which is further processed by the `semantic_map semantic_map_node` node. 

If the sweep intermediate positions have been calibrated (using the `calibrate_sweeps calibrate_sweep_as` action server) and the parameter `register_and_correct_sweep` is set to `true`, the collected sweeps are also registered. Note that this registration is for the intermediate point clouds making up the sweep, and not between two sweeps.

The observations are stored on the disk, in the folder

```bash
~.semanticMap/ 
```

To start the `cloud_merge_node`, run:
```
roslaunch cloud_merge cloud_merge.launch
```



### Input topics

 * `/ptu/log`  : this topic provides information about the sweep (i.e. parameters, started, position reached, finished).
 * `/current_node` : the waypoint id received on this topic is associated with the sweep collected

### Output topics    

* `/local_metric_map/intermediate_point_cloud` - RGBD point cloud corresponding to an intermediate position
* `/local_metric_map/merged_point_cloud` - merged point cloud with resolution specified by the `voxel_size_table_top` parameter
* `/local_metric_map/merged_point_cloud_downsampled` - merged point cloud with resolution specified by the `voxel_size_observation` parameter
* `/local_metric_map/depth/depth_filtered` - averaged depth frames corresponding to an intermediate position
* `/local_metric_map/rgb/rgb_filtered` - averaged RGB frames corresponding to an intermediate position
* `/local_metric_map/depth/camera_info` - camera info message corresponding to the image published on the `/local_metric_map/depth/depth_filtered` topic
* `/local_metric_map/rgb/camera_info` - camera info message corresponding to the image published on the `/local_metric_map/rgb/rgb_filtered` topic
* `/local_metric_map/room_observations` - string message containing the absolute path to the xml file corresponding to the collected sweep. This is used by the `semantic_map semantic_map_node` to trigger a Meta-Room update. 

### Parameters:

* `save_intermediate` (true/false)- whether to save the intermediate point clouds to disk; default `true`
* `cleanup` (true/false) - whether to remove previously saved data from `~/.semanticMap/`; default `false`
* `generate_pointclouds` (true/false) - generate point clouds from RGBD images or use the point clouds produced by the camera driver directly; default `true`. Note that setting `false` here has not been used for a while and might not work as expected. 
* `log_to_db` (true/false) - whether to log data to mongodb database; default `true`
* `voxel_size_table_top` (double) - the cell size to downsample the merged point cloud to before being published for detecting table tops; default `0.01 m`
* `voxel_size_observation` (double) - the cell size to downsample the merge point cloud to for visualisation purposes in rviz; default `0.03 m`
* `point_cutoff_distance` (double) - maximum distance after which data should be discarded when constructing the merged point cloud; default `4.0 m`
* `max_instances` (int) - how many instances of each observation to keep stored on disk; default `2`
* `input_cloud` - name of the topic for the RGBD input point clouds (this is used when `generate_pointclouds` is `false`); default `/depth_registered/points`. Note: this has not been used for a while and might not work as expected.
* `input_rgb` - name of the topic for the RGB image (this is used when generate_pointclouds is true); default `/head_xtion/rgb/image_color`
* `input_depth` - name of the topic for the depth image (this is used when generate_pointclouds is true); default `/head_xtion/depth/image_raw` 
* `input_caminfo` - name of the topic for the camera parameters (this is used when generate_pointclouds is true); default `/head_xtion/rgb/camera_info`

### Extracting data from mongodb

After logging some data, you can extract if from the database and saved it to disk in a folder of your choice using:

```bash
rosrun semantic_map load_from_mongo /path/where/to/save/
```

After extracting data from the database, you can load all the recorded observations in appropriate datastructures (containing the waypoint_id, merged cloud, individual point clouds, individual rgb and depth images and camera parameters):

```bash
rosrun metaroom_xml_parser load_multiple_files /path/where/to/load/from/
```

(Note the `/` at the end of the path in the command above). 

# do_sweeps.py

To start the action server manually:

```rosrun cloud_merge do_sweep.py```

Use:

```rosrun actionlib axclient.py /do_sweep```

This action server takes as input a string, with the following values defined: "complete", "medium", "short", "shortest". Internally the action server from `scitos_ptu ptu_action_server_metric_map.py` is used, so make sure that is running.

The behavior is the following:

If sweep type is `complete`, the sweep is started with parameters `-160 20 160 -30 30 30` -> 51 positions
If sweep type is `medium`, the sweep is started with parameters `-160 20 160 -30 30 -30` -> 17 positions
If sweep type is `short`, the sweep is started with parameters `-160 40 160 -30 30 -30` -> 9 positions
If sweep type is `shortest`, the sweep is started with parameters `-160 60 140 -30 30 -30` -> 6 positions (there might be blank areas with this sweep type, depending on the environment).
