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

This contains room observations (with or without individual point clouds) and meta-room information, each with a corresponding xml file. Whenever a new observation is available, the appropriate meta-room is loaded and updated. A statistical analyzer checks a set of previous observations for data which has been removed from the meta-room by mistake due to misalignment errors. The system is based on the assumption that the robots conducts patrol runs, and the patrol run number (or name) is used to keep track of room observations. The semantic map node reads this parameter from the parameter server under:

``` bash
/log_run_name
```

```TODO:``` add a component that automatically deduces the run number based on the number of observations available of the same place, thus eliminating the need for this parameter.

Data is published on the following topics:

* /local_metric_map/depth/depth_filtered - averaged depth frames corresponding to an intermediate position
* /local_metric_map/depth/rgb_filtered - averaged RGB frames corresponding to an intermediate position
* /local_metric_map/intermediate_point_cloud - RGBD point cloud corresponding to an intermediate position
* /local_metric_map/merged_point_cloud - RGBD point cloud corresponding to a complete observation
* /local_metric_map/metaroom - RGBD point cloud corresponding to a meta-room
* /local_metric_map/dynamic_clusters - RGBD point cloud corresponding to the dynamic clusters.

# Local metric map nodes

To run the metric map nodes do the following:

```bash
roslaunch cloud_merge cloud_merge.launch
```

Launch parameters:
* save_intermediate (yes/no)- whether to save the intermediate point clouds to disk; default no
* cleanup (yes/no) - whether to remove previously saved data; default yes

```bash
roslaunch semantic_map semantic_map.launch
```

Launch parameters:
* save_intermediate (yes/no)- whether to save the intermediate steps when updating metarooms to disk; default no

## Start the pan tilt action server:
```bash
rosrun scitos_ptu ptu_action_server_metric_map.py
```

Add a pan tilt sweep as a task
```bash
rosrun semantic_map metric_map_task_client.py
```

If doing multiple patrol runs and reobserving the same area, set the parameter ```/log_run_name``` to some value, ideally ```patrol_run_#```; the default is ```patrol_run_1```. This is used to sort the stored data.
