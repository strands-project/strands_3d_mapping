This package provides an action server for performing object learning, as demonstrated at the Y2 review and published in (RA-L reference). It depends heavily on the STRANDS setup, in particular the robot PTU configuration and topological navigation.

# Dependencies

There are a large number of STRANDS package dependencies for this package:

- [`strands_navigation_msgs`](https://github.com/strands-project/strands_navigation/tree/indigo-devel/strands_navigation_msgs)
is required as monitored_navigation is used for all robot motion.
- [`object_view_generator`](https://github.com/cburbridge/scitos_3d_mapping/tree/hydro-devel/object_view_generator)
is used to generate the position for the robot to move to observe the object.
- [`ptu_follow_frame`](https://github.com/strands-project/scitos_apps/tree/hydro-devel/ptu_follow_frame)
is used to keep the object in the centre of the PTU throughout the learning.
- [`static_transform_manager`](https://github.com/strands-project/strands_apps/tree/indigo-devel/static_transform_manager)
is used to set a TF from for the object to allow tracking.
- [`scitos_ptu`](https://github.com/strands-project/scitos_apps/tree/hydro-devel/scitos_ptu)
is used to control the ptu.
- [`object_manager`](https://github.com/cburbridge/scitos_3d_mapping/tree/hydro-devel/object_manager)
is used to store and retrieve information about dynamic clusters.
- [`semantic_map_to_2d`](https://github.com/cburbridge/scitos_3d_mapping/tree/hydro-devel/semantic_map_to_2d)
is used to obtain a down-projected 3D map of the area around an object for planning.
- [`cloud_merge`](https://github.com/cburbridge/scitos_3d_mapping/tree/hydro-devel/cloud_merge)
is used to trigger a metric map action.
- [`camera_srv_definitions`](https://github.com/strands-project/v4r_ros_wrappers/tree/master/camera_tracker_srv_definitions)
is used to track the camera pose as the robot drives around.
- [`incremental_object_learning_srv_definitions`](https://github.com/strands-project/v4r_ros_wrappers/tree/master/incremental_object_learning_srv_definitions)
is used to stitch together views of the object and learn a model.
- [`rosbag_openni_compression`](https://github.com/strands-project/data_compression) is used to record a rosbag containing most of the published ros topics (including images from the `head_xtion`camera).
- [`recognition_srv_definitions`](https://github.com/strands-project/v4r_ros_wrappers/tree/master/recognition_srv_definitions)
is (not yet) used to re-recognise the learned object.


# Starting the server node 

A lot of nodes need to be running before this action server will start. If any are missing then a warning will be printed out.
The required nodes are as follows:

Standard nodes/launches:
- `strands_bringup / strands_core` is needed to provide MongoDB access with message store
- `strands_bringup / strands_robot` is needed for robot motion and to enable the PTU to be controlled by velocity
- `strands_bringup / strands_cameras` is needed to provide `head_xtion`, using the latest OpenNI2 launch approach.
- `strands_bringup / strands_navigation` is required for monitored_navigation etc.

Additional specific nodes to launch:
- metric mapping: `roslaunch semantic_map_launcher semantic_map.launch`
- metric map ptu: `rosrun scitos_ptu ptu_action_server_metric_map.py`
- map down projector: `rosrun semantic_map_to_2d semantic_map_2d_server`
- camera tracker: `rosrun camera_tracker camera_track_service  _camera_topic:=/head_xtion/depth_registered`
- object learning node: `rosrun incremental_object_learning incremental_object_learning_service`
- ptu tracker: `rosrun ptu_follow_frame ptu_follow.py`
- transform manager: `rosrun static_transform_manager static_tf_services.py`
- view planner: `rosrun object_view_generator view_points_service.py _map_topic:=/waypoint_map`

Finally the learn object action can be started:

```
rosrun learn_objects_action server.py _model_path:=/home/strands/test_models _record_run:=False
```

There node can be started with the following parameters:
- `model_path`: this is non-optional and must provide the directory where the learnt model should be stored.
- `rois_file`: this is the optional path to a file detailing the SOMA roi to use for which waypoint. If not 
provided then no SOMA regions will be needed. (see below for example file)
- `debug_mode`: by default this is False, but if set True then the action will step through the various parts of the 
learning process. At each stage the action server will need confirmation to proceed, supplied over a ros topic.
- `planning_method`: This selects which planning method to use to aquire the additional object views. Currently just the 
default 'ral16' is working, but there is a skeleton method 'infogain' that is ready to add the [nbv_planning](https://github.com/cburbridge/scitos_3d_mapping/tree/hydro-devel/nbv_planning) code to.
- `record_run` : This denotes whether or not to record the ros topics as the robot navigates around a cluster collecting additional views. By default this is False. 

# Launch file

All the dependencies, including the `learn_objects_action` action server can be started with:

```
roslaunch learn_objects_action learn_objects_dependencies.launch model_path:=/path/to/models/folder 
```


# Triggering a learning session

The object learning is triggered by the action `/learn_object`. This takes the waypoint name as the only field in it's goal definition. This sould be the waypoint that the robot is already at when triggering the learning.

Once started, the robot will perform a short metric map sweep, calculate the difference between the new map and the previous one at that location, select a cluster to learn based on how many views it can get of it, then drive around the object acquiring views. These views will be stitched together by the incremental object learning code from V4r, and finally a model will be save in the specified (by a parameter) folder. 

# RViz montoring

There are several topics that can be monitored in RViz to get an idea of what is happening:
- `/waypoint_map` shows the free space that the robot can plan in. Add it as a costmap to make it easier to see ontop of the base map.
- `/object_view_goals : PoseArray` shows the view points that the robot will try and reach
- `/local_metric_map/dynamic_clusters : PointCloud2` shows the difference between this map and the last one. Once of the shown clusters will be the learning target.
- `/local_metric_map/observations : PointCloud2` shows the new observations of the target object as they arrive.


# Debug mode

When in debug mode the action server will wait for confirmation to proceed between states, and confirmation of which dynamic cluster to select. If running in debug mode then it is best to [look at the code](https://github.com/cburbridge/scitos_3d_mapping/blob/hydro-devel/learn_objects_action/src/learn_objects_action/metric_sweep.py#L51) to see what is required.

# SOMA Rois file format

The optional file detailing which ROI to use for which region should be of the following format:

```
WayPointName: 'conf/map/region_no`
OtherPointName: 'conf/map/another_region`
WayPointN: ''
```

where every waypoint is covered, ones that should not be constrained to a region are given empty strings.

# Limitations
 - Sometimes, if the object is too far behind the robot, the robot turning completely will be too fast and the camera tracker fail. This results in bad models due to failed registration.
 - If the object crosses the back of the robot while driving, then the PTU has to do a full 360 degree spin to keep tracking it. During this the camera tracker will likely fail. Therefore runs with objects infront of the waypoint are more likely to be nice.
 - If monitored navigation fails to move the robot, only one re-attempt is made. If that fails the action fails.
 - The PTU tilt angle is super restricted. Very often objects are too low down, so the PTU can not see them at a reasonable close distance to the object, resulting in tracking of the object without it actually being in view. Make sure objects to learn are at chest height.
