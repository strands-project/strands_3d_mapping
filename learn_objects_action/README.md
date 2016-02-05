This package provides an action server for performing object learning, as demonstrated at the Y2 review and published in (RA-L reference).

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
- [`recognition_srv_definitions`](https://github.com/strands-project/v4r_ros_wrappers/tree/master/recognition_srv_definitions)
is (not yet) used to re-recognise the learned object.


# Running 
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
rosrun learn_objects_action server.py _model_path:=/home/strands/test_models
```

There node can be started with the following parameters:
- `model_path`: this is non-optional and must provide the directory where the learnt model should be stored.
- `rois_file`: this is the optional path to a file detailing the SOMA roi to use for which waypoint. If not 
provided then no SOMA regions will be needed.
- `debug_mode`: by default this is False, but if set True then the action will step through the various parts of the 
learning process. At each stage the action server will need confirmation to proceed, supplied over a ros topic.
- `planning_method`: This selects which planning method to use to aquire the additional object views. Currently just the 
default 'ral16' is working, but there is a skeleton method 'infogain' that is ready to add the [nbv_planning](https://github.com/cburbridge/scitos_3d_mapping/tree/hydro-devel/nbv_planning) code to.

