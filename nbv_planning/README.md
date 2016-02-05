This package provides a implementation of the paper 

"A probabilistic framework for next best view estimation in a cluttered environment.", Potthas & Sukhatme, 2014

Instead of using s simple 3D array to store occupancy probabilities, an octomap is used. This more efficient and avoids the need for the 
markov random field gap filling method in the paper.

The code provides two interfaces, a C++ class and a ROS node. The C++ class is documented in source, with an example for running offline on PCD files.

# Running an example on PCDs
The example program "nbv_pcds" can be used to run offline with some logged PCD files.  Run this with a single command line argument:

```
rosrun nbv_planning nbv_pcds path_to_yaml
```

Where the YAML file supplied gives the location of the pointclouds, target volume and sensor model. See `test_files/out.yaml`
for an example YAML file.
The pointclouds supplied need to contain the pose of the camera within the VIEWPOINT field. Compatible point clouds can be
captured using `scripts/capture_some_clouds.py`.
The output of the program will be the view scores and the selected view. To visualise the progress in RViz, subscribe
to `/nbv_planner/views`, `/nbv_planner/octomap`, and `/nbv_planner/volume`.

# Running and using the planning as a ROS node
This package provides a single ROS node that can be used to do NBV planning. To start the node:

```
rosrun nbv_planning nbv_server _camera_info_topic:=.ead_xtion/depth/camera_info
```
The camera info topic needs to be correct so that the planner can get the intrinsic parameters of the camera.

#### /nbv_planner/set_target_volume
This sets the volume that views should be selected for. The request takes the centroid of a bounding box, and 
the extents. See [the service definition here](https://github.com/cburbridge/scitos_3d_mapping/blob/hydro-devel/nbv_planning/srv/SetTarget.srv)
#### /nbv_planner/update
This updates the current knowledge of the target volume, so that the next best view can be selected based 
on updated information from the last selected view. See [the service definition here](https://github.com/cburbridge/scitos_3d_mapping/blob/hydro-devel/nbv_planning/srv/Update.srv)
#### /nbv_planner/select_next_view
This returns the view that should be used next, based on how much information gain is predicted to be achieved 
by it. This returns the view as a pose (for the camera), the index of the view and the score. As argument it takes a 
boolean to say if the selected view should be disabled after selection - i.e not selected again later. 
See [the service definition here](https://github.com/cburbridge/scitos_3d_mapping/blob/hydro-devel/nbv_planning/srv/SelectNextView.srv) 
#### /nbv_planner/set_views
This service sets the 'candidate' views that the planner should select from. Each view should be a geometry_msgs/Pose, which is the pose of the camera not the robot.
See [the service definition here](https://github.com/cburbridge/scitos_3d_mapping/blob/hydro-devel/nbv_planning/srv/SetViews.srv)


# RViz
There are some topics to visualise the planning:
- `/nbv_planner/octomap` : this is an octomap_msgs/Octomap, subscribe using the RViz plugin to see the current volume knowledge
- `/nbv_planner/volume` : this is a MarkerArray showing the target region to select views for
- `/nbv_planner/views` : this is a MarkerArray showing the candidate views the planner is working with. 
 
# Limitations

This code has not been thoroughly tested. In particular, there is likely to be bugs in relation to formulae (6) and (7), and the advantage of the method stated in the paper over "simple_inf_gain" was not so apparant in tests that I carried out. None the less, hopefully this package can be the basis for some better implementation or alternative method.
