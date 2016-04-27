# quasimodo
Quasimodo - Qu(erying) A(nd) S(patio-Temporal) I(ncremental) Mod(el building of) O(bjects)

## Build \& Setup

The packes distributed here do note require any special setup, simply build them in your catkin workspace.
Note that you will need the `2.0` branch of <https://github.com/strands-project/soma>
(which in turn requires an up-to-date version of <https://github.com/strands-project/mongodb_store>).
However, our packages require graphics for data fusion and visualization, the following describes how
to set that up on a computer without display.

### Headless Display \& ElasticFusion

This package is meant to run on a robot with no screen but with Nvidia graphics card.
This requires some setup every time the package is run (see the last two lines of code).

Right now, this package has a few dependencies that have to be installed manually.
In the future, our ElasticFusion fork will be replaced by a new version implemented in CUDA.
For now, you need to clone the repo
<https://github.com/stevenlovegrove/Pangolin>
anywhere in your computer home folder. Follow the build instructions in the readme.

Then you need to install our fork of ElasticFusion. For project members, please contact one of
the maintainers and we will give you access to the code. Note that you need at least
CUDA version 7 to run ElasticFusion. To get a graphics driver compatible with this version,
the easiest way (under Ubuntu 14.04) is to `sudo apt-get install nvidia-352`.

To run these programs on a headless computer, we need to perform the following steps.
First, do
```
sudo nvidia-xconfig -a --use-display-device=None --virtual=1280x1024
```
and then restart the computer. Further, we need to point to the new display that will be used
in the X server that will be used by typing
```
export DISPLAY=:0
```
If you are gonna run the programs multiple times, you might consider putting this in the `.bashrc`.
Note that this will mean that you have to set the `DISPLAY` again manually if you want to `ssh` with `-X`.
Then, every time you restarted the computer and run the nodes, you need to run
```
sudo service lightdm stop
sudo /usr/bin/X :0 &
```
This will kill the previous X server and start a new one that works in a headless state.

Apart from these acrobatics, all you should need to do is `catkin_make` in your workspace.

## General Info \& Launching

The packages in this repo provides functionality for building a database of objects from observations
in an unsupervised manner. It builds on the Metaroom paradigm for data collection. The libraries
used for retrieving point clouds from several weeks of data collection can be found in
<https://github.com/strands-project/strands_3d_mapping/tree/hydro-devel/dynamic_object_retrieval>.

For launching the pipeline for building the representation and querying on the robot, do
```
roslaunch quasimodo_launch quasimodo.launch data_folder:=/path/to/.semanticMap
```
Here, `/path/to/.semanticMap` is typically located in `~/.semanticMap`.
Please type the complete path.
This will launch the necessary nodes and launch files, both for maintaining a data base of object
models and for retrieving point clouds across weeks of exploration.

## Visualization

### Retrieval component

The easiest way to visualize the output of the retrieval (point cloud history search) pipeline
is to look at the image published on the `/quasimodo_retrieval/visualization` topic.
The leftmost image shows the masked RGB image of the query object and to the right are rendered views of the
ten closest matches represented as 3D surfel clouds.

You can manually trigger a search (i.e. without using the incremental object building framework)
of an object with additional views by starting
```
rosrun quasimodo_retrieval quasimodo_retrieve_observation
```
and then, in another window specifying the path to the xml of the additional views:
```
rostopic pub /object_learning/learned_object_xml std_msgs/String "data: '/path/to/.semanticMap/201422/patrol_run_56/room_0/2016-Apr-22 14:58:33.536964_object_0.xml'"
```
You can also use soma to visualize the queries over time.

### Model building component

@jekekrantz Add some stuff here!

# Detailed description of packages, nodes, launch files and messages

## retrieval_processing

This package runs in conjunction with the metaroom nodes online on the robot. As metarooms are collected,
the package continuously segments, extracts features and adds them to a feature vocabulary representation.
To launch the entire pipeline, do
```
roslaunch retrieval_processing processing.launch data_folder:=~/.semanticMap
```

### Nodes

* `retrieval_segmentation` - uses the `convex_segmentation` package, <https://github.com/strands-project/strands_3d_mapping/tree/hydro-devel/dynamic_object_retrieval/convex_segmentation>, to segment a point cloud with normals into smaller convex segments
* `retrieval_features` - uses the `dynamic_object_retrieval` package, <https://github.com/strands-project/strands_3d_mapping/tree/hydro-devel/dynamic_object_retrieval/dynamic_object_retrieval>, to extract PFHRGB features from the convex segments
* `retrieval_vocabulary` - uses the `k_means_tree` package, <https://github.com/strands-project/strands_3d_mapping/tree/hydro-devel/dynamic_object_retrieval/k_means_tree>, to organize the features into a hierarchical vocabulary representation for fast querying
* `retrieval_simulate_observations` - this node is used if you have already collected data that you want to process, or if you want to debug without doing sweeps on the robot

### Launch files

* `processing.launch` - this file launches all of the above nodes. By default it does not launch `retrieval_simulate_observations`, this is trigged with the parameter `simulate_observations:=true`.
It is important to set the parameter `data_folder:=/path/to/metarooms` to the location of the metarooms,
typically `~/.semanticMap`.

## quasimodo_retrieval

This package provides the nodes for retrieving point clouds from the memory created by `retrieval_processing`.
Launch everything simply with
```
roslaunch quasimodo_retrieval retrieval.launch vocabulary_path:=/path/to/vocabulary
```
where the vocabulary is most often located in `~/.semanticMap/vocabulary`.

### Nodes

* `quasimodo_retrieval_node` - provides the service `/query_normal_cloud` and subscribes to the topic `/models/new`. If something is published on the topic, it returns the result on `/retrieval_result`.
* `quasimodo_visualization_server` - this node simply subscribes to `/retrieval_result` and visualizes the query result using the tools in the package `object_3d_benchmark`, <https://github.com/strands-project/strands_3d_mapping/tree/hydro-devel/dynamic_object_retrieval/benchmark>. The resulting image is published on `/visualization_image`.
* `quasimodo_retrieve_observation` - allows the system to bypass the model building component, instead searching for results directly using the retrieval framework. Simply publish something like `rostopic pub /object_learning/learned_object_xml std_msgs/String "data: '/path/to/.semanticMap/201422/patrol_run_56/room_0/2016-Apr-22 14:58:33.536964_object_0.xml'"` to retrieve more views of that object.

### Other Nodes

* `quasimodo_visualize_model` - this node simply visualizes the topic `/models/new` by integrating it into a point cloud and showing a PCL visualizer
* `quasimodo_retrieval_publisher` - this node queries for all the labeled objects in a particular metaroom sweep, given by the parameter `data_path`.
* `quasimodo_retrieval_server` - a barebones version of `quasimodo_retrieval_node`, simply returns the retrieved clouds without loading any images or objects masks

### Launch files

* `retrieval.launch` - launches `quasimodo_retrieval_node`, `quasimodo_visualization_server` and a node for fusing the incoming RGB-D frames. Takes the parameter `vocabulary_path`, most often this is `~/.semanticMap/vocabulary`.

## quasimodo_optimization

This package is a general tool for optimizing some value by evaluating some metric that comes from analyzing a rosbag.
The tool package uses `dynamic_reconfigure` to play back the rosbag with different parameters and record the
values associated with the parameters.

Can be launched with
```
roslaunch quasimodo_optimizer optimizer.launch
```
Afterwards, run `plot_values.py` in the folder where you ran the launch file.

### Nodes

* `optimizer.py` - steps through the parameters and plays back the rosbags for every parameter configuration
* `rosbag_player.py` - an action server for playing back ros bags on demand
* `plot_values.py` - plots the values as a heat map in parameter space

### Launch files

* `optimizer.launch` - launches `optimizer.py` and `rosbag_player.py`.

## quasimodo_msgs

All the message and service types required for the Quasimodo framework.

### Message types

* `image_array.msg` - an array of images
* `int_array.msg` - an array of ints
* `model.msg` - a model object, consisting of point clouds, frames, camera parameters and relative transforms
* `retrieval_query.msg` - message type for querying `quasimodo_retrieval`
* `retrieval_result.msg`- message type result from querying `quasimodo_retrieval`
* `retrieval_query_result.msg` - a combined message for querying and result
* `rgbd_frame.msg` - RGD images, depth images and camera parameters
* `string_array.msg` - an array of strings

### Service types

* `cloud_from_model.srv` - service for fusing models into clouds
* `fuse_models.srv` - several models to one fused model
* `get_model.srv` - get model for identifier
* `index_frame.srv` - add frame to model data base
* `model_from_frame.srv` - turn frame into model
* `query_cloud.srv` - query retrieval using `retrieval_query.msg`
* `simple_query_cloud.srv` - query retrieval using `sensor_msgs/PointCloud2` pointcloud with normals
* `visualize_query.srv`- visualize a `retrieval_result.msg`

## quasimodo_brain

This package controlls the flow of data in the quasimodo system and maintains the database of object models. Relies heavily on the quasimodo_models package. The package also contains loaders for different formats of data, such as for example the metarooms.
```
roslaunch quasimodo_brain modelserver.launch
```

```
roslaunch quasimodo_brain robot_listener.launch
```

### Nodes

* `preload_object_data` - Reads data in the metarooms format. Uppon requests publishes data for the `modelserver`. Input: paths to a set of folders containing data. 


* `robot_listener` - Listenes to topic. Whenever it recieves the path to an xml file it reads data in the metarooms format from the file and publishes data for the `modelserver`. Input: topicname to listen at. 


* `modelserver` - Listens to data from input modules, uses the `quasimodo_models` package to register and merge models into more complete models and thereby maintain the database of objects. Input: '-v' for visualization, '-p /path/to/folder' to set a folder where the database is read/stored, '-m' initializes the database with the data from /path/to/folder, '-massreg_timeout value' sets the stopping time for the multiview registration, '-occlusion_penalty value' sets the penalty value for occlusions(controlling how likeley the database is to merge models).

### Launch files

* `modelserver.launch` - this file launches the modelserver node without the visualization flag.

* `robot_listener.launch` - this file launches the robot_listener node without the topicname set to "/some/topic".

* `brain.launch` - Launches the modelserver and the preload_object_data nodes. On automatic restart.

## quasimodo_models

This package is contains libraries for registering, splitting, merging and optimizing quasimodo object models. Quasimodo object models contain RGBDFrames, Segmentation masks, Depthimages and relative poses between the data for the frames.
