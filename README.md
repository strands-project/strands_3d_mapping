# quasimodo
Quasimodo - Qu(erying) A(nd) S(patio-Temporal) I(ncremental) Mod(el building of) O(bjects)

## General Info \& Launching

The packages in this repo provides functionality for building a database of objects from observations
in an unsupervised manner. It builds on the Metaroom paradigm for data collection. The libraries
used for retrieving point clouds from several weeks of data collection can be found in
<https://github.com/strands-project/strands_3d_mapping/tree/hydro-devel/dynamic_object_retrieval>.

For launching the pipeline for building the representation and querying on the robot, do
```
roslaunch quasimodo_launch quasimodo.launch
```
This will launch the necessary nodes and launch files, both for maintaining a data base of object
models and for retrieving point clouds across weeks of exploration.

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

## quasimodo_brain

## quasimodo_data

## quasimodo_models

## quasimodo_msgs

## quasimodo_test
