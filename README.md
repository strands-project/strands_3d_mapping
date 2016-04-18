# quasimodo
Quasimodo - Qu(erying) A(nd) S(patio-Temporal) I(ncremental) Mod(el building of) O(bjects)

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

### Nodes

* `quasimodo_retrieval_node`
* `quasimodo_retrieval_publisher`
* `quasimodo_retrieval_server`
* `quasimodo_visualization_server`
* `quasimodo_visualize_model`

### Launch files

* `retrieval.launch`

## quasimodo_optimization

### Nodes

* `optimizer.py`
* `plot_values.py`
* `rosbag_player.py`

### Launch files

* `optimizer.launch`

## quasimodo_brain

## quasimodo_data

## quasimodo_models

## quasimodo_msgs

## quasimodo_test
