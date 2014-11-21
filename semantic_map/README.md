Package for building metarooms and extracting dynamic clusters
==========================

# Description 

This package takes room observations as they are constructed and extracts the corresponding metaroom and also computes dynamic clusters at each waypoint. 

Some data is stored on the disk, in the folder

```bash
~.semanticMap/metarooms
```

Data is published on the following topics:

* /local_metric_map/metaroom - RGBD point cloud corresponding to a meta-room
* /local_metric_map/dynamic_clusters - RGBD point cloud corresponding to the dynamic clusters.
 
# Semantic map nodes

To run the semantic map nodes do the following:

```bash
roslaunch semantic_map semantic_map.launch
```

Launch parameters:
* save_intermediate (yes/no)- whether to save the intermediate steps when updating metarooms to disk; default no
* 
