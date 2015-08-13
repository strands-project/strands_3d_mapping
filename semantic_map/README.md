Package for building metarooms and extracting dynamic clusters
==========================

# semantic_map_node

## Description 

The local metric map consists of a series of meta-rooms, each corresponding to a different location. A meta-room contains only those parts of the scene which are observed to be static, and it is created incrementally as the robot re-observes the same location over time.

This package takes room observations as they are constructed by the `cloud_merge` package and extracts the corresponding metaroom and also computes dynamic clusters at each waypoint. 

Some data is stored on the disk, in the folder

```bash
~.semanticMap/metarooms
```

To start this node run:

```roslaunch semantic_map semantic_map.launch```

## Input topics

* `/local_metric_map/room_observations` : on this topic the observation xml is published. After receiving the xml, the appropriate Meta-Room will be loaded and updated.
 

## Output topics 

* `/local_metric_map/metaroom` - RGBD point cloud corresponding to a meta-room, published after a new observation has been processed. 
* `/local_metric_map/dynamic_clusters` - RGBD point cloud corresponding to the dynamic clusters, published after a new observation has been processed.

## ClearMetaroomService

This service resets the Meta-Rooms at specific waypoints.

Message type:
```
string[] waypoint_id
bool initialize
---
```
If `initialize` is set to `True`, the Meta-Rooms at the specific waypoints are re-initialized with the latest observations collected at those waypoints. Otherwise, the Meta-Rooms are just deleted. 

## Parameters

* `save_intermediate` : whether to save intermediate Meta-Room update steps to the disk. Default `false`
* `log_to_db` : log the Meta-Rooms to mongodb. Default `false`
* `update_metaroom` : update the Meta-Rooms (if `false` they will only be initialized and used as they are for dynamic cluster computation). Default `true`
* `min_object_size` : a dynamic cluster will be reported only if it has more points than this threshold. Default `500`
* `newest_dynamic_clusters` : compute dynamic clusters by comparing the latest sweep with the previous one (as opposed to comparing the latest sweep to the metaroom). Default `false`

# Export sweeps from mongodb to the disk

```
rosrun semantic_map load_from_mongo /path/where/to/export/data/
```

# Import sweeps from the disk into mongodb

```
rosrun semantic_map add_to_mongo /path/where/to/load/data/from/
```
 



