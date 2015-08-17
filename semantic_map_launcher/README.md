# semantic_map_launcher  

This launches the metric mapping / semantic mapping nodes from the `strands_3d_mapping` repository.

```roslaunch semantic_map_launcher semantic_map.launch```

## Nodes started

* `cloud_merge`
* `semantic_map`
* `calibrate_sweep_as`
* `semantic_map_publisher`
* `objcet_manager`
* `do_sweep`

Note: the  `ptu_action_server_metric_map` node needs to be started separetely with:

```rosrun scitos_ptu ptu_action_server_metric_map.py```

## Parameters

The parameters accepted by the launch file are:

* `save_intermediate_clouds` : whether to save the intermediate point clouds from the sweeps to the disk. Default `true`
* `save_intermediate_images` : whether to save all the images making up an intermediate cloud to the disk (this takes a lot of space!!). Default `false`
* `log_to_db` : log the sweeps to mongodb. Default `true`
* `log_objects_to_db` : log the dynamic clusters to mongodb. Default `true`
* `cleanup` : at startup, delete everything in the `~/.semanticMap/` folder. Default `false` 
* `max_instances` : maximum number of sweeps, per waypoint to keep in the `~/.semanticMap/` folder. Default: `10`
* `cache_old_data` : if there are more sweeps per waypoint than the `max_instances` parameter, delete them or move them to the cache folder `~/.semanticMap/cache/`. Default  `false`, i.e. delete older sweeps.
* `update_metaroom` : update the metaroom with new sweeps. Default `true`
* `newest_dynamic_clusters` : compute dynamic clusters by comparing the latest sweep with the previous one (as opposed to comparing the latest sweep to the metaroom). Default `true`
* `min_object_size` : the minimum number of points for a cluster to be reported. Default `500`

