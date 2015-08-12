# calibrate_sweeps

Implements an action server for calibrating the intermediate sweep positions. Uses the `strands_sweep_registration` package internally. 

## Start node

```rosrun calibrate_sweeps calibrate_sweep.as```

## Call action server

```rosrun actionlib axclient.py /calibrate_sweeps```

## Action

```
#goal
int32 min_num_sweeps
int32 max_num_sweeps
string sweep_location
string save_location
---
#result
string calibration_file
---
#feedback
int32 percentage_done
```

* `min_num_sweeps` - the minimum number of sweeps needed to start the calibration
* `max_num_sweeps` - the maximum number of sweeps to use when doing the calibration
* `sweep_location` - where to look for the sweeps. If this argument is left empty, the default path is `~/.semanticMap/`
* `save_location` - where to save the registered sweeps after the calibration has finished. If this argument is left empty, the default path is the same as `sweep_location`

## Sweeps used

The calibration process uses only sweeps recorded with the type `complete` if using the `do_sweeps.py` action server from the `cloud_merge` package, i.e. with 51 positions. 

If using the `ptu_action_server_metric_map.py` action server from the `scitos_ptu` package, the parameters are `-160 20 160 -30 30 30`. 

Sweeps recorded with different parameters are ignored for the calibration. For registration, sweeps with different parameters are also processed if their parameters are a subset of the `complete` sweep type parameters (e.g. `comlpete` sweep type parameters are `-160 20 160 -30 30 30`; an example subset of those would be `-160 40 160 -30 30 30`, i.e. fewer pan stops).

## Results

The calibration results are saved in `~/.ros/semanticMap`. These are:

* `registration_transforms.txt` the result of the 51 transforms for the intermediate poses.
* `registration_transforms_raw.txt` legacy - contains the same data as above in a different format, needed by the `strands_sweep_registration` package. 
* `camera_params.txt` contains the optimized camera parameters. This is currently disabled, and the stored camera parameters should be the same as the input camera parameters.
* `sweep_paramters.txt` the sweep parameters used by the calibration (`-160 20 160 -30 30 30`)
 
