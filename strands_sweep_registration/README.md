# strands_sweep_registration

Component that calibrates the intermediate positions of the point clouds making up a sweep. Currently only accepts sweeps of type `complete`, as recorded by the `do_sweep.py` action server from the `cloud_merge` package, i.e. with 51 intermediate positions. 

Internally it uses the Ceres optimization engine.
