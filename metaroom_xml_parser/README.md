Package for parsing saved room observations
==========================

# Description	

The `metaroom_xml_parser` package is used to parse previously saved room observations. The data will be read into an appropriate data structure containing: merged point cloud, individual point clouds, individual RGB and depth images and corresponding camera parameters. 

# Usage

* Parsing one file

The `load_single_file` application reads in one room observation.

```
rosrun metaroom_xml_parser load_single_file /path/to/xml
```

* Parsing multiple observations

The 'load_multiple_files' application reads in multiple room observations and returns a vector. It takes as input the root folder where the observations are stored. 

```
rosrun metaroom_xml_parser load_multiple_files /path/to/folder
```


