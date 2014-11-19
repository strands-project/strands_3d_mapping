^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package semantic_map
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2014-11-12)
------------------
* renaming
* renaming
* Changed filename to ndt_registration.h
* Changed filename
* Changed filename
* Changed include statement to reflect new header layout
* Changed install rule to reflect new header layout
* updated include paths in semantic_map to reflect the new include layout
* Moved headers in the include/semantic_map folder
* Contributors: Rares Ambrus

0.0.6 (2014-11-19)
------------------
* Deleting old data by default (instead of storing it in the cache to be uploaded to an ftp server)
* Bugfixes in loading metric map data from mongo and saving it on the disk
* Importing room observations from the databse and saving them to disk
* Fix for saving updated observations
* Contributors: Rares Ambrus, RaresAmbrus

0.0.5 (2014-11-12)
------------------
* 0.0.4
* updated changelogs
* renaming
* renaming
* Changed filename to ndt_registration.h
* Changed filename
* Changed filename
* Changed include statement to reflect new header layout
* Changed install rule to reflect new header layout
* updated include paths in semantic_map to reflect the new include layout
* Moved headers in the include/semantic_map folder
* Contributors: Jenkins, Rares Ambrus

0.0.3 (2014-11-11)
------------------
* removed deprecated call to setInputCloud
* Contributors: Rares Ambrus

0.0.2 (2014-11-11)
------------------
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Fixed some dependencies
* Removing ftp_upload info
* Contributors: Rares Ambrus

0.0.1 (2014-11-11)
------------------
* removed launching of the ftp upload action server
* removed launching of the ftp upload action server
* Fixed method for detecting oldest rooms in the cache
* Changed room centroid distance to 1m
* Added image_geometry dependency
* Added saving of camera parameters
* changes from upstream
* Fixed qt_build and qt_ros dependencies
* Removed package dependency on cloud_register
* removed dependency on cloud_register package
* Added ndt registration wrapper in the semantic_map package
* Fixed pcl dependency
* Added install targets for semantic_map and cloud_register
* Changed qt dependency
* Fixed license and maintainer email
* Fixed mongodb dependency
* First verison of mongodb dependency
* merge from upstream
* Renamed ros_datacentre to mongodb_store
* Bugfixing, mostly about saving and loading metaroom data
* Added flag -mno-avx to tackle assembler errors during compiling on some new Intel core processors
* More colors for dynamic clusters
* Merge branch 'hydro-devel' of https://github.com/RaresAmbrus/scitos_3d_mapping into hydro-devel
* Publishing the clustered differences with difference colors. Also made the publishers latching - i.e. they will republish the last published message to each new subscriber
* Y1Review working changes
* Saving pcd files only if they don't exist already (only for rooms, not for metarooms)
* Saving dynamic clusters in the room xml file and as a pcd file
* Added launch file parameters for the table top voxel size, observation voxel size and a parameter for the point distance cutoff. Also added a parameter to specify whether to update the metarooms with new room observations
* merged commit
* Added another stream containing the downsampled observation point cloud and changed the size of the voxel grid to get smaller observation point clouds
* Added services for waypoint based querying of observations, dynamic clusters and metarooms
* Minor bugfix in naming of saved data
* Added ftp upload action server to the launch file
* Ftp upload task client
* respawn set to true
* Minor bugfix related to deleting of metric map saved data
* Added function to move old data to a cache folder instead of deleting it
* Added a launch file parameter for saving to the database and fixed a bug.
* Logging intermediate point clouds to the database. Logging dynamic clusters to the database
* task registration on demand option
* Update README.md
* Added functionality to check how many instances of an observation have been saved, and remove some of them if there are too many
* Added a launch file for the entire local metric map system
* Added ros-hydro-qt-build as a dependency in package.xml and updated the readme.
* Updated the readme
* Added readme file for the semantic_map package
* Added functionality to remove previously saved metric map data, which can be set via the launch parameter cleanup (yes/no). The default behavior atm is to delete previously saved data, i.e. all metarooms will be created from scratch. This does not affect the creation of individual room observations
* Downsampling of observation point cloud using a 2cm voxel grid instead of 0.5 cm
* Metric map task client
* Added launch file parameters for configuring the saving of intermediate data (would be used fro debugging purposes)
* launch files
* Local metric map nodes: cloud_merge - processing depth & rgb frames / point clouds and merging them into room observations; cloud_register - utilities for ICP and NDT point cloud registration; semantic_map - creating and managing the local metric map, updating the map with new room observations, extracting dynamic clusters, maintaining the XML structure on the disk.
* Contributors: Bob, Johan Ekekrantz, Linda's sidekick, Nick Hawes, Nils Bore, Rares Ambrus, RaresAmbrus, cburbridge, cvapdemo, thomas.faeulhammer@tuwien.ac.at
