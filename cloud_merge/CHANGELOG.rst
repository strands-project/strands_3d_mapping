^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloud_merge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2014-11-11)
------------------

0.0.2 (2014-11-11)
------------------

0.0.1 (2014-11-11)
------------------
* Removed ability to process input point clouds. Using images to generate the point clouds
* Logging intermediate data to the database disabled by default
* Saving intermediate data enabled by default
* removed input point cloud topic in the launch file. Not supported anymore as images are used as input directly
* reorganized the code into additional methods
* Some error checking
* changes from upstream
* Added image_transport as a dependency
* Fixed qt_build and qt_ros dependencies
* Removed package dependency on cloud_register
* Removed unnecessary dependency on cloud_register
* Fixed pcl dependency
* Set up install targets for cloud_merge
* removed unnecessary file
* Changed qt dependency
* Fixed license and maintainer email
* Fixed mongodb dependency
* Added dependency to message generation
* First verison of mongodb dependency
* merge from upstream
* Renamed ros_datacentre to mongodb_store
* Changed some methods to static
* Added flag -mno-avx to tackle assembler errors during compiling on some new Intel core processors
* Added launch file parameters for the table top voxel size, observation voxel size and a parameter for the point distance cutoff. Also added a parameter to specify whether to update the metarooms with new room observations
* merged commit
* Added another stream containing the downsampled observation point cloud and changed the size of the voxel grid to get smaller observation point clouds
* Merge branch 'hydro-devel' of https://github.com/RaresAmbrus/scitos_3d_mapping into hydro-devel
* Added services for waypoint based querying of observations, dynamic clusters and metarooms
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
  Conflicts:
  semantic_map/launch/mapping.launch
* Changed the number of observations to 5
* Added ftp upload action server to the launch file
* respawn set to true
* Launch parameter to cache or delete old data
* Added function to move old data to a cache folder instead of deleting it
* Handling preemption of the pan tilt sweep
* Added a launch file parameter for saving to the database and fixed a bug.
* Logging intermediate point clouds to the database. Logging dynamic clusters to the database
* Handling the case when the observation point cloud is empty - should only happen if the camera isn't running
* Automatically deducing the patrol run number and room id based on previously saved data
* Added max number of instances per observation as a launch file parameters and made cleanup of the saved semantic map information false by default in the launch file
* Added functionality to check how many instances of an observation have been saved, and remove some of them if there are too many
* Changed types of launch file parameters to bool
* Added launch file paremters to specify ros topics for input point cloud, input rgb image, input depth image and input camera info
* Added launch file parameter generate_pointclouds for cloud_merge node specifying whether to use the RGBD images from the sensor to generate point clouds or whether to use the point clouds generate by the openni driver directly
* Modification to use intermediate point clouds instead of generating them from RGBD images
* Added ros-hydro-qt-build as a dependency in package.xml and updated the readme.
* Added functionality to remove previously saved metric map data, which can be set via the launch parameter cleanup (yes/no). The default behavior atm is to delete previously saved data, i.e. all metarooms will be created from scratch. This does not affect the creation of individual room observations
* Changed the voxel grid cell size to 1cm for downsampling the merged point cloud
* Downsampling of observation point cloud using a 2cm voxel grid instead of 0.5 cm
* Added launch file parameters for configuring the saving of intermediate data (would be used fro debugging purposes)
* launch files
* Local metric map nodes: cloud_merge - processing depth & rgb frames / point clouds and merging them into room observations; cloud_register - utilities for ICP and NDT point cloud registration; semantic_map - creating and managing the local metric map, updating the map with new room observations, extracting dynamic clusters, maintaining the XML structure on the disk.
* Contributors: Johan Ekekrantz, Linda's sidekick, Nick Hawes, Rares Ambrus, cburbridge, cvapdemo, thomas.faeulhammer@tuwien.ac.at
