^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloud_merge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2014-11-12)
------------------
* renaming
* Changed filename
* Changed cloud_merge include statement by prefixing it with the package_name
* Contributors: Rares Ambrus

0.0.12 (2015-08-21)
-------------------

0.0.11 (2015-08-21)
-------------------
* Fixed some dependencies
* Moved cloud_merge headers into include/cloud_merge
* Update README.md
* Update README.md
* Update README.md
* Subsampling bvefore publishing
* Changed topics to work with openni2_launch
* Bugfix: publishing observations after registration (only used for visualization)
* Removed debug prints
* Added new sweep action server which takes a string (sweep type) as parameter
* Using corresponding registered transforms (if sweep parameters are different)
* Cleanup of unused parameters
* Added SweepParameters class
* Seeting sweep parameters as published by the PTU action server
* Contributors: Rares Ambrus, Rareș Ambruș

0.0.10 (2015-08-13)
-------------------
* updated changelogs
* Updated dependencies for cloud_merge package
* merge from hydro-devel
* bugfix
* Loading camera params from file
* Computing ORB features
* Register individual sweep clouds using precomputed sweep poses
* Added parameters for loading precalibrated sweep poses
* Changed launch file parameters to disable logging the intermediate clouds to the datacenter. Also enabled updating the metaroom with new observations
* Moved mongodb_interface class to the semantic_map package. Updated corresponding include files
* SemanticMapSummaryParser no longer templated (not required). Some methods are still templated, todo remove the templates by avoiding instantiating objects of type MetaRoom or SemanticRoom (which are templated)
* Added explicit template instantiation for cloud_merge package
* More informative print statement
* save_intermediate_images parameter defaults to false
* Renamed save_intermediate parameter to save_intermediate_clouds
* Added save_intermediate_images parameter to the launch file
* Saving intermediate images default value false
* Since we are using images no need to wait for the TF to reflect the PTU movement
* Bugfix - rgb camera info message
* Bugfix - loading proper cam info parameters for intermediate depth images
* bugfix cam info topic
* Added launch file parameters for depth and rgb camera parameter topics
* When saving intermediate images: added transforms for both depth and rgb cameras as well as camera parameters for each intermediate position
* Added one more input camera info topic for the depth camera
* Added debug message when saving intermediate images
* Added empty readme file
* Retrieving intermediate cloud images and adding them to the new room
* Contributors: Marc Hanheide, Rares, Rares Ambrus, RaresAmbrus

* Updated dependencies for cloud_merge package
* merge from hydro-devel
* bugfix
* Loading camera params from file
* Computing ORB features
* Register individual sweep clouds using precomputed sweep poses
* Added parameters for loading precalibrated sweep poses
* Changed launch file parameters to disable logging the intermediate clouds to the datacenter. Also enabled updating the metaroom with new observations
* Moved mongodb_interface class to the semantic_map package. Updated corresponding include files
* SemanticMapSummaryParser no longer templated (not required). Some methods are still templated, todo remove the templates by avoiding instantiating objects of type MetaRoom or SemanticRoom (which are templated)
* Added explicit template instantiation for cloud_merge package
* More informative print statement
* save_intermediate_images parameter defaults to false
* Renamed save_intermediate parameter to save_intermediate_clouds
* Added save_intermediate_images parameter to the launch file
* Saving intermediate images default value false
* Since we are using images no need to wait for the TF to reflect the PTU movement
* Bugfix - rgb camera info message
* Bugfix - loading proper cam info parameters for intermediate depth images
* bugfix cam info topic
* Added launch file parameters for depth and rgb camera parameter topics
* When saving intermediate images: added transforms for both depth and rgb cameras as well as camera parameters for each intermediate position
* Added one more input camera info topic for the depth camera
* Added debug message when saving intermediate images
* Added empty readme file
* Retrieving intermediate cloud images and adding them to the new room
* Contributors: Rares, Rares Ambrus, RaresAmbrus

0.0.9 (2014-11-23)
------------------
* Fixed bug: transforming intermediate cloud to global frame of ref before adding it to the merged cloud (it's still saved in the local frame of ref)
* Update README.md
* Initial README
* Contributors: Rares Ambrus, RaresAmbrus

0.0.8 (2014-11-22)
------------------
* Added empty readme
* Not launching the sweep action server from here anymore (doesn't start properly when located on another machine)
* Merge branch 'hydro-devel' of https://github.com/RaresAmbrus/strands_3d_mapping into hydro-devel
* Changed depth topic
* Contributors: Rares Ambrus, RaresAmbrus

0.0.7 (2014-11-20)
------------------
* Added machine and user parameters
* Saving intermediate clouds in the local frame of reference
* Added scitos_ptu run dependency
* Moved this launch file to the cloud_merge package (since it already depends on semantic_map, makes sense to have the launch file here). Also added starting the scitos_ptu metric map action server
* Logging to mongodb in a separate thread
* Contributors: Rares Ambrus

0.0.6 (2014-11-19)
------------------
* Deleting old data by default (instead of storing it in the cache to be uploaded to an ftp server)
* Logging to database enabled by default
* Added support for logging room sweeps into mongodb
* Contributors: Rares Ambrus, RaresAmbrus

0.0.5 (2014-11-12)
------------------
* 0.0.4
* updated changelogs
* renaming
* Changed filename
* Changed cloud_merge include statement by prefixing it with the package_name
* Contributors: Jenkins, Rares Ambrus

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
