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

0.0.12 (2015-08-21)
-------------------

0.0.11 (2015-08-21)
-------------------
* Bugfix (introduced 2 commits ago ...)
* Publishing the dynamic clusters in the same frame of ref as the sweep (before metaroom registration)
* Moved metaroom_xml_parser headers into include/metaroom_xml_parser and adapted other projects
* Update README.md
* Update README.md
* Added parameter for minimum clustered object size
* Bugfix: publishing observations after registration (only used for visualization)
* Removed debug prints
* Removed some debug prints
* Not necessary anymore, as shorter sweeps can be triggered directly
* Made NDT registration the only option available to register sweeps to each other (the feature based registration wasn't used at the moment anyway)
* Operator overloading
* Loading corresponding registered transforms if the current sweep has different parameters
* Cleanup of unused parameters
* Methods to find the pan&tilt angles from int position and vice-versa
* Changes to use SweepParameters class
* Changes to use SweepParameters class
* Added SweepParameters class
* Added functions to save sweep parameters
* Setting member vars pan&tilt sweep parameters
* Contributors: Rares Ambrus, Rareș Ambruș

0.0.10 (2015-08-13)
-------------------
* updated changelogs
* Fixed return statement to get rid of warning
* Updated debug statement
* Fixed memory management bug; changed to use shared pointer; fixed parameter bug
* Using shared pointers
* Returning shared pointers and not objects; less memory management hassles
* Checking sweep match by waypoint ID and fixed bug in sweep saving
* Changed default parameter for saving metaroom steps to false
* updated sweep add task script with better parameters
* Added a launch file argument to switch between NDT registration and image feature registration for registering sweeps. Default is NDT
* Added service to clear and reinitialize the metarooms at specific (or all) waypoints
* Older sweeps moved to cache by checking waypoint id
* Find matching metaroom by waypoint id instead of centroid distance
* Publishing status message once an observation has been processed
* Fixed typo in package name
* Added support for getting only the latest dynamic clusters
* separate function for updating room with metaroom
* Bugfix when setting the room transform after registration
* Updating packages with new include paths
* Package renaming
* Updated dependencies for semantic_map package
* Removed debug prints
* merge from hydro-devel
* cleanup
* Minor code cleanup
* Enabled feature based registration between sweeps
* Adding methods to convert from the intermediate cloud to the pan and tilt PTU angles
* Added saving and loading of sweep parameters in the xml
* Fixed bug in finding the corresponding metaroom based on waypoint id
* Fixed semantic_map_node crash if supplied xml file is invalid
* Creating and updating metaroom using new registration
* debug print
* Sweep ORB registration
* Saving ORB features for metarooms
* Saving camera parameters after registration
* Bugfix for the case when a sweep xml file cannot be parsed
* removed services
* debug statement cleanup
* Bugfix
* Saving raw registration data
* new class
* Include guards
* utilities for manipulation sweeps
* Bugfix
* New classes & fixed exporting the library
* Computing orb features from intermediate images making up a sweep and saving them to disk
* Added registered transforms save/load routine
* Minor changes
* Added a method to clear the intermediate corrected camera parameters
* Room registration before updating metaroom now optional
* Removed debug statement
* Now storing corrected camera parameters for each intermediate point cloud (useful for correction)
* Changed the interior cloud size to 0.02m (from 0.01m). More robust to noise
* Lowered the max dynamic cluster size
* Changed launch file parameters to disable logging the intermediate clouds to the datacenter. Also enabled updating the metaroom with new observations
* Not updating the metaroom with an observation when having to remove/add too many points
* Fixed ndt registration bug
* Changed returned type of updateMetaRoom to make it easier to see the changes
* Point based way of checking for occlusions
* Added optional parameter specifying where to save room after using it to update metaroom
* Adding executable that parses a metric map folder structure and adds all the saved sweeps to mongodb
* Moved mongodb_interface class to the semantic_map package. Updated corresponding include files
* Checking that we actually got a point cloud from mongodb; useful if inserting the point returned failure (e.g. due to file size) but we would still look for it in the database
* Removed debug statements
* Clearing the intermediate registered transforms vector (useful when re-registering a sweep)
* Setting the root folder from a room xml file
* Saving and load intermediate registered transform
* Added registered transform
* Forward declarations
* SemanticMapSummaryParser no longer templated (not required). Some methods are still templated, todo remove the templates by avoiding instantiating objects of type MetaRoom or SemanticRoom (which are templated)
* Added roombase.hpp and moved implementation out of the header file
* Added semantic_map_node to CMakeLists
* Added metaroom_xml_parser.hpp and moved implementation out of the header file
* Added metaroom.hpp and moved implementation out of the header file
* Added metaroom_update_iteration class and moved definition and implementation from the metaroom class header
* Added room_xml_parser.hpp and moved implementation out of the header file
* Added room.hpp and moved implementation out of the header file
* Added explicit template instantiation for semantic_map package
* Fixed compilation dependency on messages generated by semantic_map
* Merge remote-tracking branch 'upstream/hydro-devel' into log_images
  Conflicts:
  cloud_merge/include/cloud_merge_node.h
* Bugfix - incrementing the intermediate images position counter
* Loading intermediate room images from disk
* Loading camera parameters and transforms for intermediate room imageS
* Bugfix - rgb camera info message
* Bugfix - adding intermediate images
* Saving intermediate position images into the room xml file
* When saving intermediate images: added transforms for both depth and rgb cameras as well as camera parameters for each intermediate position
* debugging
* Added debug message when saving intermediate images
* Merge remote-tracking branch 'upstream/hydro-devel' into log_images
* Storing individual images and saving them to disk
* Added cv_bridge dependency (for converting between sensor_msgs/Image and cv::Mat)
* Contributors: Marc Hanheide, Rares, Rares Ambrus, RaresAmbrus, rares

* Fixed return statement to get rid of warning
* Updated debug statement
* Fixed memory management bug; changed to use shared pointer; fixed parameter bug
* Using shared pointers
* Returning shared pointers and not objects; less memory management hassles
* Checking sweep match by waypoint ID and fixed bug in sweep saving
* Changed default parameter for saving metaroom steps to false
* updated sweep add task script with better parameters
* Added a launch file argument to switch between NDT registration and image feature registration for registering sweeps. Default is NDT
* Added service to clear and reinitialize the metarooms at specific (or all) waypoints
* Older sweeps moved to cache by checking waypoint id
* Find matching metaroom by waypoint id instead of centroid distance
* Publishing status message once an observation has been processed
* Fixed typo in package name
* Added support for getting only the latest dynamic clusters
* separate function for updating room with metaroom
* Bugfix when setting the room transform after registration
* Updating packages with new include paths
* Package renaming
* Updated dependencies for semantic_map package
* Removed debug prints
* merge from hydro-devel
* cleanup
* Minor code cleanup
* Enabled feature based registration between sweeps
* Adding methods to convert from the intermediate cloud to the pan and tilt PTU angles
* Added saving and loading of sweep parameters in the xml
* Fixed bug in finding the corresponding metaroom based on waypoint id
* Fixed semantic_map_node crash if supplied xml file is invalid
* Creating and updating metaroom using new registration
* debug print
* Sweep ORB registration
* Saving ORB features for metarooms
* Saving camera parameters after registration
* Bugfix for the case when a sweep xml file cannot be parsed
* removed services
* debug statement cleanup
* Bugfix
* Saving raw registration data
* new class
* Include guards
* utilities for manipulation sweeps
* Bugfix
* New classes & fixed exporting the library
* Computing orb features from intermediate images making up a sweep and saving them to disk
* Added registered transforms save/load routine
* Minor changes
* Added a method to clear the intermediate corrected camera parameters
* Room registration before updating metaroom now optional
* Removed debug statement
* Now storing corrected camera parameters for each intermediate point cloud (useful for correction)
* Changed the interior cloud size to 0.02m (from 0.01m). More robust to noise
* Lowered the max dynamic cluster size
* Changed launch file parameters to disable logging the intermediate clouds to the datacenter. Also enabled updating the metaroom with new observations
* Not updating the metaroom with an observation when having to remove/add too many points
* Fixed ndt registration bug
* Changed returned type of updateMetaRoom to make it easier to see the changes
* Point based way of checking for occlusions
* Added optional parameter specifying where to save room after using it to update metaroom
* Adding executable that parses a metric map folder structure and adds all the saved sweeps to mongodb
* Moved mongodb_interface class to the semantic_map package. Updated corresponding include files
* Checking that we actually got a point cloud from mongodb; useful if inserting the point returned failure (e.g. due to file size) but we would still look for it in the database
* Removed debug statements
* Clearing the intermediate registered transforms vector (useful when re-registering a sweep)
* Setting the root folder from a room xml file
* Saving and load intermediate registered transform
* Added registered transform
* Forward declarations
* SemanticMapSummaryParser no longer templated (not required). Some methods are still templated, todo remove the templates by avoiding instantiating objects of type MetaRoom or SemanticRoom (which are templated)
* Added roombase.hpp and moved implementation out of the header file
* Added semantic_map_node to CMakeLists
* Added metaroom_xml_parser.hpp and moved implementation out of the header file
* Added metaroom.hpp and moved implementation out of the header file
* Added metaroom_update_iteration class and moved definition and implementation from the metaroom class header
* Added room_xml_parser.hpp and moved implementation out of the header file
* Added room.hpp and moved implementation out of the header file
* Added explicit template instantiation for semantic_map package
* Fixed compilation dependency on messages generated by semantic_map
* Merge remote-tracking branch 'upstream/hydro-devel' into log_images
  Conflicts:
  cloud_merge/include/cloud_merge_node.h
* Bugfix - incrementing the intermediate images position counter
* Loading intermediate room images from disk
* Loading camera parameters and transforms for intermediate room imageS
* Bugfix - rgb camera info message
* Bugfix - adding intermediate images
* Saving intermediate position images into the room xml file
* When saving intermediate images: added transforms for both depth and rgb cameras as well as camera parameters for each intermediate position
* debugging
* Added debug message when saving intermediate images
* Merge remote-tracking branch 'upstream/hydro-devel' into log_images
* Storing individual images and saving them to disk
* Added cv_bridge dependency (for converting between sensor_msgs/Image and cv::Mat)
* Contributors: Rares, Rares Ambrus, RaresAmbrus, rares

0.0.9 (2014-11-23)
------------------

0.0.8 (2014-11-22)
------------------
* Initial README
* Contributors: RaresAmbrus

0.0.7 (2014-11-20)
------------------
* Added machine and user parameters
* Moved this launch file to the cloud_merge package (since it already depends on semantic_map, makes sense to have the launch file here). Also added starting the scitos_ptu metric map action server
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
