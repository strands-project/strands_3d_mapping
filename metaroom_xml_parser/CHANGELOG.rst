^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package metaroom_xml_parser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2014-11-12)
------------------
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* metaroom_xml_parser tf dependencies
* Contributors: Rares Ambrus

0.0.12 (2015-08-21)
-------------------

0.0.11 (2015-08-21)
-------------------
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Moved metaroom_xml_parser headers into include/metaroom_xml_parser and adapted other projects
* Update README.md
* Update README.md
* Update README.md
* Contributors: Rares Ambrus, Rareș Ambruș

0.0.10 (2015-08-13)
-------------------
* updated changelogs
* Removed debug print
* Added methods to load labelled data & example program
* Added flag to indicate whether to load intermediate cloud (useful when looking only for the transform)
* Fixed sweep sort (taking date into account
* Sorting sweeps based on patrol number and room number
* Also logging room run number (useful for reading objects  from mongo and saving them in the proper folder structure on the disk)
* Returning the room log start time as part of the sweep structure
* Returning the room log name as part of the sweep structure
* Cleanup
* debug statement cleanup
* bigfix
* Load orb features
* Minor changes
* Fixed include guard
* Simple xml parser now reads corrected camera parameters and registered tranforms as well
* Bugfix in loading all dynamic clusters at a particular waypoint
* Bugfixes
* Methods to parse and return dynamic cluster point clouds
* SimpleXmlParser now parses and returns the dynamic clusters clouds, if it's been saved
* utilities to get all the sweep xmls in a folder or for a particular waypoint
* Removed debug prints
* Safeguard against corrupted xmls
* Removed test utilities main file
* Simple summary parser now checks that found sweep xml files start with the correct element
* Simple summary parser now parses any folder structure looking for sweep xml files (not just the semantic map standard folder structure - date/patrol_run_#/room_#/room.xml)
* Added load_utilities.hpp and moved function implementations out of load_utilities.h
* Commented out visualization of intermediate position images after parsing
* Added routines for parsing intermediate sweep clouds and intermediate position images
* Added / at the end of the folder path (more robust this way)
* Added a vector of xml nodes that are to be parsed - this allows to load only particular fields (and thus point clouds or images) for sweeps. The fields covered are RoomCompleteCloud, RoomIntermediateCloud and IntermediatePosition
* utilities to load all the merged point clouds from sweeps saved in some particular folder, and to load all the merged clouds for all the sweeps taken at some waypoint
* Method to load the merged point cloud from a single sweep
* Added verbose flag to the simple xml parser (default value false)
* Removed unnecessary comments
* Added load_utilities files - will contain higher level functions which will allow parsing local metric maps and returning various components
* Renaming
* Added explicit template instantiation for metaroom_xml_parser package
* SimpleSummaryParser no longer templated
* Bugfix
* Printing intermediate image parameters
* Loading intermediate room images, transforms and parameters
* Contributors: Marc Hanheide, Rares, Rares Ambrus

* Removed debug print
* Added methods to load labelled data & example program
* Added flag to indicate whether to load intermediate cloud (useful when looking only for the transform)
* Fixed sweep sort (taking date into account
* Sorting sweeps based on patrol number and room number
* Also logging room run number (useful for reading objects  from mongo and saving them in the proper folder structure on the disk)
* Returning the room log start time as part of the sweep structure
* Returning the room log name as part of the sweep structure
* Cleanup
* debug statement cleanup
* bigfix
* Load orb features
* Minor changes
* Fixed include guard
* Simple xml parser now reads corrected camera parameters and registered tranforms as well
* Bugfix in loading all dynamic clusters at a particular waypoint
* Bugfixes
* Methods to parse and return dynamic cluster point clouds
* SimpleXmlParser now parses and returns the dynamic clusters clouds, if it's been saved
* utilities to get all the sweep xmls in a folder or for a particular waypoint
* Removed debug prints
* Safeguard against corrupted xmls
* Removed test utilities main file
* Simple summary parser now checks that found sweep xml files start with the correct element
* Simple summary parser now parses any folder structure looking for sweep xml files (not just the semantic map standard folder structure - date/patrol_run_#/room_#/room.xml)
* Added load_utilities.hpp and moved function implementations out of load_utilities.h
* Commented out visualization of intermediate position images after parsing
* Added routines for parsing intermediate sweep clouds and intermediate position images
* Added / at the end of the folder path (more robust this way)
* Added a vector of xml nodes that are to be parsed - this allows to load only particular fields (and thus point clouds or images) for sweeps. The fields covered are RoomCompleteCloud, RoomIntermediateCloud and IntermediatePosition
* utilities to load all the merged point clouds from sweeps saved in some particular folder, and to load all the merged clouds for all the sweeps taken at some waypoint
* Method to load the merged point cloud from a single sweep
* Added verbose flag to the simple xml parser (default value false)
* Removed unnecessary comments
* Added load_utilities files - will contain higher level functions which will allow parsing local metric maps and returning various components
* Renaming
* Added explicit template instantiation for metaroom_xml_parser package
* SimpleSummaryParser no longer templated
* Bugfix
* Printing intermediate image parameters
* Loading intermediate room images, transforms and parameters
* Contributors: Rares, Rares Ambrus

0.0.9 (2014-11-23)
------------------

0.0.8 (2014-11-22)
------------------

0.0.7 (2014-11-20)
------------------

0.0.6 (2014-11-19)
------------------
* Fixed bug in generating depth images from saved pointclouds
* Contributors: Rares Ambrus

0.0.5 (2014-11-12)
------------------
* 0.0.4
* updated changelogs
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* metaroom_xml_parser tf dependencies
* Contributors: Jenkins, Rares Ambrus

0.0.3 (2014-11-11)
------------------
* Merge pull request `#48 <https://github.com/strands-project/strands_3d_mapping/issues/48>`_ from RaresAmbrus/hydro-devel
  Removed deprecated function call
* Added maintainer details
* Removed pcl_ros include. Not needed
* Contributors: Marc Hanheide, Rares Ambrus

0.0.2 (2014-11-11)
------------------
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Fixed some dependencies
* Contributors: Rares Ambrus

0.0.1 (2014-11-11)
------------------
* Added readme
* renaming
* Contributors: Rares Ambrus
