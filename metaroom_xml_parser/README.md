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

* Parsing labelled data

The `load_labelled_data` application reads labelled data for observations taken at a particular waypoint. The waypoint id and the folder where the observations are stored are taken in as parameters.

```
rosrun metaroom_xml_parser load_labelled_data` /path/to/sweeps WayPointXYZ
```


## Utilities

A number of utilities are provided by this package, for easy data manipulation. The definitions can be seen in the file `load_utilities.h`

### Merged cloud utilities

The complete cloud datatype is:

```template <class PointType> boost::shared_ptr<pcl::PointCloud<PointType>>```

The utilities for loading only the merged cloud are:
* `loadMergedCloudFromSingleSweep` # returns one cloud
* `loadMergedCloudFromMultipleSweeps` # returns a vector of merged clouds, one for each sweep
* `loadMergedCloudForTopologicalWaypoint` # same as above
 
### Intermediate cloud utilities

The intermediate cloud datatype is:
```
    template <class PointType>
    struct IntermediateCloudCompleteData
    {
        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>  vIntermediateRoomClouds;
        std::vector<tf::StampedTransform>                           vIntermediateRoomCloudTransforms;
        std::vector<image_geometry::PinholeCameraModel>             vIntermediateRoomCloudCamParams;
        std::vector<cv::Mat>                                        vIntermediateRGBImages; // type CV_8UC3
        std::vector<cv::Mat>                                        vIntermediateDepthImages; // type CV_16UC1
    };
```

The utilities for loading the intermediate clouds are:
* `loadIntermediateCloudsFromSingleSweep`                  # just the point clouds
* `loadIntermediateCloudsCompleteDataFromSingleSweep`      # complete data, with transforms and images
* `loadIntermediateCloudsFromMultipleSweeps`
* `loadIntermediateCloudsCompleteDataFromMultipleSweeps`
* `loadIntermediateCloudsForTopologicalWaypoint`
* `loadIntermediateCloudsCompleteDataForTopologicalWaypoint`
 

### Sweep XML utilities

The sweep XML is an `std::string`

The utilities for finding sweep XMLS are:
* `getSweepXmls` # takes a folder where to search as argument. Returns a `vector<string>`
* `getSweepXmlsForTopologicalWaypoint`

### Dynamic cluster utilities

The dynamic clusters type is:

```template <class PointType> std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>```

The dynamic cluster  utilities are:
* `loadDynamicClustersFromSingleSweep`
* `loadDynamicClustersFromMultipleSweeps`
* `loadDynamicClustersForTopologicalWaypoint`
 

### Labelled data utilities

The labelled data type is:

```
    template <class PointType>
    struct LabelledData
    {
        boost::shared_ptr<pcl::PointCloud<PointType>>               completeCloud;
        tf::StampedTransform                                        transformToGlobal;
        tf::Vector3                                                 sweepCenter;
        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>  objectClouds;
        std::vector<std::string>                                    objectLabels;
        boost::posix_time::ptime                                    sweepTime;
        std::string                                                 waypoint;

    };
```

The labelled data utilities are:
* `loadLabelledDataFromSingleSweep`





