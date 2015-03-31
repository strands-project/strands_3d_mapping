------------- WARNING - SURF features disabled in this version ----------------

Section 1: License
Section 2: Dependencies
Section 3: BUILD
Section 4: SOURCE CODE
Section 5: EXAMPLES
Section 6: TODOLIST

-------------------------Section 1: License------------------------------------

License: BSD

-------------------------Section 2: Dependencies-------------------------------

Dependencies required:
1: Install ros hydro from:	http://wiki.ros.org/hydro/Installation/
2: Install ros-pcl with:	apt-get install ros-hydro-perception-pcl
3: Install ros-opencv2:		apt-get install ros-hydro-opencv2

Optional(recommended):
4: Install ros openni: 		apt-get install ros-hydro-openni-launch
-------------------------Section 3: BUILD--------------------------------------
Place in catkin src folder.
Use catkin_make from catkin folder.

-------------------------Section 4: SOURCE CODE--------------------------------
found in installation directory + /src/
This section summarizes the contents of the src directory. For each folder a summary of the contents are provided with a short list important files that the user is likely to interact with.

--
File: ekz.h
Info: To include library just include ekz.h. Includes of other files for the library.

--
Folder:core
Info: Contains core classes for the library.
Important Files:

Calibration.h		//Calibration class, controls focal length etc

FrameInput.h		//Contains the input of one frame. Got functions like getXYZ(int w, int h), getPointCloud() etc.

RGBDFrame.h			//Representation of a frame. Contains extracted information for the frame such as segmentation and keypoints etc.

Transformation.h	//Represents a transformation between two frames.

--
Folder:FeatureDescriptor
Info: Contains different types of feature descriptors used by the library.
Important files:
FeatureDescriptor.h //Base class for Feature descriptors, contains functions such as distance(FeatureDescriptor * other_descriptor).

--
Folder:FeatureExtractor
Info: Contains different types of feature extractors to be chosen from by the library.
Important files:
FeatureExtractor.h	//Core class for feature extractors
OrbExtractor.h		//Class used to extract Orb keypoints
SurfExtractor.h		//Class used to extract Surf keypoints

--
Folder:FrameMatcher
Info: Contains registration algorithms that takes two frames as inputs(no initial guess given).
Important files:
FrameMatcher.h	//Core class for Frame matchers
AICK.h			//AICK base implementation without heuristic matching algorithm.
bowAICK.h		//AICK implementation with heuristic matching algorithm. Faster than AICK.h.

--
Folder:Map
Info: Contains Map3D classes.
Important files:
Map3D.h			//Basic map class. Contains many usefull functions to reduce the complexity for the user. Registers frames added sequentially.

--
Folder: mygeometry
Info: Contains geometry classes such as planes and points. Also contains Keypoints base class.

--
Folder:RGBDSegmentation
Info: Contains RGBD segmentation algorithms to be used on the RGBDFrames. Currently unused.

--
Folder:TransformationFilter
Info:Contains registration algorithms that takes two frames as inputs with an initial transformation and improves the solution. Currently unused.

--
Folder:apps
Info:Contains example code of how to use the library. See Section 5 for details.

-------------------------Section 5: EXAMPLES-----------------------------------
found in installation directory + /src/apps/
Unzip testdata.7z to gain access to some test data to run the examples with.

====================image_recorder.cpp====================
Summary:
	Records data and stores it in .pcd files from a the rostopic /camera/depth_registered/points
Input:
	path where to store recorded data
Output:
	png image pairs with RGBD data captured from a the rostopic /camera/depth_registered/points
USAGE:
	Run roscore
	Run roslaunch openni_launch openni.launch
	Run image_recorder program with an argument telling the recorder where to store the data

====================pcd_recorder.cpp====================
Summary:
	Records data and stores it in .png files from a the rostopic /camera/depth_registered/points
Input:
	path where to store recorded data
Output:
	captured pairs(depth and RGB) of .png files
USAGE:
	Run roscore
	Run roslaunch openni_launch openni.launch
	Run pcd_recorder program with an argument telling the recorder where to store the data

====================example_register_pcd_map.cpp====================
Summary:
	Minimalistic example for registering data provided in .pcd files sequentially using a Map3D object.
Input:
	a set of paths to pcd files
Output:
	.pcd file of aligned data
USAGE:
	Run example_register_pcd_map program with a set of paths to pcd files to be registed
	
====================example_register_images_map.cpp====================
Summary:
	Minimalistic example for registering data provided in .png files sequentially using a Map3D object.
Input:
	a path to a folder where png files with the correct names are located
Output:
	.pcd file of aligned data
USAGE:
	Run example_register_images_map program with an argument telling the program where to find the data
	
====================example_register_pcd_standalone.cpp====================
Summary:
	Example for registering data provided in .pcd files sequentially.
Input:
	a set of paths to pcd files
Output:
	.pcd file of aligned data
USAGE:
	Run example_register_pcd_map program with a set of paths to pcd files to be registed
	
====================example_register_images_standalone.cpp====================
Summary:
	Example for registering data provided in .png files sequentially.
Input:
	a path to a folder where png files with the correct names are located
Output:
	.pcd file of aligned data
USAGE:
	Run example_register_images_map program with an argument telling the program where to find the data

====================example_register_images_fast_map.cpp====================
Summary:
	Example for registering data provided in .png files sequentially using a Map3D object using ORB features and AICK with bag of words.
Input:
	a path to a folder where png files with the correct names are located
	a path+fileprefix to a folder where a pre trained bag of words model is located
Output:
	.pcd file of aligned data
USAGE:
	Run example_register_images_fast_map program with an argument telling the program where to find the data

====================example_bow_images.cpp====================
Summary:
	Example for training a bag of words model for data provided in .png files using a Map3Dbow.
Input:
	a path to a folder where png files with the correct names are located, a path/name for output, number of files to read and a number to controll how what part of the frames given will be used.
Output:
	.pcd file of aligned data
USAGE:
	Run example_bow_images program with a path to a folder where png files with the correct names are located, a path/name for output, number of files to read and a number to controll how what part of the frames given will be used.
	
-------------------------Section 1: TODOLIST   -------------------------
Use trees to speed up word association during frame generation.
Provide more maptypes.
Give option to provide initial guess for poses in map.
