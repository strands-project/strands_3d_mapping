object_3d_retrieval
===================

This contains our implementation of a vocabulary tree (Nister & Stewenius) together with some modifications for our paper. It also includes some functions e.g. for extracting segments & features and querying on our data structure.

To use the system, follow the instructions in `scripts/menu.py`. There are submenus, for example for querying that can be run separately, i.e. `scripts/training_menu.py` and `scripts/querying_menu.py`.

## Instructions for stepping through menu.py

If you build the project and go to your build folder, you should see the
python scripts mentioned above. To use the scripts you will need data
captured according to the meta room description, defined in `strands_3d_mapping`.

When you run `menu.py`, you are first asked for the data path. This is the base
folder of your sweeps and it should contain folders with different dates
as names. Once you enter the name you should get the menu itself:

```
Please supply the data path: /home/nbore/Data/KTH_longterm_dataset_labels

Please make sure to do the following in order both for the noise data folder and for the annotated data. Note that you should choose the a or b option consistently.

Working on data path /home/nbore/Data/KTH_longterm_dataset_labels

1.  Set data path (if you want to change)
2.  Init sweep segment folders
3a. Create convex segments (alternative to 3b & 5b)
3b. Create convex segments and supervoxels (alternative to 3a & 5a)
4.  Extract PFHRGB features
5a. Segment keypoints into subsegments (alternative to 5b)
5b. Create PFHRGB feature for supervoxels (alternative to 5a)
6.  Extract SIFT from sweeps (for re-weighting)
7.  Vocabulary training menu
8.  Querying & benchmarking menu
9.  Exit

Please enter an option 1-9(a/b):
```

Now you should step through the numbers in order. Note that if you
choose e.g. `3a` you should also choose `5a`. Start by inputting `2`.
This will set up all the folders. Then choose either `3a` or `3b`. This will
perform a convex segmentation of the data. It will take a while. Then input
`4` to extract features, this will also take a while. The depending on if you
chose `a` or `b` previously, input `5a` or `5b`. Finally, input `6` to finish
the data processing by extracting `sift` features.

You are now ready to go on to training the vocabulary tree representation!

## Instructions for running training_menu.py

To go into the training menu, either input `7` in the previous menu or
run `training_menu.py` separately. This time you are asked for the path
to your vocabulary representation. Simply give the path to some empty folder:

```
Please supply the path to the vocabulary: /home/nbore/Data/KTH_longterm_dataset_labels_convex

Please make sure to do the following in order

Working on vocabulary path /home/nbore/Data/KTH_longterm_dataset_labels_convex

1. Set vocabulary path (if you want to change)
2. Initialize vocabulary folders and files
3. Build vocabulary tree representation
4. Exit

Please enter an option 1-4:
```

First, input `2`. This will ask you for paths to an annotated meta room data
set and another "noise" data set which does not need to be annotated.
It will also ask if you want a "standard" or "incremental" type vocabulary.
The two types are detailed in the paper, basically the standard works on
the convex segments extracted earlier while incremental is more flexible.
Once this is done, simply enter `3` to train and build the representation.
You are now ready to query the representation, see
<https://github.com/nilsbore/quasimodo/tree/master/quasimodo_retrieval>
for information on the ROS interface. Also check out example usage in
<https://github.com/nilsbore/quasimodo/tree/master/quasimodo_test>.

Happy querying!

## Dependencies

Right now, we do not use the debian package of `strands_3d_mapping` (see below),
instead you should compile
<https://github.com/RaresAmbrus/strands_3d_mapping> manually in a catkin
workspace, be sure to check out the `hydro-devel` branch. There is a commented
section in the cmake file where you can set the variable `parser_workspace`,
which should point to the catkin workspace where your `strands_3d_mapping`
checkout lives. See the lines
<https://github.com/nilsbore/dynamic_object_retrieval/blob/dynamic/CMakeLists.txt#L52>
and
<https://github.com/nilsbore/dynamic_object_retrieval/blob/dynamic/benchmark/CMakeLists.txt#L31>.

In the future, you will instead use the packaged version of `strands_3d_mapping`
from the STRANDS project <http://strands.acin.tuwien.ac.at/>. Follow the instructions on <https://github.com/strands-project-releases/strands-releases/wiki> to add the debian package repository. Then install the `metaroom_xml_parser` by typing `sudo apt-get install ros-indigo-metaroom-xml-parser`.

Tested with Ubuntu 14.04 with ROS Indigo and corresponding OpenCV + PCL, QT4. The repos <https://github.com/nilsbore/k_means_tree>, <https://github.com/USCiLab/cereal>
and <https://github.com/mp3guy/Stopwatch> are included in the repo as subtrees.
