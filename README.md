object_3d_retrieval
===================

This contains our implementation of a vocabulary tree (Nister & Stewenius) together with some modifications for our paper. It also includes some functions e.g. for extracting segments & features and querying on our data structure.

To use the system, follow the instructions in `scripts/menu.py`. There are submenus, for example for querying that can be run separately, i.e. `scripts/training_menu.py` and `scripts/querying_menu.py`.

## Dependencies

Tested with Ubuntu 14.04 with ROS Indigo and corresponding OpenCV + PCL, QT4. The repos <https://github.com/nilsbore/k_means_tree> and <https://github.com/USCiLab/cereal> are included in the repo as subtrees.

You further need one package from the STRANDS project <http://strands.acin.tuwien.ac.at/>. Follow the instructions on <https://github.com/strands-project-releases/strands-releases/wiki> to add the debian package repository. Then install the `metaroom_xml_parser` by typing `sudo apt-get install ros-indigo-metaroom-xml-parser`.

If you do not want to add the debian repository, you can compile <https://github.com/strands-project/strands_3d_mapping> manually in a catkin workspace. There is a commented section in the cmake file where you can add the necessary include and linking information. 
