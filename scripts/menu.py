#!/usr/bin/python

from os import path, system
from training_menu import TrainingPicker
from querying_menu import QueryingPicker

class ExecutablePicker(object):

    data_path = None

    def __init__(self):

        self.set_data_path()

    def set_data_path(self):

        while True:

            data_path_string = raw_input(" Please supply the data path: ")
            self.data_path = path.abspath(data_path_string)

            if path.isdir(self.data_path):
                break
            else:
                print " " + self.data_path + " is not a valid folder!"

    def run(self):

        while True:

            print (" \n"
                   " Please make sure to do the following in order both"
                   " for the noise data folder and for the annotated data\n"
                   " \n"
                   " Working on data path " + self.data_path + "\n"
                   " \n"
                   " 1. Set data path (if you want to change)\n"
                   " 2. Init sweep segment folders\n"
                   " 3. Create convex segments\n"
                   " 4. Create convex segments and supervoxels (alternative to 3 & 6)\n"
                   " 5. Extract PFHRGB features\n"
                   " 6. Segment keypoints into subsegments\n"
                   " 7. Create PFHRGB feature for supervoxels (alternative to 6)\n"
                   " 8. Extract SIFT from sweeps (for re-weighting)\n"
                   " 9. Vocabulary training menu\n"
                   " 10. Querying & benchmarking menu\n"
                   " 11. Exit\n")
            option = raw_input(" Please enter an option 1-9: ")

            if option == "1":
                self.set_data_path()
            elif option == "2":
                print " Running dynamic_init_folders...\n"
                system("./dynamic_init_folders " + self.data_path + "/")
            elif option == "3":
                print " Running dynamic_convex_segmentation...\n"
                system("./dynamic_convex_segmentation " + self.data_path + "/")
            elif option == "4":
                print " Running dynamic_supervoxel_convex_segmentation...\n"
                system("./dynamic_supervoxel_convex_segmentation " + self.data_path + "/")
            elif option == "5":
                print " Running dynamic_extract_convex_features...\n"
                system("./dynamic_extract_convex_features " + self.data_path + "/")
            elif option == "6":
                print " Running dynamic_create_subsegments...\n"
                system("./dynamic_create_subsegments " + self.data_path + "/")
            elif option == "7":
                print " Running dynamic_extract_supervoxel_features...\n"
                system("./dynamic_extract_supervoxel_features " + self.data_path + "/")
            elif option == "8":
                print " Running dynamic_extract_sift...\n"
                system("./dynamic_extract_sift " + self.data_path + "/")
            elif option == "9":
                print " Entering the vocabulary training menu..."
                picker = TrainingPicker()
                picker.run()
            elif option == "10":
                print " Entering the querying & benchmarking menu..."
                picker = QueryingPicker()
                picker.run()
            elif option == "11":
                return
            else:
                print " Option " + option + " is not valid."


if __name__ == "__main__":

    picker = ExecutablePicker()
    picker.run()
