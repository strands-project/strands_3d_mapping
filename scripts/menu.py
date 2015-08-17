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
                   " 2. Create scan folders\n"
                   " 3. Create convex segment folders\n"
                   " 4. Extract features from convex segments\n"
                   " 5. Segment convex features into subsegments\n"
                   " 6. Extract SIFT features from scans (needed for re-weighting)\n"
                   " 7. Vocabulary training menu\n"
                   " 8. Querying & benchmarking menu\n"
                   " 9. Exit\n")
            option = raw_input(" Please enter an option 1-9: ")

            if option == "1":
                self.set_data_path()
            elif option == "2":
                print " Running create_scan_folders...\n"
                system("./create_scan_folders " + self.data_path + "/")
            elif option == "3":
                print " Running create_convex_folders...\n"
                system("./create_convex_folders " + self.data_path + "/")
            elif option == "4":
                print " Running create_convex_features...\n"
                system("./create_convex_features " + self.data_path + "/")
            elif option == "5":
                print " Running create_subsegment_features...\n"
                system("./create_subsegment_features " + self.data_path + "/")
            elif option == "6":
                print " Running create_scan_sift_features...\n"
                system("./create_scan_sift_features " + self.data_path + "/")
            elif option == "7":
                print " Entering the vocabulary training menu..."
                picker = TrainingPicker()
                picker.run()
            elif option == "8":
                print " Entering the querying & benchmarking menu..."
                picker = QueryingPicker()
                picker.run()
            elif option == "9":
                return
            else:
                print " Option " + option + " is not valid."


if __name__ == "__main__":

    picker = ExecutablePicker()
    picker.run()
