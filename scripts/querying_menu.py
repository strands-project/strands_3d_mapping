#!/usr/bin/python

from os import path, system

class QueryingPicker(object):

    annotated_data_path = None
    noise_data_path = None

    def __init__(self):

        self.set_data_path()

    def set_data_path(self):

        while True:

            data_path_string = raw_input(" Please supply the path to the annotated data: ")
            self.annotated_data_path = path.abspath(data_path_string)

            if path.isdir(self.annotated_data_path):
                break
            else:
                print " " + self.annotated_data_path + " is not a valid folder!"

        while True:

            data_path_string = raw_input(" Please supply the path to the noise data: ")
            self.noise_data_path = path.abspath(data_path_string)

            if path.isdir(self.noise_data_path):
                break
            else:
                print " " + self.noise_data_path + " is not a valid folder!"

    def run(self):

        print (" \n"
               " Working on annotated data path " + self.annotated_data_path + "\n"
               " and noise data path " + self.noise_data_path + "\n"
               " \n"
               " 1. Set data paths (if you want to change)\n"
               " 2. Benchmark convex segment vocabulary tree querying\n"
               " 3. Benchmark convex segment vocabulary tree with re-weighting\n"
               " 4. Benchmark subsegment vocabulary tree querying\n"
               " 5. Benchmark subsegment vocabulary tree with re-weighting\n"
               " 6. Exit\n")
        option = raw_input(" Please enter an option 1-6: ")

        if option == "1":
            self.set_data_path()
        elif option == "2":
            print " Running create_scan_folders...\n"
            system("./create_scan_folders " + self.data_path)
        elif option == "3":
            print " Running create_convex_folders...\n"
            system("./create_convex_folders " + self.data_path)
        elif option == "4":
            return
        elif option == "6":
            return
        else:
            print " Option " + option + " is not valid."

if __name__ == "__main__":

    picker = QueryingPicker()
    picker.run()
