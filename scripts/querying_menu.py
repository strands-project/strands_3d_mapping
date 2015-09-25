#!/usr/bin/python

from os import path, system

class QueryingPicker(object):

    vocabulary_path = None

    def __init__(self):

        self.set_data_path()

    def set_data_path(self):

        while True:

            data_path_string = raw_input(" Please supply the path to the vocabulary: ")
            self.vocabulary_path = path.abspath(data_path_string)

            if path.isdir(self.vocabulary_path):
                break
            else:
                print " " + self.vocabulary_path + " is not a valid folder!"

    def run(self):

        while True:

            print (" \n"
                   " Working on vocabulary path " + vocabulary_path + "\n"
                   " \n"
                   " 1. Set vocabulary path (if you want to change)\n"
                   " 2. Query vocabulary for point cloud\n"
                   " 3. Exit\n")
            option = raw_input(" Please enter an option 1-6: ")

            if option == "1":
                self.set_data_path()
            elif option == "2":
                print " Running query_vocabulary...\n"
                pcd_path_string = raw_input(" Please supply the point cloud to query: ")
                system("./query_vocabulary " + self.vocabulary_path + " " + pcd_path_string)
            elif option == "3":
                return
            else:
                print " Option " + option + " is not valid."

if __name__ == "__main__":

    picker = QueryingPicker()
    picker.run()
