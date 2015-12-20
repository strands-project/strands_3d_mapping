#!/usr/bin/python

from os import path, system

class TrainingPicker(object):

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
                   " Please make sure to do the following in order\n"
                   " \n"
                   " Working on vocabulary path " + self.vocabulary_path + "\n"
                   " \n"
                   " 1. Set vocabulary path (if you want to change)\n"
                   " 2. Initialize vocabulary folders and files\n"
                   " 3. Build vocabulary tree representation\n"
                   " 4. Exit\n")
            option = raw_input(" Please enter an option 1-4: ")

            if option == "1":
                self.set_data_path()
            elif option == "2":
                print " Running dynamic_init_vocabulary...\n"
                noise_path_string = raw_input(" Please supply the noise data path: ")
                annotated_path_string = raw_input(" Please supply the annotated data path: ")
                vocabulary_type = raw_input(" Enter vocabulary type (standard/incremental): ")
                subsegment_type = ""
                if vocabulary_type == "incremental":
                    subsegment_type = raw_input(" Enter subsegment type (subsegment/convex_segment/supervoxel): ")
                system("./dynamic_init_vocabulary " + self.vocabulary_path + " " + \
                       noise_path_string + " " + annotated_path_string + " " + vocabulary_type + " " + subsegment_type)
            elif option == "3":
                print " Running dynamic_train_vocabulary...\n"
                system("./dynamic_train_vocabulary " + self.vocabulary_path)
            elif option == "4":
                return
            else:
                print " Option " + option + " is not valid."

if __name__ == "__main__":

    picker = TrainingPicker()
    picker.run()
