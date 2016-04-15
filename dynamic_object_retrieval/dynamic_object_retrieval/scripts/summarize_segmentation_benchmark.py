#!/usr/bin/python

import os
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import sys

def get_values(folder):
    rootdir = os.path.abspath(folder)

    print rootdir

    topdirs = [ os.path.join(rootdir, name) for name in os.listdir(rootdir) if os.path.isdir(os.path.join(rootdir, name)) ]
    for objectdir in topdirs:
        #print subdir
        overlaps = []
        for subdir, dirs, files in os.walk(objectdir):
            for somefile in files:
                if somefile != "overlaps.txt":
                    continue
                with open(os.path.join(subdir, somefile)) as f:
                    for line in f:
                        overlaps.append(float(line))
                #print overlaps
        meanoverlap = np.mean(np.array(overlaps))
        print "Mean for %s: %f" % (objectdir, meanoverlap)



if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("Please provide directory to check!")
    get_values(sys.argv[1])
