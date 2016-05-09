#!/usr/bin/python

import itertools
import os.path
import json
import numpy as np
import matplotlib.pyplot as plt

def plot_values():

    iss_res = 7 # run2 # 7 # run1
    pfh_res = 1 # run2 # 6 # run1

    mat = np.zeros((iss_res, pfh_res))
    iss = np.zeros((iss_res, pfh_res))
    pfh = np.zeros((iss_res, pfh_res))
    for i in itertools.count():
        fpath = os.path.abspath(str(i) + '.json')
        if not os.path.exists(fpath):
            break

        with open(fpath) as f:
            rates = json.load(f)
            y, x = divmod(i, pfh_res)
            mat[y, x] = rates[2]['total'][2]
            iss[y, x] = rates[0]
            pfh[y, x] = rates[1]
    print mat
    print iss
    print pfh
    plt.matshow(mat)
    plt.show()


if __name__ == '__main__':
    plot_values()
