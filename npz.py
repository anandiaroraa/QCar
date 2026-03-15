from turtle import speed

import numpy as np
import matplotlib.pyplot as plt
import math
import glob, os
files = glob.glob('hardware_results_test1/velocity0.15-0.20.npz')
latest = max(files, key=os.path.getmtime)
d = np.load(latest, allow_pickle=True)
print(list(d.keys()))
for key in d.keys():
    print(f"{key}: {d[key]}")