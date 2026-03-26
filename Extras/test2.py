# verify check_npz.py reads radius correctly
import numpy as np
import glob, os

files = sorted(glob.glob('hardware_results_test1/*.npz'), key=os.path.getmtime)
for f in files[-3:]:
    d = np.load(f, allow_pickle=True)
    radius = float(d['radius']) if 'radius' in d else 'NOT SAVED'
    target_speed = float(d['target_speed']) if 'target_speed' in d else 'NOT SAVED'
    print(f"{os.path.basename(f)}: radius={radius}, target_speed={target_speed}")