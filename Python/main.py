import pandas as pd

import os
os.environ['LD_LIBRARY_PATH'] = os.getcwd()
import pyximport
pyximport.install(setup_args={"script_args" : ["--verbose"]})
from wrapper import call
from plot import plot

import numpy as np
from mat4py import loadmat

if os.path.exists("processed.csv"):
    os.remove("processed.csv")

with open("./PyShoe Dataset/pyshoe/data/vicon/raw/2018-02-22-10-10-56/imudata.csv") as csvfile:
    df = pd.read_csv(csvfile, delimiter=',', usecols= ['x.2','y.2','z.2','x.1','y.1','z.1'])
    pos = pd.DataFrame([], columns=['X', 'Y', 'Z'])
    line_count = 0
    timeDelta = 1/104
    f = loadmat("./PyShoe Dataset/pyshoe/data/vicon/raw/2018-02-22-10-10-56/processed_data.mat")
    ts = np.array(f['ts'])
    prev_t = 0

    for row in df.iterrows():
        if line_count == 0:
            line_count += 1
            continue
        #elif line_count > 100:
        elif line_count > 3247:
            break
        else:
            line_count += 1
        #print()

        ret = call(row[1]['x.2'], row[1]['y.2'], row[1]['z.2'], row[1]['x.1'], row[1]['y.2'], row[1]['z.1'], (ts[line_count-2]-prev_t))
        new = pd.DataFrame(data={'X': ret[0], 'Y': ret[1], 'Z': ret[2]}, index=[0])
        pos = pd.concat([pos, new], ignore_index = True)
        print(ts[line_count-2])
        prev_t = ts[line_count-2]
        print(new)
        if np.isnan(ret[0]):
            print('NaN divergence @ ' + str(line_count-2))
            break

    pos.to_csv("processed.csv", index=False)

    plot()

