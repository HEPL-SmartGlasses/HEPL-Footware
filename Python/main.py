import pandas as pd

import os
os.environ['LD_LIBRARY_PATH'] = os.getcwd()
import pyximport
pyximport.install(setup_args={"script_args" : ["--verbose"]})
from wrapper import call
from wrapper import init
from plot import plot

import numpy as np
from mat4py import loadmat

if os.path.exists("processed.csv"):
	os.remove("processed.csv")

line_count = 0
timeDelta = 1/104
f = loadmat("./PyShoe Dataset/pyshoe/data/vicon/processed/2018-02-22-10-10-56.mat")
imu = np.array(f['imu']) # 0-2: a, 3-5: w
ts = np.array(f['ts'])
gt = np.array(f['gt'])
prev_t = 0
pos = pd.DataFrame([], columns=['corrX','corrY','corrZ','predX','predY','predZ','optX','optY','optZ','gainX','gainY','gainZ','w_avgX','w_avgY','w_avgZ','quatW','quatX','quatY','quatZ','zuptPhase','w_mag','ts'])

init()

#offsets = [imu[0][0], imu[0][1], imu[0][2], imu[0][3], imu[0][4], imu[0][5]]
offsets = [0,0,0,0,0,0]

for row in imu:
	#row = [2.5993, -0.0364, 9.4176,0,0,0]
	if line_count == 0:
		line_count += 1
		continue
	#elif line_count > (200 * 4): # Analyze specific time range
	elif line_count > 3247:
		break
	else:
		line_count += 1

	ret = call(row[0]-offsets[0], row[1]-offsets[1], row[2]-offsets[2], row[3]-offsets[3], row[4]-offsets[4], row[5]-offsets[5], (ts[line_count-2]-prev_t))
	#ret += gt[0]
	new = pd.DataFrame(data={'corrX':ret[0],'corrY':ret[1],'corrZ':ret[2],'predX':ret[3],'predY':ret[4],'predZ':ret[5],'optX':ret[6],'optY':ret[7],'optZ':ret[8],'gainX':ret[9],'gainY':ret[10],'gainZ':ret[11],'w_avgX':ret[12],'w_avgY':ret[13],'w_avgZ':ret[14],'quatW':ret[15],'quatX':ret[16],'quatY':ret[17],'quatZ':ret[18],'zuptPhase':ret[19],'w_mag':ret[20],'ts': ts[line_count-2]}, index=[0])
	pos = pd.concat([pos, new], ignore_index = True)
	prev_t = ts[line_count-2]

	if np.isnan(ret[0]):
		print('NaN divergence @ ' + str(line_count-2))
		break
# Store in csv
#print("GT:")
#print(gt)
print("Pos:")
print(pos)
pos.to_csv("processed.csv", index=False)

# Call plot function
plot()
