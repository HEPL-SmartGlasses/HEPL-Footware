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

line_count = 0
timeDelta = 1/104
f = loadmat("./PyShoe Dataset/pyshoe/data/vicon/raw/2018-02-22-10-10-56/processed_data.mat")
imu = np.array(f['imu']) # 0-2: w, 3-5: a
ts = np.array(f['ts'])
gt = np.array(f['gt'])
prev_t = 0
pos = pd.DataFrame([], columns=['measX','measY','measZ','corrX','corrY','corrZ','optX','optY','optZ','gainX','gainY','gainZ','w_avgX','w_avgY','w_avgZ','quatW','quatX','quatY','quatZ','w_mag','ts'])

for row in imu:
	if line_count == 0:
		line_count += 1
		continue
	#elif line_count > (200 * 10): # Analyze specific time range
	elif line_count > 3247:
		break
	else:
		line_count += 1

	ret = call(row[3], row[4], row[5], row[0], row[1], row[3], (ts[line_count-2]-prev_t))
	#ret += gt[0]
	new = pd.DataFrame(data={'measX':ret[0],'measY':ret[1],'measZ':ret[2],'corrX':ret[3],'corrY':ret[4],'corrZ':ret[5],'optX':ret[6],'optY':ret[7],'optZ':ret[8],'gainX':ret[9],'gainY':ret[10],'gainZ':ret[11],'w_avgX':ret[12],'w_avgY':ret[13],'w_avgZ':ret[14],'quatW':ret[15],'quatX':ret[16],'quatY':ret[17],'quatZ':ret[18],'w_mag':ret[19],'ts': ts[line_count-2]}, index=[0])
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
