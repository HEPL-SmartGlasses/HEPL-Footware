from mpl_toolkits import mplot3d

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from mat4py import loadmat

def plot():
	fig = plt.figure()

	hepl = pd.read_csv("processed.csv")
	f = loadmat("./PyShoe Dataset/pyshoe/data/vicon/raw/2018-02-22-10-10-56/processed_data.mat")
	gt = np.array(f['gt'])
	gt_x = [(x[0]-gt[0][0]) for x in gt]
	gt_y = [(x[1]-gt[0][1]) for x in gt]
	gt_z = [(x[2]-gt[0][2]) for x in gt]

	gt_x = gt_x[0:len(hepl.ts)]
	gt_y = gt_y[0:len(hepl.ts)]
	gt_z = gt_z[0:len(hepl.ts)]

	imu = np.array(f['imu'])
	imu_x = [(x[3]-imu[0][3]) for x in imu]
	imu_x = imu_x[0:len(hepl.ts)]

# Calculate Error
	rmse = np.sqrt(((np.array([hepl.optX, hepl.optY, hepl.optZ]) - np.array([gt_x, gt_y, gt_z])) ** 2).mean())
	print("RMSE:")
	print(rmse)
	mae = np.abs(np.array([hepl.optX, hepl.optY, hepl.optZ]) - np.array([gt_x, gt_y, gt_z])).mean()
	print("MAE:")
	print(mae)

# 3D Plot
	ax = fig.add_subplot(2, 2, 1, projection='3d')
	ax.plot3D(hepl.optX, hepl.optY, hepl.optZ, 'blue')
	ax.plot3D(gt_x, gt_y, gt_z, 'black')

# X Plot
	scale = 1
	axX = fig.add_subplot(2, 2, 2)
	axX.plot(hepl.ts, gt_x * np.array(scale), 'black')
	axX.plot(hepl.ts, hepl.optX * np.array(scale), 'blue')
	axX.plot(hepl.ts, hepl.measX * np.array(scale), 'red')
	axX.plot(hepl.ts, hepl.predX * np.array(scale), 'green')
	#axX.plot(hepl.ts, imu_x, 'gray')
	axX.set_xlabel('Time (s)')
	axX.set_ylabel('X Position (m)')
	axX.legend(['Truth', 'HEPL Optimal', 'HEPL Measured', 'HEPL Prediction'], fontsize=6)

# Y Plot
	axY = fig.add_subplot(2, 2, 3)
	axY.plot(hepl.ts, gt_y, 'black')
	axY.plot(hepl.ts, hepl.optY, 'blue')
	axY.plot(hepl.ts, hepl.measY, 'red')
	axY.plot(hepl.ts, hepl.predX, 'green')
	axY.set_xlabel('Time (s)')
	axY.set_ylabel('Y Position (m)')
	axY.legend(['Truth', 'HEPL Optimal', 'HEPL Measured', 'HEPL Prediction'], fontsize=6)

# Z Plot
	axZ = fig.add_subplot(2, 2, 4)
	axZ.plot(hepl.ts, gt_z, 'black')
	axZ.plot(hepl.ts, hepl.optZ, 'blue')
	axZ.plot(hepl.ts, hepl.measZ, 'red')
	axZ.plot(hepl.ts, hepl.predZ, 'green')
	axZ.set_xlabel('Time (s)')
	axZ.set_ylabel('Z Position (m)')
	axZ.legend(['Truth', 'HEPL Optimal', 'HEPL Measured', 'HEPL Prediction'], fontsize=6)

# Save Image
	fig.suptitle('HEPL Eval\n' + 'RMSE: ' + str(rmse) + '\nMAE: ' + str(mae))
	plt.tight_layout()
	plt.savefig('Processed.png')
