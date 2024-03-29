from mpl_toolkits import mplot3d

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from mat4py import loadmat

def plot():
	fig = plt.figure(figsize=(20, 20))

	hepl = pd.read_csv("processed.csv")
	f = loadmat("./PyShoe Dataset/pyshoe/data/vicon/processed/2018-02-22-10-10-56.mat")
	gt = np.array(f['gt'])
	gt_x = [(x[0]-gt[0][0]) for x in gt]
	gt_y = [(x[1]-gt[0][1]) for x in gt]
	gt_z = [(x[2]-gt[0][2]) for x in gt]

	gt_x = gt_x[0:len(hepl.ts)]
	gt_y = gt_y[0:len(hepl.ts)]
	gt_z = gt_z[0:len(hepl.ts)]

	imu = np.array(f['imu'])
	imu_x = [(x[0]) for x in imu]
	imu_x = imu_x[0:len(hepl.ts)]

# Calculate Error
	rmse = np.sqrt(((np.array([hepl.optX, hepl.optY, hepl.optZ]) - np.array([gt_x, gt_y, gt_z])) ** 2).mean())
	print("RMSE:")
	print(rmse)
	mae = np.abs(np.array([hepl.optX, hepl.optY, hepl.optZ]) - np.array([gt_x, gt_y, gt_z])).mean()
	print("MAE:")
	print(mae)

# 3D Plot
	ax = fig.add_subplot(4, 2, 1, projection='3d')
	ax.plot3D(hepl.optX, hepl.optY, hepl.optZ, 'blue')
	ax.plot3D(gt_x, gt_y, gt_z, 'black')

# X Plot
	scale = 1
	imuscale = 0.1
	axX = fig.add_subplot(4, 2, 2)
	axX.plot(hepl.ts, gt_x * np.array(scale), 'black')
	axX.plot(hepl.ts, hepl.optX * np.array(scale), 'blue')
	axX.plot(hepl.ts, hepl.corrX * np.array(scale), 'red')
	axX.plot(hepl.ts, hepl.predX * np.array(scale), 'green')
	axX.plot(hepl.ts, imu_x * np.array(imuscale), 'yellow')
	axX.set_xlabel('Time (s)')
	axX.set_ylabel('X Position (m)')
	#axX.legend(['Truth', 'HEPL Optimal', 'HEPL Measured', 'HEPL Correction'], fontsize=12)
	axX.legend(['Truth', 'HEPL Optimal','HEPL Correction', 'HEPL Prediction','IMU'], fontsize=12)

# Y Plot
	axY = fig.add_subplot(4, 2, 3)
	axY.plot(hepl.ts, gt_y, 'black')
	axY.plot(hepl.ts, hepl.optY, 'blue')
	axY.plot(hepl.ts, hepl.corrY, 'red')
	axY.plot(hepl.ts, hepl.predY, 'green')
	axY.set_xlabel('Time (s)')
	axY.set_ylabel('Y Position (m)')
	axY.legend(['Truth', 'HEPL Optimal','HEPL Correction', 'HEPL Prediction'], fontsize=12)

# Z Plot
	axZ = fig.add_subplot(4, 2, 4)
	axZ.plot(hepl.ts, gt_z, 'black')
	axZ.plot(hepl.ts, hepl.optZ, 'blue')
	axZ.plot(hepl.ts, hepl.corrZ, 'red')
	axZ.plot(hepl.ts, hepl.predZ, 'green')
	axZ.set_xlabel('Time (s)')
	axZ.set_ylabel('Z Position (m)')
	axZ.legend(['Truth', 'HEPL Optimal','HEPL Correction','HEPL Prediction'], fontsize=12)

# Gain Plot
	axK = fig.add_subplot(4, 2, 5)
	axK.plot(hepl.ts, hepl.gainX, 'blue')
	axK.plot(hepl.ts, hepl.gainY, 'red')
	axK.plot(hepl.ts, hepl.gainZ, 'green')
	axK.set_xlabel('Time (s)')
	axK.set_ylabel('Kalman Gain, X Y Z')
	axK.legend(['X','Y','Z'], fontsize=12)

# w_avg plot
	axW = fig.add_subplot(4, 2, 6)
	axW.plot(hepl.ts, hepl.w_avgX, 'blue')
	axW.plot(hepl.ts, hepl.w_avgY, 'red')
	axW.plot(hepl.ts, hepl.w_avgZ, 'green')
	axW.plot(hepl.ts, hepl.w_mag, 'gray')
	axW.set_xlabel('Time (s)')
	axW.set_ylabel('w_avg, X Y Z mag')
	axW.legend(['X','Y','Z','Mag'], fontsize=12)

# Quaternion plot
	axQ = fig.add_subplot(4, 2, 7)
	axQ.plot(hepl.ts, hepl.quatX, 'blue')
	axQ.plot(hepl.ts, hepl.quatY, 'red')
	axQ.plot(hepl.ts, hepl.quatZ, 'green')
	#axQ.plot(hepl.ts, hepl.quatW, 'gray')
	axQ.set_xlabel('Time (s)')
	axQ.set_ylabel('Quat, X Y Z W')
	axQ.legend(['X','Y','Z'], fontsize=12)

# ZUPT Plot
	axZUPT = fig.add_subplot(4, 2, 8)
	axZUPT.plot(hepl.ts, (hepl.zuptPhase*np.array(-1))+np.array(1), 'red')
	axZUPT.set_xlabel('Time (s)')
	axZUPT.set_ylabel('ZUPT')
	axZUPT.legend(['Phase-Sw:1,St:0'])

# Save Image
	fig.suptitle('HEPL Eval\n' + 'RMSE: ' + str(rmse) + '\nMAE: ' + str(mae))
	plt.tight_layout()
	plt.savefig('Processed.png')
