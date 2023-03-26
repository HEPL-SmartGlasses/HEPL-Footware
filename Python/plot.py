from mpl_toolkits import mplot3d

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from mat4py import loadmat
from scipy import integrate

def plot():
	fig = plt.figure()
	ax = plt.axes(projection='3d')

	hepl = pd.read_csv("processed.csv")
	f = loadmat("./PyShoe Dataset/pyshoe/data/vicon/raw/2018-02-22-10-10-56/processed_data.mat")
	gt = np.array(f['gt'])
	gt_x = [(x[0]) for x in gt]
	gt_y = [(x[1]) for x in gt]
	gt_z = [(x[2]) for x in gt]

	ts = np.array(f['ts'])

	#imu = np.array(f['imu'])
	#imu_x = np.array([x[3] for x in imu])
	#imu_y = np.array([x[4] for x in imu])
	#imu_z = np.array([x[5] for x in imu])

	#scale = np.array(1.0)
	#imu_x = integrate.cumtrapz(integrate.cumtrapz(imu_x, ts, axis=0), np.resize(ts,3246))/scale
	#imu_y = integrate.cumtrapz(integrate.cumtrapz(imu_y, ts, axis=0), np.resize(ts,3246))/scale
	#imu_z = integrate.cumtrapz(integrate.cumtrapz(imu_z, ts, axis=0), np.resize(ts,3246))/scale
	one = np.array(1)
	hundred = np.array(100)
	diff = np.abs((np.array(hepl)-gt)*hundred/gt)

	#for i in diff:
	#	print("X: " + str(i[0]) + "% Y: " + str(i[1]) + "% Z: " + str(i[2]) + "%")

	ax.plot3D(hepl.X, hepl.Y, hepl.Z, 'blue')
	ax.plot3D(gt_x, gt_y, gt_z, 'red')
	#ax.plot3D(imu_x, imu_y, imu_z, 'gray')
	plt.xlabel('X')
	plt.ylabel('Y')

	ax.legend(['HEPL', 'Truth'])
#	ax.legend(['HEPL', 'Truth', 'IMU'])

	plt.savefig('Processed.png')

	fig2 = plt.figure()
	ax2 = plt.axes()
	ax2.plot(ts, gt_z, 'red')
	#ax2.plot(ts, hepl.Z, 'blue')
	plt.xlabel('Time (s)')
	plt.ylabel('Z Position (m)')
	#ax2.legend(['HEPL', 'Truth'])

	plt.savefig('Processed_Z.png')
