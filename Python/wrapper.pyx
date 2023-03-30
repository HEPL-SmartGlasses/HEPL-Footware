from cython.operator cimport dereference as deref

cdef extern from "processing.h":
	ctypedef struct SensorData:
		float XL_X
		float XL_Y
		float XL_Z
		float G_X
		float G_Y
		float G_Z
		
	ctypedef struct Position:
		float X
		float Y
		float Z

	ctypedef struct Quaternion:
		float W
		float X
		float Y
		float Z
	
	void init_processing()
	
	void calculateCorrectedState(SensorData* IMU0_data, SensorData* IMU1_data, float timeDelta)
	void calculateAvgAngularRate(SensorData* IMU0_data, SensorData* IMU1_data)
	void calculateRotationMatrix(float timeDelta)
	void updateFMatrix(float timeDelta)
	void updateBMatrix(float timeDelta)
	void updateUVector(SensorData* IMU0_data, SensorData* IMU1_data)
	void calculateStateEstimation()
	void calculateStateEstimationErrorCovariance()
	void updatePreviousMatrices()
	
	float returnDebugOutput(Position* corr, Position* pred, Position* opt, Position* gain, Position* w_avg, Quaternion* quat, Position* zupt) 
	
cpdef init():
	init_processing()

cpdef call(float XL_Xin, float XL_Yin, float XL_Zin, float G_Xin, float G_Yin, float G_Zin, float timeDelta):

	cdef SensorData IMU0_data = SensorData(XL_X=XL_Xin,XL_Y=XL_Yin,XL_Z=XL_Zin,G_X=G_Xin,G_Y=G_Yin,G_Z=G_Zin)

	cdef Position corr = Position(X=0,Y=0,Z=0)
	cdef Position pred = Position(X=0,Y=0,Z=0)
	cdef Position opt = Position(X=0,Y=0,Z=0)
	cdef Position gain = Position(X=0,Y=0,Z=0)
	cdef Position w_avg = Position(X=0,Y=0,Z=0)
	cdef Quaternion quat = Quaternion(W=0,X=0,Y=0,Z=0)
	cdef Position zupt = Position(X=0,Y=0,Z=0)

	cdef float w_mag = 0;

	calculateCorrectedState(&IMU0_data, &IMU0_data, timeDelta)
		
	w_mag = returnDebugOutput(&corr, &pred, &opt, &gain, &w_avg, &quat, &zupt)
	
	return [corr.X,corr.Y,corr.Z,pred.X,pred.Y,pred.Z,opt.X,opt.Y,opt.Z,gain.X,gain.Y,gain.Z,w_avg.X,w_avg.Y,w_avg.Z,quat.W,quat.X,quat.Y,quat.Z,zupt.X,w_mag]	

#cdef printPosition(Position* p):
	#print('x: ' + str(p.X) + ' y: ' + str(p.Y) + ' z: ' + str(p.Z))
