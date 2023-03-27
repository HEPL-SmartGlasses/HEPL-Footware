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
	
	float returnDebugOutput(Position* meas, Position* pred, Position* opt) 
	

cpdef call(float XL_Xin, float XL_Yin, float XL_Zin, float G_Xin, float G_Yin, float G_Zin, float timeDelta):
	init_processing()

	cdef SensorData IMU0_data = SensorData(XL_X=XL_Xin,XL_Y=XL_Yin,XL_Z=XL_Zin,G_X=G_Xin,G_Y=G_Yin,G_Z=G_Zin)

	cdef Position meas = Position(X=0,Y=0,Z=0)
	cdef Position pred = Position(X=0,Y=0,Z=0)
	cdef Position opt = Position(X=0,Y=0,Z=0)

	calculateCorrectedState(&IMU0_data, &IMU0_data, timeDelta)
		
	
	returnDebugOutput(&meas, &pred, &opt)
	
	return [meas.X,meas.Y,meas.Z,pred.X,pred.Y,pred.Z,opt.X,opt.Y,opt.Z]	

#cdef printPosition(Position* p):
	#print('x: ' + str(p.X) + ' y: ' + str(p.Y) + ' z: ' + str(p.Z))
