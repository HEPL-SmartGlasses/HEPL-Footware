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
	
	float returnCurrentPosition(Position* p) 
	

cpdef call(float XL_X0):
	init_processing()

	timeDelta = 1.0/104

	cdef SensorData IMU0_data = SensorData(XL_X=XL_X0,XL_Y=0,XL_Z=0,G_X=0,G_Y=0,G_Z=0)
	cdef SensorData IMU1_data = SensorData(XL_X=0,XL_Y=0,XL_Z=0,G_X=0,G_Y=0,G_Z=0)

	cdef Position p = Position(X=0,Y=0,Z=0)

	for i in range (0,104):
		calculateCorrectedState(&IMU0_data, &IMU1_data, timeDelta)
		
	printPosition(&p)	

cdef printPosition(Position* p):
	returnCurrentPosition(&p)
	print(p.X)
	print(p.Y)
	print(p.Z)