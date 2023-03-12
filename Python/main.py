from ctypes import *
import os

__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

so_file = os.path.join(__location__, 'processing.so')
p_funcs = CDLL(so_file)

class SensorData(Structure):
        __fields__ = [
                    ('XL_X', c_float),
                    ('XL_Y', c_float),
                    ('XL_Z', c_float),
                    ('G_X', c_float),
                    ('G_Y', c_float),
                    ('G_Z', c_float)]

IMU0_data = SensorData(0,0,0,0,0,0)
IMU1_data = SensorData(0,0,0,0,0,0)

p_funcs.calculateCorrectedState(pointer(IMU0_data), pointer(IMU1_data), 1/104)