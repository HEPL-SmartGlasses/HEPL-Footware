# HEPL-Footware

Firmware to drive the foot processor of HEPL. Uses a dual-IMU sensor board to determine displacement. Implements a Kalman filter and ZUPT algorithms to reduce accumulated error due to sensor drift.

## Using the Python Plotter

cd to the Python dir, and start the virtual environment:
	```source ./env/bin/activate```

Run ```python3 -m pip install -r requirements.txt``` to install requirements

Run ```make copy``` to copy required files from STM32 project

Run ```make debug``` to compile processing.c from STM32 and run the plot util

Run ```make run``` if you want to skip compile step

The plot utility prints useful info to the console and makes a plot image called "Processing.png" using "processing.csv"
