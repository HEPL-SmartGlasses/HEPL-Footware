CC=gcc
CFLAGS=-fPIC -Wall -shared -o
OTHERDIR=../Firmware/Core/Src/
DSPDIR=../Firmware/Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/

SOURCE=$(OTHERDIR):$(OTHERDIR)../../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/

INCLUDE= -I$(DSPDIR) -I$(OTHERDIR)../../../Python -I$(DSPDIR)../../Core/Include/ 

VPATH=$(OTHERDIR):%.h

all: clean copy processing.so

copy:
	cp $(OTHERDIR)../Inc/processing.h .
	sed -i 's/#include "IMU.h"/typedef struct {float XL_X;float XL_Y;float XL_Z;float G_X;float G_Y;float G_Z;} SensorData;/g' processing.h

processing.so: processing.c processing.h Makefile
	$(CC) $(CFLAGS) $@ $< $(INCLUDE)

clean:
	@-rm -f *.so *.h

