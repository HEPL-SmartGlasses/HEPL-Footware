CC=gcc
CFLAGS=-Wall
OTHERDIR=../Firmware/Core/Src/
DSPDIR=../Firmware/Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/
DSPSRC=$(DSPDIR)../Source/
DSPFMF=$(DSPSRC)FastMathFunctions/
DSPMF=$(DSPSRC)MatrixFunctions/
DSPQMF=$(DSPSRC)QuaternionMathFunctions/

SOURCE=$(OTHERDIR):$(DSPDIR):$(DSPSRC):$(DSPFMF):$(DSPMF):$(DSPQMF)

INCLUDE= -I$(DSPDIR) -I$(OTHERDIR)../../../Python -I$(DSPDIR)../../Core/Include/

OBJS= arm_mat_add_f32.o arm_mat_init_f32.o arm_mat_inverse_f32.o arm_mat_mult_f32.o arm_mat_sub_f32.o arm_mat_trans_f32.o arm_quaternion2rotation_f32.o arm_quaternion_normalize_f32.o arm_quaternion_product_single_f32.o

VPATH=$(OTHERDIR):%.h:%.c

all: clean copy processing.so

copy:
	cp $(OTHERDIR)../Inc/processing.h .
	cp -a $(DSPQMF)*_f32.c .
	cp -a $(DSPMF)*_f32.c .
	cp -r $(DSPDIR)dsp/ .
	sed -i 's/#include "IMU.h"/typedef struct {float XL_X;float XL_Y;float XL_Z;float G_X;float G_Y;float G_Z;} SensorData;/g' processing.h

%.o: %.c
	$(CC) -c -o $@ $^ $(INCLUDE)

processing.so: processing.c $(OBJS) processing.h Makefile
	$(CC) -fPIC -shared -o $@ $< $(INCLUDE) $(OBJS)

clean:
	@-rm -f CMakeLists.txt *.o *.so *.h *.c
	@rm -r dsp
