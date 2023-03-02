################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctionsF16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f64.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f64.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q7.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f64.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f64.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q7.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctionsF16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f64.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f64.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q7.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f64.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f64.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q7.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctionsF16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f64.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f64.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q7.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f64.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f64.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q7.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/%.su: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DSTM32L443xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-CMSIS-2f-DSP-2f-Source-2f-MatrixFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-CMSIS-2f-DSP-2f-Source-2f-MatrixFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctionsF16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f64.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_trans_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f16.o
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f64.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f64.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f64.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_ldlt_f64.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f64.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f64.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f64.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q7.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q7.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q7.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_lower_triangular_f64.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f32.su
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_solve_upper_triangular_f64.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f64.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f64.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f64.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f64.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f64.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f64.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q7.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q7.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q7.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f16.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q15.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q31.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_q7.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-CMSIS-2f-DSP-2f-Source-2f-MatrixFunctions

