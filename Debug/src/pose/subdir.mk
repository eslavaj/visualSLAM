################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/pose/CameraPoseEstimator.cpp \
../src/pose/KeypointProcessorGpu.cpp \
../src/pose/triangulate.cpp 

OBJS += \
./src/pose/CameraPoseEstimator.o \
./src/pose/KeypointProcessorGpu.o \
./src/pose/triangulate.o 

CPP_DEPS += \
./src/pose/CameraPoseEstimator.d \
./src/pose/KeypointProcessorGpu.d \
./src/pose/triangulate.d 


# Each subdirectory must supply rules for building sources it contributes
src/pose/%.o: ../src/pose/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/usr/local/include/opencv4 -I/usr/local/include -I/usr/include/eigen3 -I/usr/include -I/usr/include/opencv4 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


