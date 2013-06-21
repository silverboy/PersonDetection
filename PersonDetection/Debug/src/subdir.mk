################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Detector.cpp \
../src/MyReactiveInterface.cpp \
../src/controlador.cpp \
../src/navegacion.cpp \
../src/svm.cpp 

OBJS += \
./src/Detector.o \
./src/MyReactiveInterface.o \
./src/controlador.o \
./src/navegacion.o \
./src/svm.o 

CPP_DEPS += \
./src/Detector.d \
./src/MyReactiveInterface.d \
./src/controlador.d \
./src/navegacion.d \
./src/svm.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ $(shell pkg-config --cflags mrpt-reactivenav mrpt-gui) -I/usr/include/mrpt/base/include/ -I/usr/include/mrpt/mrpt-config/ -I/usr/include/mrpt/gui/include/ -I/usr/include/mrpt/opengl/include/ -I/usr/include/mrpt/reactivenav/include/ -I/usr/include/mrpt/graphs/include/ -I/usr/include/mrpt/maps/include/ -I/usr/include/mrpt/obs/include/ -I/usr/include -I/usr/include/mrpt/gui/ -I/usr/local/Aria/include -I../include -I/usr/local/Arnl/include -I/usr/local/Arnl/include/ArNetworking -I/usr/include/c++/4.7.2/ -O0 -g3 -Wall -c -fmessage-length=0 -fpermissive -Wno-enum-compare -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


