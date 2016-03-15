################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../bus/BusDevice.cpp \
../bus/SPIDevice.cpp 

OBJS += \
./bus/BusDevice.o \
./bus/SPIDevice.o 

CPP_DEPS += \
./bus/BusDevice.d \
./bus/SPIDevice.d 


# Each subdirectory must supply rules for building sources it contributes
bus/%.o: ../bus/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++0x -I/usr/include/arm-linux-gnueabihf/c++/4.9 -I/home/debian/circuitbotDriver/test_app -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


