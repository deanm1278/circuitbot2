################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Chameleon.cpp \
../ConfigFile.cpp \
../gcParser.cpp \
../i2cfunc.cpp \
../main.cpp \
../motion_planner.cpp 

C_SRCS += \
/home/debian/circuitbotDriver/test_app/servodrv.c 

OBJS += \
./Chameleon.o \
./ConfigFile.o \
./gcParser.o \
./i2cfunc.o \
./main.o \
./motion_planner.o \
./servodrv.o 

C_DEPS += \
./servodrv.d 

CPP_DEPS += \
./Chameleon.d \
./ConfigFile.d \
./gcParser.d \
./i2cfunc.d \
./main.d \
./motion_planner.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++0x -I/usr/include/arm-linux-gnueabihf/c++/4.9 -I/home/debian/circuitbotDriver/test_app -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

servodrv.o: /home/debian/circuitbotDriver/test_app/servodrv.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-linux-gnueabihf-gcc -std=c11 -I/usr/include/arm-linux-gnueabihf -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


