################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/debian/circuitbotDriver/test_app/servodrv.c \
/home/debian/circuitbotDriver/test_app/servodrvtestapp.c 

OBJS += \
./test_app/servodrv.o \
./test_app/servodrvtestapp.o 

C_DEPS += \
./test_app/servodrv.d \
./test_app/servodrvtestapp.d 


# Each subdirectory must supply rules for building sources it contributes
test_app/servodrv.o: /home/debian/circuitbotDriver/test_app/servodrv.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-linux-gnueabihf-gcc -std=c11 -I/usr/include/arm-linux-gnueabihf -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

test_app/servodrvtestapp.o: /home/debian/circuitbotDriver/test_app/servodrvtestapp.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-linux-gnueabihf-gcc -std=c11 -I/usr/include/arm-linux-gnueabihf -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


