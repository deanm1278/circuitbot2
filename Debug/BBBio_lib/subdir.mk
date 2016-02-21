################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BBBio_lib/BBBiolib.c \
../BBBio_lib/BBBiolib_ADCTSC.c \
../BBBio_lib/BBBiolib_McSPI.c \
../BBBio_lib/BBBiolib_PWMSS.c 

OBJS += \
./BBBio_lib/BBBiolib.o \
./BBBio_lib/BBBiolib_ADCTSC.o \
./BBBio_lib/BBBiolib_McSPI.o \
./BBBio_lib/BBBiolib_PWMSS.o 

C_DEPS += \
./BBBio_lib/BBBiolib.d \
./BBBio_lib/BBBiolib_ADCTSC.d \
./BBBio_lib/BBBiolib_McSPI.d \
./BBBio_lib/BBBiolib_PWMSS.d 


# Each subdirectory must supply rules for building sources it contributes
BBBio_lib/%.o: ../BBBio_lib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-linux-gnueabihf-gcc -std=c11 -I/usr/include/arm-linux-gnueabihf -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


