################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/lcd/font12.c \
../Core/Src/lcd/font16.c \
../Core/Src/lcd/font20.c \
../Core/Src/lcd/font24.c \
../Core/Src/lcd/font8.c \
../Core/Src/lcd/ili9341.c \
../Core/Src/lcd/peashooter_000.c \
../Core/Src/lcd/peashooter_001.c \
../Core/Src/lcd/peashooter_002.c \
../Core/Src/lcd/peashooter_003.c \
../Core/Src/lcd/peashooter_004.c \
../Core/Src/lcd/peashooter_005.c 

OBJS += \
./Core/Src/lcd/font12.o \
./Core/Src/lcd/font16.o \
./Core/Src/lcd/font20.o \
./Core/Src/lcd/font24.o \
./Core/Src/lcd/font8.o \
./Core/Src/lcd/ili9341.o \
./Core/Src/lcd/peashooter_000.o \
./Core/Src/lcd/peashooter_001.o \
./Core/Src/lcd/peashooter_002.o \
./Core/Src/lcd/peashooter_003.o \
./Core/Src/lcd/peashooter_004.o \
./Core/Src/lcd/peashooter_005.o 

C_DEPS += \
./Core/Src/lcd/font12.d \
./Core/Src/lcd/font16.d \
./Core/Src/lcd/font20.d \
./Core/Src/lcd/font24.d \
./Core/Src/lcd/font8.d \
./Core/Src/lcd/ili9341.d \
./Core/Src/lcd/peashooter_000.d \
./Core/Src/lcd/peashooter_001.d \
./Core/Src/lcd/peashooter_002.d \
./Core/Src/lcd/peashooter_003.d \
./Core/Src/lcd/peashooter_004.d \
./Core/Src/lcd/peashooter_005.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/lcd/%.o Core/Src/lcd/%.su: ../Core/Src/lcd/%.c Core/Src/lcd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-lcd

clean-Core-2f-Src-2f-lcd:
	-$(RM) ./Core/Src/lcd/font12.d ./Core/Src/lcd/font12.o ./Core/Src/lcd/font12.su ./Core/Src/lcd/font16.d ./Core/Src/lcd/font16.o ./Core/Src/lcd/font16.su ./Core/Src/lcd/font20.d ./Core/Src/lcd/font20.o ./Core/Src/lcd/font20.su ./Core/Src/lcd/font24.d ./Core/Src/lcd/font24.o ./Core/Src/lcd/font24.su ./Core/Src/lcd/font8.d ./Core/Src/lcd/font8.o ./Core/Src/lcd/font8.su ./Core/Src/lcd/ili9341.d ./Core/Src/lcd/ili9341.o ./Core/Src/lcd/ili9341.su ./Core/Src/lcd/peashooter_000.d ./Core/Src/lcd/peashooter_000.o ./Core/Src/lcd/peashooter_000.su ./Core/Src/lcd/peashooter_001.d ./Core/Src/lcd/peashooter_001.o ./Core/Src/lcd/peashooter_001.su ./Core/Src/lcd/peashooter_002.d ./Core/Src/lcd/peashooter_002.o ./Core/Src/lcd/peashooter_002.su ./Core/Src/lcd/peashooter_003.d ./Core/Src/lcd/peashooter_003.o ./Core/Src/lcd/peashooter_003.su ./Core/Src/lcd/peashooter_004.d ./Core/Src/lcd/peashooter_004.o ./Core/Src/lcd/peashooter_004.su ./Core/Src/lcd/peashooter_005.d ./Core/Src/lcd/peashooter_005.o ./Core/Src/lcd/peashooter_005.su

.PHONY: clean-Core-2f-Src-2f-lcd

