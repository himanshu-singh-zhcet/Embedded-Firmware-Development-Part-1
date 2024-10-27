################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/17_rtc_lcd.c 

OBJS += \
./Src/17_rtc_lcd.o 

C_DEPS += \
./Src/17_rtc_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F401RE -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"C:/MCU1-Course/MCU1/06_stm32_f4xx_drivers/drivers/Inc" -I"C:/MCU1-Course/MCU1/06_stm32_f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/17_rtc_lcd.d ./Src/17_rtc_lcd.o ./Src/17_rtc_lcd.su

.PHONY: clean-Src

