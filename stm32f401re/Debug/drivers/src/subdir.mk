################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f_gpio_driver.c 

OBJS += \
./drivers/src/stm32f_gpio_driver.o 

C_DEPS += \
./drivers/src/stm32f_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DNUCLEO_F401RE -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I"D:/shiva/em/workspace/stm32f401re/drivers/inc" -I"D:/shiva/em/workspace/stm32f401re/drivers/src" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f_gpio_driver.cyclo ./drivers/src/stm32f_gpio_driver.d ./drivers/src/stm32f_gpio_driver.o ./drivers/src/stm32f_gpio_driver.su

.PHONY: clean-drivers-2f-src

