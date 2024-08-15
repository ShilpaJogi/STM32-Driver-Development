################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/10i2c_slave_tx_string.c \
../Src/main.c \
../Src/sysmem.c 

OBJS += \
./Src/10i2c_slave_tx_string.o \
./Src/main.o \
./Src/sysmem.o 

C_DEPS += \
./Src/10i2c_slave_tx_string.d \
./Src/main.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"/home/shilpa/Desktop/STM32/workspace/Project/stm32f446xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/10i2c_slave_tx_string.cyclo ./Src/10i2c_slave_tx_string.d ./Src/10i2c_slave_tx_string.o ./Src/10i2c_slave_tx_string.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

