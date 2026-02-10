################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Algorithm+Sensors/BNO055/bno055.c 

OBJS += \
./Algorithm+Sensors/BNO055/bno055.o 

C_DEPS += \
./Algorithm+Sensors/BNO055/bno055.d 


# Each subdirectory must supply rules for building sources it contributes
Algorithm+Sensors/BNO055/%.o Algorithm+Sensors/BNO055/%.su Algorithm+Sensors/BNO055/%.cyclo: ../Algorithm+Sensors/BNO055/%.c Algorithm+Sensors/BNO055/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Tasks" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/BLUETHOOTH" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/BNO055" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/MAESTRO" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/MS5837" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/PID" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/SEGGER/Config" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/SEGGER/OS" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/SEGGER/SEGGER" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/FreeRTOS" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/FreeRTOS/include" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/FreeRTOS/portable/GCC/ARM_CM4F" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Algorithm-2b-Sensors-2f-BNO055

clean-Algorithm-2b-Sensors-2f-BNO055:
	-$(RM) ./Algorithm+Sensors/BNO055/bno055.cyclo ./Algorithm+Sensors/BNO055/bno055.d ./Algorithm+Sensors/BNO055/bno055.o ./Algorithm+Sensors/BNO055/bno055.su

.PHONY: clean-Algorithm-2b-Sensors-2f-BNO055

