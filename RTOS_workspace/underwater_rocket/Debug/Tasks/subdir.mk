################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Tasks/communication.c \
../Tasks/tasks_app.c 

OBJS += \
./Tasks/communication.o \
./Tasks/tasks_app.o 

C_DEPS += \
./Tasks/communication.d \
./Tasks/tasks_app.d 


# Each subdirectory must supply rules for building sources it contributes
Tasks/%.o Tasks/%.su Tasks/%.cyclo: ../Tasks/%.c Tasks/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Tasks" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/BLUETHOOTH" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/BNO055" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/MAESTRO" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/MS5837" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Algorithm+Sensors/PID" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/SEGGER/Config" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/SEGGER/OS" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/SEGGER/SEGGER" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/FreeRTOS" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/FreeRTOS/include" -I"C:/Users/husey/OneDrive/Masaüstü/Underwater_rocket/RTOS_workspace/underwater_rocket/Thirdparty/FreeRTOS/portable/GCC/ARM_CM4F" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Tasks

clean-Tasks:
	-$(RM) ./Tasks/communication.cyclo ./Tasks/communication.d ./Tasks/communication.o ./Tasks/communication.su ./Tasks/tasks_app.cyclo ./Tasks/tasks_app.d ./Tasks/tasks_app.o ./Tasks/tasks_app.su

.PHONY: clean-Tasks

