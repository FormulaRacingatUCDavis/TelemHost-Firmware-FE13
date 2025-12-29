################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/xsens/xsens_mdata2.c \
../Core/Src/xsens/xsens_mti.c \
../Core/Src/xsens/xsens_utility.c 

OBJS += \
./Core/Src/xsens/xsens_mdata2.o \
./Core/Src/xsens/xsens_mti.o \
./Core/Src/xsens/xsens_utility.o 

C_DEPS += \
./Core/Src/xsens/xsens_mdata2.d \
./Core/Src/xsens/xsens_mti.d \
./Core/Src/xsens/xsens_utility.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/xsens/%.o Core/Src/xsens/%.su Core/Src/xsens/%.cyclo: ../Core/Src/xsens/%.c Core/Src/xsens/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/dp83848 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-xsens

clean-Core-2f-Src-2f-xsens:
	-$(RM) ./Core/Src/xsens/xsens_mdata2.cyclo ./Core/Src/xsens/xsens_mdata2.d ./Core/Src/xsens/xsens_mdata2.o ./Core/Src/xsens/xsens_mdata2.su ./Core/Src/xsens/xsens_mti.cyclo ./Core/Src/xsens/xsens_mti.d ./Core/Src/xsens/xsens_mti.o ./Core/Src/xsens/xsens_mti.su ./Core/Src/xsens/xsens_utility.cyclo ./Core/Src/xsens/xsens_utility.d ./Core/Src/xsens/xsens_utility.o ./Core/Src/xsens/xsens_utility.su

.PHONY: clean-Core-2f-Src-2f-xsens

