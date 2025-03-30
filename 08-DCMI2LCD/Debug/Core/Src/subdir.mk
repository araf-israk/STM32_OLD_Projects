################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/board.c \
../Core/Src/camera.c \
../Core/Src/dcmi.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/lcd.c \
../Core/Src/logo_128_160.c \
../Core/Src/logo_160_80.c \
../Core/Src/main.c \
../Core/Src/ov2640.c \
../Core/Src/ov2640_regs.c \
../Core/Src/ov5640.c \
../Core/Src/ov5640_regs.c \
../Core/Src/ov7670.c \
../Core/Src/ov7670_regs.c \
../Core/Src/ov7725.c \
../Core/Src/ov7725_regs.c \
../Core/Src/rtc.c \
../Core/Src/spi.c \
../Core/Src/st7735.c \
../Core/Src/st7735_reg.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c 

OBJS += \
./Core/Src/board.o \
./Core/Src/camera.o \
./Core/Src/dcmi.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/lcd.o \
./Core/Src/logo_128_160.o \
./Core/Src/logo_160_80.o \
./Core/Src/main.o \
./Core/Src/ov2640.o \
./Core/Src/ov2640_regs.o \
./Core/Src/ov5640.o \
./Core/Src/ov5640_regs.o \
./Core/Src/ov7670.o \
./Core/Src/ov7670_regs.o \
./Core/Src/ov7725.o \
./Core/Src/ov7725_regs.o \
./Core/Src/rtc.o \
./Core/Src/spi.o \
./Core/Src/st7735.o \
./Core/Src/st7735_reg.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o 

C_DEPS += \
./Core/Src/board.d \
./Core/Src/camera.d \
./Core/Src/dcmi.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/lcd.d \
./Core/Src/logo_128_160.d \
./Core/Src/logo_160_80.d \
./Core/Src/main.d \
./Core/Src/ov2640.d \
./Core/Src/ov2640_regs.d \
./Core/Src/ov5640.d \
./Core/Src/ov5640_regs.d \
./Core/Src/ov7670.d \
./Core/Src/ov7670_regs.d \
./Core/Src/ov7725.d \
./Core/Src/ov7725_regs.d \
./Core/Src/rtc.d \
./Core/Src/spi.d \
./Core/Src/st7735.d \
./Core/Src/st7735_reg.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/board.cyclo ./Core/Src/board.d ./Core/Src/board.o ./Core/Src/board.su ./Core/Src/camera.cyclo ./Core/Src/camera.d ./Core/Src/camera.o ./Core/Src/camera.su ./Core/Src/dcmi.cyclo ./Core/Src/dcmi.d ./Core/Src/dcmi.o ./Core/Src/dcmi.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/lcd.cyclo ./Core/Src/lcd.d ./Core/Src/lcd.o ./Core/Src/lcd.su ./Core/Src/logo_128_160.cyclo ./Core/Src/logo_128_160.d ./Core/Src/logo_128_160.o ./Core/Src/logo_128_160.su ./Core/Src/logo_160_80.cyclo ./Core/Src/logo_160_80.d ./Core/Src/logo_160_80.o ./Core/Src/logo_160_80.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ov2640.cyclo ./Core/Src/ov2640.d ./Core/Src/ov2640.o ./Core/Src/ov2640.su ./Core/Src/ov2640_regs.cyclo ./Core/Src/ov2640_regs.d ./Core/Src/ov2640_regs.o ./Core/Src/ov2640_regs.su ./Core/Src/ov5640.cyclo ./Core/Src/ov5640.d ./Core/Src/ov5640.o ./Core/Src/ov5640.su ./Core/Src/ov5640_regs.cyclo ./Core/Src/ov5640_regs.d ./Core/Src/ov5640_regs.o ./Core/Src/ov5640_regs.su ./Core/Src/ov7670.cyclo ./Core/Src/ov7670.d ./Core/Src/ov7670.o ./Core/Src/ov7670.su ./Core/Src/ov7670_regs.cyclo ./Core/Src/ov7670_regs.d ./Core/Src/ov7670_regs.o ./Core/Src/ov7670_regs.su ./Core/Src/ov7725.cyclo ./Core/Src/ov7725.d ./Core/Src/ov7725.o ./Core/Src/ov7725.su ./Core/Src/ov7725_regs.cyclo ./Core/Src/ov7725_regs.d ./Core/Src/ov7725_regs.o ./Core/Src/ov7725_regs.su ./Core/Src/rtc.cyclo ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/rtc.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/st7735.cyclo ./Core/Src/st7735.d ./Core/Src/st7735.o ./Core/Src/st7735.su ./Core/Src/st7735_reg.cyclo ./Core/Src/st7735_reg.d ./Core/Src/st7735_reg.o ./Core/Src/st7735_reg.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su

.PHONY: clean-Core-2f-Src

