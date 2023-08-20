


* all files (except tz_context.h) from 
  https://github.com/STMicroelectronics/STM32CubeG0/tree/master/Drivers/CMSIS/Include

 * startup_stm32g031xx.s from 
  https://github.com/STMicroelectronics/STM32CubeG0/blob/master/Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/gcc/startup_stm32g031xx.s

 * system_stm32g0xx.c from 
  https://github.com/STMicroelectronics/STM32CubeG0/blob/master/Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/system_stm32g0xx.c

 * all files from from 
  https://github.com/STMicroelectronics/STM32CubeG0/tree/master/Drivers/CMSIS/Device/ST/STM32G0xx/Include

 * linker script copied and renamed to stm32g031x8.ld
  https://github.com/STMicroelectronics/STM32CubeG0/blob/master/Projects/NUCLEO-G031K8/Examples/Cortex/CORTEXM_SysTick/STM32CubeIDE/STM32G031K8TX_FLASH.ld

* Aug 2023: The STM32G031J6 is not available any more,  moving to the STM32G030J6 instead (which doen't have low power timer and usart), added https://github.com/STMicroelectronics/STM32CubeG0/blob/master/Drivers/CMSIS/Device/ST/STM32G0xx/Source/Templates/gcc/startup_stm32g030xx.s


