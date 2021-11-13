
### Background:
The BOOT0 pin is disabled for STM32G0 devices by default: The bootloader
will not be executed with a high level applied to BOOT0 pin during reset.
This means: The bootloader is not executed any more after first successful upload of any code.
As a result the upload via UART can be only done once for an empty device.
    
### Solution:
Upload and execute this code: It will enable the BOOT0 pin behavior, so that
a high level on BOOT0 pin during reset will force the bootloader to execute.
    
### Instructions for UART uploads
This sequence needs to be executed once for a new / empty STM32G0 device:
  1. Generate hex file from this code
  2. Upload and execute the generated hex file via UART (stm32flash -g 0)
  3. Wait for 1 second: The BOOT0 pin is now activated
  4. Upload your own code
