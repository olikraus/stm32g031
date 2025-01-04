
Notes: 
  * The below picture has an error: IN1 is connected to PB9 (TSOP20 Pin 2), which is below PB7/8.
  * A single 1 Ohm resistor towards power ground is used to measure the current.
  * Current sense is only possible during forward/backward operation. During "break" or "coast" phase of the PWM nothing can be measured.
  * During forward/backward phase, if no current is measured, then the DC motor is disconnected (not present)

![https://raw.githubusercontent.com/olikraus/stm32g031/main/motor_test_8/motor_test_schematic.jpg](https://raw.githubusercontent.com/olikraus/stm32g031/main/motor_test_8/motor_test_schematic.jpg)

