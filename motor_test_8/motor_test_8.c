/* 

  DC motor test with the STM32G031

  Copyright (c) 2023, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  


  Default clock is HSI16 (16 MHz)
    
  Ensure that -DUSER_VECT_TAB_ADDRESS is set during compilation, otherwise
  interrupts will not work after the "go" commant of the flashware USART upload.
  
  Problem is, that SCB->VTOR doesn't point to FLASH_BASE, so manually
  assigning SCB->VTOR = FLASH_BASE;	will also fix this.

  If -DUSER_VECT_TAB_ADDRESS is defined, then the SCB->VTOR is set in
  SystemInit(void) which is called by the reset handler (.s file).

  This code was tested with my STM32G031 TSOP20 board
  
  OLED
    SCL @PA2
    SDA @PA1

  Varpot:
    PA4 (ADC IN4)
  
  LEDs (future H-Bridge inputs) at 
    A13
    B9
  
  The code will show the value (12 bit) of the varpot on the OLED screen.

TIM17 could trigger TIM1, which in turn could trigger ADC
  --> Chapter 21.3.26 TIM1 Slave mode: Combined reset + trigger mode

input start trigger for ADC EXTSEL[2:0] in CFGR1
TRG0    TIM1_TRGO2  000
TRG1    TIM1_CC4    001
TRG2    TIM2_TRGO   010
TRG3    TIM3_TRGO   011
TRG4    TIM15_TRGO  100
TRG5    TIM6_TRGO   101
TRG6    TIM4_TRGO   110
TRG7    EXTI11      111

*/

#include "stm32g0xx.h"
#include "delay.h"
#include "sys_util.h"
#include "adc.h"
#include "u8g2.h"

uint8_t u8x8_gpio_and_delay_stm32g0(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

/*===========================================*/

/*
  TIM17: PWM generation
  Output: PB9 and PA13 via IR_OUT (both pins, PB9 and PA13 are connected to IR_OUT depending on the direction
  TIM16 must be permanently 1 for IR_OUT
    IR_OUT = XOR(IR_POL, NAND(TIM16_CH1, TIM17_CH1))
      --> set IR_POL polarity 1, to compensate NAND 
      TIM16_CH1 must be permanently high
        --> TIM16 MOE (master output enable) not active
        --> TIM16 OSSI but enable
        --> TIM16 CCxE enabled
        --> TIM16 OCxP set to 1 (so that OCx becomes 1)
*/

#define TIM17_BIT_CNT 11
#define TIM17_ARR ((1<<(TIM17_BIT_CNT))-1)

void hardware_init(uint16_t hz)
{
  uint16_t prescaler = (SystemCoreClock >> TIM17_BIT_CNT)/(uint32_t)hz;
  
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;		/* Enable clock for GPIO Port B */
  RCC->APBENR2 |= RCC_APBENR2_TIM1EN;		/* Enable TIM1: Trigger for ADC */
  RCC->APBENR2 |= RCC_APBENR2_TIM16EN;		/* Enable TIM16: Fixed to 1 for TIM17 gate in IR Interface*/
  RCC->APBENR2 |= RCC_APBENR2_TIM17EN;		/* Enable TIM17: PWM Generator for the DC Motor */
  RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;		/* Enable SysCfg  */
  __NOP();
  __NOP();

  /*=== IR Interface ===*/
  
  /* TIM17_CH1 is routed via IR interface */
  /* IR_OUT = POLARITY ( TIM16_CH1 NAND TIM17_CH1 ) --> Polarity is "NOT" to compansate the NAND */
  SYSCFG->CFGR1 |= SYSCFG_CFGR1_IR_POL; /* assign 1 to the polarity flag to compensate the NAND */
  
  /* IR mode 0: Use TIM16_CH1 */
  SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_IR_MOD;   
  
  /*=== PB9 ===*/
  
  GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode */
  GPIOB->MODER |= GPIO_MODER_MODE9_1;	/* Alternate Function mode */
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT9;	/* no Push/Pull */
  GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9;	/* low speed */
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD9;	/* no pullup/pulldown */
  GPIOB->BSRR = GPIO_BSRR_BR9;		/* atomic clr */
  GPIOB->AFR[1] &= ~(15 << 4);
  GPIOB->AFR[1] |= 0 << 4;   // AF0: IR Interface IR_OUT

  /*=== PA13: DRV8871 IN2 ===*/
  
  GPIOA->MODER &= ~GPIO_MODER_MODE13;	/* clear mode */
  GPIOA->MODER |= GPIO_MODER_MODE13_1;	/* Alternate Function mode */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT13;	/* no Push/Pull */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED13;	/* low speed */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD13;	/* no pullup/pulldown */
  GPIOA->BSRR = GPIO_BSRR_BR13;		/* atomic clr */
  GPIOA->AFR[1] &= ~(15 << 20);
  GPIOA->AFR[1] |= 1 << 20;   // AF1: IR Interface IR_OUT

  /*=== TIM17 ===*/

  /* configuration for PWM */
  TIM17->CR1 = 0;               // all default values
  TIM17->CR2 = 0;               // all default values
  TIM17->CCER = TIM_CCER_CC1E;              // default values, output mode, output enable
  TIM17->BDTR = TIM_BDTR_MOE;                   // main output enable (REQUIRED!)
  TIM17->CCMR1 = (6<<TIM_CCMR1_OC1M_Pos)        // PWM Mode 1 (output high if cnt<cmp)
                            | TIM_CCMR1_OC1PE   // preload for counter
                            ;
        
  /* prescaler */
  TIM17->PSC = prescaler-1;
  /* auto reload register */
  TIM17->ARR = TIM17_ARR;
  /* Compare Register */
  TIM17->CCR1 = TIM17_ARR/2;

  TIM17->DIER = 0;
  TIM17->DIER |= TIM_DIER_CC1IE;
  //TIM17->DIER |= TIM_DIER_UIE;
  
  NVIC_SetPriority(TIM17_IRQn, 0);      // level 0 is highest prio (PM0223)
  NVIC_EnableIRQ(TIM17_IRQn);

  /* CR1 configuration */
  TIM17->CR1 = TIM_CR1_ARPE  // buffered output for ARR register
                        | TIM_CR1_CEN   // enable counter
                        ;



  /*=== TIM1 ===*/
  /* TIM1 will be triggered by TIM17 to generate the ADC start event (which can't be done by TIM17 */
  
  /* use default values for control reg. (up counting) */
  /* this will use the reset event for TRGO0 and TRGO2 output trigger lines */
  TIM1->CR1 = 0;
  TIM1->CR2 = 0;                // Reset event set to TRGO2 (for ADC)

  TIM1->CR1 |= TIM_CR1_OPM;             // one pulse mode

  TIM1->SMCR = 0;       // clear the slave mode control registier
  //TIM1->SMCR |= TIM_SMCR_SMS_0; 
  //TIM1->SMCR |= TIM_SMCR_SMS_1; 
  //TIM1->SMCR |= TIM_SMCR_SMS_2; 
  TIM1->SMCR |= TIM_SMCR_SMS_3; // select slave mode: Combined reset + trigger mode on rising edge of trigger input
  
  TIM1->SMCR |= TIM_SMCR_TS_0 | TIM_SMCR_TS_1; //ITR3: TIM17 OC1 as Trigger Source 

  TIM1->PSC = 50000;  // prescaler
  TIM1->CNT = 0;
  TIM1->ARR = 0xfff0;
  
  //TIM1->CR1 |= TIM_CR1_CEN;             // enable TIM1: not required, will by done by trigger event
  


  /*=== TIM16 ===*/
  
  /* Force TIM16 output to high so that TIM17 output is passed through IR Interface */
  
  TIM16->CR1 = 0;               // all default values
  TIM16->CR2 = 0;               // all default values
  TIM16->CCER = TIM_CCER_CC1E;              // default values, output mode, output enable
  TIM16->BDTR = TIM_BDTR_MOE;                   // main output enable (REQUIRED!)
  TIM16->CCMR1 = 5<<TIM_CCMR1_OC1M_Pos;        // force high
}

/*
  duty:
    0 .. (1<<TIM17_BIT_CNT)-1
*/
void tim17_set_duty(uint16_t duty, uint16_t is_backward)
{
  TIM17->CCR1 = duty;
  if ( is_backward )
  {
    GPIOA->MODER &= ~GPIO_MODER_MODE13;	/* clear mode */
    GPIOA->MODER |= GPIO_MODER_MODE13_1;	/* Alternate Function mode */
    
    GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode */
    GPIOB->MODER |= GPIO_MODER_MODE9_0;	/* output mode */    
  }
  else
  {
    GPIOA->MODER &= ~GPIO_MODER_MODE13;	/* clear mode */
    GPIOA->MODER |= GPIO_MODER_MODE13_0;	/* output mode */
    
    GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode */
    GPIOB->MODER |= GPIO_MODER_MODE9_1;	/* Alternate Function mode */
  }
}

#define ADC_RAW_SAMPLE_CNT 2
uint16_t adc_raw_sample_array[ADC_RAW_SAMPLE_CNT+100] __attribute__ ((aligned (4)));

void __attribute__ ((interrupt, used)) TIM17_IRQHandler(void)
{
  uint16_t sr = TIM17->SR;
  //if ( sr & TIM_SR_UIF )
  //{  
    adc_get_multiple_values(adc_raw_sample_array, ADC_RAW_SAMPLE_CNT, 4);
  //}
  TIM17->SR = 0;        // status must be cleared otherwise another IRQ is generated
  
}


/*===========================================*/

u8g2_t u8g2;
volatile unsigned long SysTickCount = 0;


void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS3;		/* atomic set PA3 */
  else
    GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */
}



void drawDisplay(void)
{
  uint16_t refint = adc_get_value(13);  // bandgap reference 1212mV
  uint16_t supply = (4095UL*1212UL)/refint;


  u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
  u8g2_ClearBuffer(&u8g2);
  u8g2_DrawStr(&u8g2, 0,12, "STM32G031");
  u8g2_DrawStr(&u8g2, 70,12, u8x8_u8toa(SystemCoreClock/1000000, 2));
  u8g2_DrawStr(&u8g2, 85,12, "MHz");

  u8g2_DrawStr(&u8g2, 0,24, "TIM1:");
  u8g2_DrawStr(&u8g2, 75,24, u8x8_u16toa(TIM1->CNT, 5));

  
  u8g2_DrawStr(&u8g2, 0,36, "PA4:");
  u8g2_DrawStr(&u8g2, 40,36, u8x8_u16toa(adc_raw_sample_array[0], 4));
  u8g2_DrawStr(&u8g2, 70,36, u8x8_u16toa(adc_raw_sample_array[1], 4));

  u8g2_DrawStr(&u8g2, 0,48, "PA4:");
  u8g2_DrawStr(&u8g2, 75,48, u8x8_u16toa(adc_get_value(4), 5));

  
  //u8g2_DrawStr(&u8g2, 0,36, "SysTick:");
  //u8g2_DrawStr(&u8g2, 75,36, u8x8_u16toa(SysTickCount, 5));


  u8g2_DrawStr(&u8g2, 0,60, "Supply (mV):");
  u8g2_DrawStr(&u8g2, 75,60, u8x8_u16toa(supply, 4));
  
  
  
  u8g2_SendBuffer(&u8g2);
}

void initDisplay(void)
{
  /* setup display */
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_stm32g0);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
}

int main()
{
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		/* Enable clock for GPIO Port A */
  __NOP();
  __NOP();

  set_64mhz_sysclk(); 
  
  GPIOA->AFR[0] &= ~(0xf << (3*4));       /* clear alternative function */
  //GPIOA->AFR[0] = 0;       /* clear alternative function */
  
  GPIOA->MODER &= ~GPIO_MODER_MODE3;	/* clear mode for PA3 */
  GPIOA->MODER |= GPIO_MODER_MODE3_0;	/* Output mode for PA3 */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;	/* no Push/Pull for PA3 */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3;	/* low speed for PA3 */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3;	/* no pullup/pulldown for PA3 */
  GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */
  
  SysTick->LOAD = 16000*500 - 1;        // Blink with 4 Hz 
  NVIC_SetPriority( SysTick_IRQn, 3 );  // 3: lowest priority
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
  
  adc_init();
  initDisplay();
  hardware_init(10);
  
  for(;;)
  {

    drawDisplay();
    
    uint16_t adc = adc_get_value(4);  // 12 Bit
    uint16_t inv = adc >= (1<<11)?1:0;
    if ( inv )
      adc = adc - (1<<11);
    else
      adc = (1<<11) - 1 - adc;
      
    tim17_set_duty(adc, inv);
  }
}

