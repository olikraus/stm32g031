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


DRV8871:
IN1     IN2     OUT1    OUT2            DESCRIPTION
0         0         High-Z  High-Z           Coast; H-bridge disabled to High-Z (sleep entered after 1 ms)
0         1         L           H                   Reverse (Current OUT2 → OUT1)
1         0         H           L                   Forward (Current OUT1 → OUT2)
1         1         L           L                    Brake; low-side slow decay

--> no measurement possible in Coast and Break mode

https://electronics.stackexchange.com/questions/674622/braking-vs-coasting-during-the-pwm-off-state-of-a-h-bridge

Coast vs Break
Coast: High voltage and probably High current back --> more stressful for the FETs
Break: Coil current can continue as usual 

*/

#include "stm32g0xx.h"
#include "delay.h"
#include "sys_util.h"
#include "adc.h"
#include "u8g2.h"
#include <string.h>     /* memcpy */

uint8_t u8x8_gpio_and_delay_stm32g0(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
/*===========================================*/
/* global variables */
volatile uint16_t varpot = 0; // PA4, IN4, updated via DMA
volatile uint16_t refint = 0;  // bandgap reference 1212mV, updated via DMA

volatile uint32_t adc_motor_ticks = 0;
volatile uint32_t adc_varpot_ticks = 0;
volatile uint32_t adc_refint_ticks = 0;
volatile uint32_t update_adc_ticks_start = 0;
volatile uint32_t *update_adc_ticks_address = NULL;     // the addres of the global variable, which should be updated
volatile uint32_t tim17_peroidic_ticks = 0;                     // number of ticks between calls to TIM17 irq (5000Hz --> 12200 ticks)
volatile uint32_t tim17_peroidic_ticks_start = 0;               // start value for calculation of tim17_peroidic_ticks

/*===========================================*/
/* Utility Procedures */

/*
  Measure the number or processor clock cycles:
  uint32_t start;

  start = SysTick->VAL;
  ...
  getProcessorClockDelta(start); // retun the number of processor cycles since start was recorded

  Limitation: The number of cycles between start and getProcessorClockDelta(start) must not exceed Systick->LOAD

*/
uint32_t getProcessorClockDelta(uint32_t start_value)
{
  uint32_t current_value = SysTick->VAL;
  /* SysTick->VAL is decremented, so the simple case is current_value < start_value */
  if ( current_value < start_value )
    return start_value-current_value;
  /* reload happend since start_value */
  return SysTick->LOAD - current_value + start_value;
}


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
  //GPIOB->BSRR = GPIO_BSRR_BR9;		/* atomic clr */
  GPIOA->BSRR = GPIO_BSRR_BS9;		/* atomic set: default value for IN1 to H --> defaults to break mode */
  GPIOB->AFR[1] &= ~(15 << 4);
  GPIOB->AFR[1] |= 0 << 4;   // AF0: IR Interface IR_OUT

  /*=== PA13: DRV8871 IN2 ===*/
  
  GPIOA->MODER &= ~GPIO_MODER_MODE13;	/* clear mode */
  GPIOA->MODER |= GPIO_MODER_MODE13_1;	/* Alternate Function mode */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT13;	/* no Push/Pull */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED13;	/* low speed */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD13;	/* no pullup/pulldown */
  //GPIOA->BSRR = GPIO_BSRR_BR13;		/* atomic clr */
  GPIOA->BSRR = GPIO_BSRR_BS13;		/* atomic set: default value for IN2 to H --> defaults to break mode */
  GPIOA->AFR[1] &= ~(15 << 20);
  GPIOA->AFR[1] |= 1 << 20;   // AF1: IR Interface IR_OUT

  /*=== TIM17 ===*/
  /* CK_INT = 64 MHz */

  /* configuration for PWM */
  TIM17->CR1 = 0;               // all default values
  TIM17->CR2 = 0;               // all default values
  TIM17->CCER = TIM_CCER_CC1E;                   // default values, output mode, output enable, polarity not inverted
                            //| TIM_CCER_CC1P;              // inverted polarity
  TIM17->BDTR = TIM_BDTR_MOE;                   // main output enable (REQUIRED!)
  TIM17->CCMR1 = (6<<TIM_CCMR1_OC1M_Pos)        // PWM Mode 1 (output high if cnt<cmp)
                            | TIM_CCMR1_OC1PE   // preload for counter
                            ;
        
  /* prescaler */
  TIM17->PSC = prescaler-1;
  /* auto reload register */
  TIM17->ARR = TIM17_ARR;
  /* Compare Register */
  TIM17->CCR1 = TIM17_ARR/2;            // 50% duty as a default

  TIM17->DIER = 0;
  TIM17->DIER |= TIM_DIER_CC1IE;  // enable irq if counter reaches CCR1 value
  //TIM17->DIER |= TIM_DIER_UIE; // enable irq if counter is reseted to 0 (ARR value reached)
  
  tim17_peroidic_ticks_start = SysTick->VAL;            // at least get some suitable starting point
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

  DRV8871:
  IN1     IN2     OUT1    OUT2            DESCRIPTION
  0         0         High-Z  High-Z           Coast; H-bridge disabled to High-Z (sleep entered after 1 ms)
  0         1         L           H                   Reverse (Current OUT2 → OUT1)
  1         0         H           L                   Forward (Current OUT1 → OUT2)
  1         1         L           L                    Brake; low-side slow decay

*/
void tim17_set_duty(uint16_t duty, uint16_t is_backward)
{
  TIM17->CCR1 = ((1<<TIM17_BIT_CNT)-1) - duty;
  /* 
    the idea is to switch between the TIM17 output and the GPIO output 
    GPIO output is always 1 (assigned during init), so the below code will swap IN1 and IN2 connection.
  
    The TIM17 output will be either connected to IN1 or IN2 whole the constant 1 is applied to the other INx
    As a result, the DRV8871 H-Bridge toggles between forward/backward and break mode  
  */
  if ( is_backward )
  {
    GPIOA->MODER &= ~GPIO_MODER_MODE13;	/* clear mode */
    GPIOA->MODER |= GPIO_MODER_MODE13_1;	/* Alternate Function mode */
    
    GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode */
    GPIOB->MODER |= GPIO_MODER_MODE9_0;	/* output mode */    
    GPIOB->BSRR = GPIO_BSRR_BS9;

  }
  else
  {
    GPIOA->MODER &= ~GPIO_MODER_MODE13;	/* clear mode */
    GPIOA->MODER |= GPIO_MODER_MODE13_0;	/* output mode */
    
    GPIOB->MODER &= ~GPIO_MODER_MODE9;	/* clear mode */
    GPIOB->MODER |= GPIO_MODER_MODE9_1;	/* Alternate Function mode */
    GPIOA->BSRR = GPIO_BSRR_BS13;
  }
}

/*===========================================*/

#define ADC_RAW_SAMPLE_CNT 128
uint16_t adc_raw_sample_array[ADC_RAW_SAMPLE_CNT] __attribute__ ((aligned (4)));
uint16_t adc_copy_sample_array[ADC_RAW_SAMPLE_CNT] __attribute__ ((aligned (4)));

#define IRQ_STATE_START_MOTOR_ADC 0
#define IRQ_STATE_WAIT_MOTOR_ADC 1
#define IRQ_STATE_ANALYSIS_1 2
#define IRQ_STATE_ANALYSIS_2 3
#define IRQ_STATE_ANALYSIS_3 4
#define IRQ_STATE_SUPPLY 5
#define IRQ_STATE_VARPOT 6


uint16_t tim17_irq_state = IRQ_STATE_START_MOTOR_ADC;
uint16_t tim17_irq_cnt = 0;
uint16_t dma_busy_cnt = 0;
uint16_t varpot_skip_cnt = 0;

void __attribute__ ((interrupt, used)) TIM17_IRQHandler(void)
{
  tim17_peroidic_ticks = getProcessorClockDelta(tim17_peroidic_ticks_start);
  tim17_peroidic_ticks_start = SysTick->VAL;
  ++tim17_irq_cnt;

  switch( tim17_irq_state )
  {
    case IRQ_STATE_START_MOTOR_ADC:
      adc_get_multiple_values(adc_raw_sample_array, ADC_RAW_SAMPLE_CNT, 3);               // this will just start the sampling process
      update_adc_ticks_start = SysTick->VAL;
      update_adc_ticks_address = &adc_motor_ticks; 
      dma_busy_cnt = 0;
      tim17_irq_state = IRQ_STATE_WAIT_MOTOR_ADC;
      break;
    case IRQ_STATE_WAIT_MOTOR_ADC:
      if ( DMA1_Channel1->CNDTR == 0 )
      {
        memcpy(adc_copy_sample_array, adc_raw_sample_array, ADC_RAW_SAMPLE_CNT*sizeof(uint16_t));
        tim17_irq_state = IRQ_STATE_ANALYSIS_1;
      }
      else
      {
        dma_busy_cnt++;
        if ( dma_busy_cnt > 16 )
        {
          tim17_irq_state = IRQ_STATE_SUPPLY;
        }
      }
      break;
    case IRQ_STATE_ANALYSIS_1:
      tim17_irq_state = IRQ_STATE_ANALYSIS_2;
      break;
    case IRQ_STATE_ANALYSIS_2:
      tim17_irq_state = IRQ_STATE_ANALYSIS_3;
      break;
    case IRQ_STATE_ANALYSIS_3:
      tim17_irq_state = IRQ_STATE_SUPPLY;
      break;
    case IRQ_STATE_SUPPLY:
      adc_get_multiple_values(&refint, 1, 13);               // read bandgap value
      update_adc_ticks_start = SysTick->VAL;
      update_adc_ticks_address = &adc_refint_ticks; 
      tim17_irq_state = IRQ_STATE_VARPOT;
      break;
    case IRQ_STATE_VARPOT:
      if ( varpot_skip_cnt == 0 )
      {
        adc_get_multiple_values(&varpot, 1, 4);               // read var pot value
        update_adc_ticks_start = SysTick->VAL;
        update_adc_ticks_address = &adc_varpot_ticks;
        varpot_skip_cnt = 200;
      }
      else
      {
        varpot_skip_cnt--;
      }
      tim17_irq_state = IRQ_STATE_START_MOTOR_ADC;
      break;
    default:
      tim17_irq_state = IRQ_STATE_START_MOTOR_ADC;
      break;
  }
  
  TIM17->SR = 0;        // status must be cleared otherwise another IRQ is generated
  
}

uint16_t dma_irq_cnt = 0;
void __attribute__ ((interrupt, used)) DMA1_Channel1_IRQHandler(void)
{
  
  
  if ( update_adc_ticks_address != NULL )
  {
    *update_adc_ticks_address = getProcessorClockDelta(update_adc_ticks_start);
  }
  
  
  dma_irq_cnt++;
  DMA1->IFCR = DMA_IFCR_CTCIF1;       // clear the transfer complete IRQ
}

/*===========================================*/

volatile unsigned long adc_irq_cnt = 0;


void adc_enable_interrupt(void)
{
  ADC1->ISR |= ADC_ISR_EOS;             /* clear the end of sequence bit */
  NVIC_SetPriority( ADC1_IRQn, 2);      // 3: lowest priority, 0: highest priority
  NVIC_EnableIRQ(ADC1_IRQn);
  ADC1->IER |= ADC_IER_EOCIE;             /* enable end of sequence interrupt */
} 

void __attribute__ ((interrupt, used)) ADC1_IRQHandler(void)
{
  ADC1->ISR |= ADC_ISR_EOS;             /* clear the end of sequence bit */
  adc_irq_cnt++;
}


/*===========================================*/

volatile unsigned long SysTickCount = 0;


void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  // PA3 is the current senser of the dc motor
  //if ( SysTickCount & 1 )
  //  GPIOA->BSRR = GPIO_BSRR_BS3;		/* atomic set PA3 */
  //else
  //  GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */
}


/*===========================================*/

u8g2_t u8g2;

void drawDisplay(void)
{
  uint16_t v;
  uint16_t i;
  uint16_t motor_current = adc_get_value(3); // PA3, IN3 
  //varpot = adc_get_value(4); // PA4, IN4 
  //refint = adc_get_value(13);  // bandgap reference 1212mV
  uint16_t supply = (4095UL*1212UL)/refint;


  u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
  u8g2_ClearBuffer(&u8g2);
  //u8g2_DrawStr(&u8g2, 0,12, "STM32G031");
  //u8g2_DrawStr(&u8g2, 70,12, u8x8_u8toa(SystemCoreClock/1000000, 2));
  //u8g2_DrawStr(&u8g2, 85,12, "MHz");
  
  u8g2_DrawStr(&u8g2, 0,8, "mV");
  u8g2_DrawStr(&u8g2, 25,8, "Pot");
  u8g2_DrawStr(&u8g2, 50,8, "Duty");
  u8g2_DrawStr(&u8g2, 75,8, "Current");

  u8g2_DrawStr(&u8g2, 0,16, u8x8_u16toa(supply, 4));
  u8g2_DrawStr(&u8g2, 25,16, u8x8_u16toa(varpot, 4));
  u8g2_DrawStr(&u8g2, 50,16, u8x8_u16toa(TIM17->CCR1, 4));
  u8g2_DrawStr(&u8g2, 75,16, u8x8_u16toa(motor_current, 4));

  //u8g2_DrawStr(&u8g2, 0,24, "TIM17 irq cnt:");
  //u8g2_DrawStr(&u8g2, 70,24, u8x8_u16toa(tim17_irq_cnt, 5));

  //u8g2_DrawStr(&u8g2, 0,24, "ADC busy:");
  //u8g2_DrawStr(&u8g2, 70,24, u8x8_u16toa(dma_busy_cnt, 5));

  //u8g2_DrawStr(&u8g2, 0,24, "DMA irq cnt:");
  //u8g2_DrawStr(&u8g2, 70,24, u8x8_u16toa(dma_irq_cnt, 5));

  u8g2_DrawStr(&u8g2, 0,24, "Motor/T17:");
  u8g2_DrawStr(&u8g2, 70,24, u8x8_u16toa((uint16_t)adc_motor_ticks, 5));
  u8g2_DrawStr(&u8g2, 100,24, u8x8_u16toa((uint16_t)tim17_peroidic_ticks, 5));



  for( i = 0; i <  ADC_RAW_SAMPLE_CNT; i++ )
  {
    v = adc_copy_sample_array[i];
    v >>= 5;
    if ( v > 40 )
      v = 40;
    u8g2_DrawPixel( &u8g2, i, 63 - v );
  }
  
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
  
  // TODO: PA3 needs to be configured as analog in for the DC motor
  //GPIOA->MODER &= ~GPIO_MODER_MODE3;	/* clear mode for PA3 */
  //GPIOA->MODER |= GPIO_MODER_MODE3_0;	/* Output mode for PA3 */
  //GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;	/* no Push/Pull for PA3 */
  //GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3;	/* low speed for PA3 */
  //GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3;	/* no pullup/pulldown for PA3 */
  //GPIOA->BSRR = GPIO_BSRR_BR3;		/* atomic clr PA3 */
  
  SysTick->LOAD = 16000*500 - 1;        // Blink with 4 Hz 
  NVIC_SetPriority( SysTick_IRQn, 3 );  // 3: lowest priority
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
  
  adc_init();
  //adc_enable_interrupt();
  initDisplay();
  hardware_init(4000);
  
  NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  
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

