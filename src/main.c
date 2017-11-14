//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "lcd.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

// TIM2 clock prescaler
#define myTIM2_PRESCALER (uint16_t)0x0000
// LCD clock prescaler
#define myLCD_PRESCALER (uint16_t)((SystemCoreClock - 1) / 1000)

// Maximum possible setting before overflow
#define myTIM2_PERIOD (uint32_t)0xFFFFFFFF
#define myLCD_UPDATE_DELAY (uint32_t)300

// Define Max Values for DAC because 12 bits
#define MAX_DAC (float)0xFFF

// If we write MAX_DAC to PA4, the maximum possible voltage we can see is 2.95.
#define MAX_DAC_VOLTAGE 2.95

// Define MAX Value for ADC because it has 12 bits.
#define MAX_ADC (float)0xFFF

// Define Max Resistance... PBMCUSLK uses a 5k pot
#define MAX_RESISTANCE 5000

// Define Deadband voltage for opto
#define OPTO_DB_VOLTAGE 1.0

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myTIM16_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void startPotWatcher(void);
uint32_t potADCValue(void);
uint32_t mapDac(uint32_t dac);

// Your global variables...
float resistance = 0.0;
float period = 0.0;
float frequency = 0.0;

int main(int argc, char* argv[]) {
  // Initialize functions.
  myGPIOA_Init();
  myADC_Init();
  myDAC_Init();
  myTIM2_Init();
  myEXTI_Init();
  myLCD_Init();
  myTIM16_Init();

  // Start watching the values from the potentiometer and altering the frequency
  startPotWatcher();

  return 0;
}

void startPotWatcher() {
  uint32_t adcValue;
  float normalizedPotADC;
  while (1) {
    // Get the digitally converted resistance across potentiometer
    adcValue = potADCValue();

    // Ensures that any changes to the potentiometers resistance corresponds
    // in a changed frequency
    uint32_t timerControlDACValue = mapDac(adcValue);

    // Write the 12 bits to PA4
    DAC->DHR12R1 = timerControlDACValue;

    // normalize the resistance across the potentiometer
    normalizedPotADC = (((float)adcValue) / MAX_ADC);
    resistance = normalizedPotADC * MAX_RESISTANCE;
  }
}

void myGPIOA_Init() {
  // Enable clock for GPIOA peripheral
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // PA0 analog input, no pull up or down.
  GPIOA->MODER &= ~(GPIO_MODER_MODER0);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

  // PA1 input, no pull up or down.
  GPIOA->MODER &= ~GPIO_MODER_MODER1;
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

  // PA4 analog output, no pull up or down.
  GPIOA->MODER &= ~(GPIO_MODER_MODER4);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
}

void myTIM2_Init() {
  // Enable clock for TIM2 peripheral
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Configure TIM2: buffer auto-reload, count up, stop on overflow,
  // enable update events, interrupt on overflow only
  TIM2->CR1 &= ~((uint16_t)0x008C);

  // Set clock prescaler value
  TIM2->PSC = myTIM2_PRESCALER;

  // Set auto-reloaded delay
  TIM2->ARR = myTIM2_PERIOD;

  // Update timer registers
  TIM2->EGR |= ((uint16_t)0x0001);

  // Assign TIM2 interrupt priority = 0 in NVIC
  NVIC_SetPriority(TIM2_IRQn, 0);

  // Enable TIM2 interrupts in NVIC
  NVIC_EnableIRQ(TIM2_IRQn);

  // Enable update interrupt generation
  TIM2->DIER |= TIM_DIER_UIE;
}

void myEXTI_Init() {
  // Map EXTI1 line to PA1
  SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;

  // EXTI1 line interrupts: set rising-edge trigger
  EXTI->RTSR |= EXTI_RTSR_TR1;

  // Unmask interrupts from EXTI1 line
  EXTI->IMR |= EXTI_IMR_MR1;

  // Assign EXTI1 interrupt priority = 0 in NVIC
  NVIC_SetPriority(EXTI0_1_IRQn, 0);

  // Enable EXTI1 interrupts in NVIC
  NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void myTIM16_Init() {
  // Enable clock for TIM16 peripheral
  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

  // Configure TIM16: buffer auto-reload, count up, stop on overflow,
  // enable update events, interrupt on overflow only
  TIM16->CR1 = ((uint16_t)0x008C);

  // Set clock prescaler value
  TIM16->PSC = myLCD_PRESCALER;

  // Set auto-reloaded delay
  TIM16->ARR = myLCD_UPDATE_DELAY;

  // Update timer registers
  TIM16->EGR = ((uint16_t)0x0001);

  // Assign TIM16 interrupt priority = 1 in NVIC
  NVIC_SetPriority(TIM16_IRQn, 1);

  // Enable TIM16 interrupts in NVIC
  NVIC_EnableIRQ(TIM16_IRQn);

  // Enable update interrupt generation
  TIM16->DIER |= TIM_DIER_UIE;

  // Start timer
  TIM16->CR1 |= TIM_CR1_CEN;
}

void myDAC_Init() {
  // Enable clock
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;

  // Enable DAC
  DAC->CR |= DAC_CR_EN1;
}

void myADC_Init() {
  // Enable clock
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

  // Begin calibration
  ADC1->CR = ADC_CR_ADCAL;
  while(ADC1->CR == ADC_CR_ADCAL);

  // use continous conversion
  // use overrun mode
  ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);

  // Use channel 0 for PA0
  ADC1->CHSELR = ADC_CHSELR_CHSEL0;

  // Enable ADC and wait
  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

// Reads the analog resistance across potentiometer and returns the digitally
// converted value.
uint32_t potADCValue() {
  // Start ADC conversion
  ADC1->CR |= ADC_CR_ADSTART;

  // Wait for End Of Conversion flag to be set
  while (!(ADC1->ISR & ADC_ISR_EOC));

  // Reset End Of Conversion flag
  ADC1->ISR &= ~(ADC_ISR_EOC);

  // Apply the data mask to the data register
  uint32_t potADCValue = (ADC1->DR) & ADC_DR_DATA;
  return potADCValue;
}

// Ensures that any changes to the resistance across pot result
// in a change to the frequency in the optocoupler.
uint32_t mapDac(uint32_t dac) {
  // Map the DAC value to the optocouplers min value
  float normalized = dac / MAX_DAC;
  float voltageOutMultiplier = MAX_DAC_VOLTAGE - OPTO_DB_VOLTAGE;
  float voltageOut = (normalized * voltageOutMultiplier) + OPTO_DB_VOLTAGE;

  // convert the voltage value back to a DAC level
  float normVoltageOut = voltageOut / MAX_DAC_VOLTAGE;
  float outputDACValue = normVoltageOut * MAX_DAC;

  uint32_t timerDACValue = (uint32_t)outputDACValue;

  return timerDACValue;
}

// Handler for timer overflow between two consecutive rising edges
void TIM2_IRQHandler() {
  // Make sure the flag is actually set
  if ((TIM2->SR & TIM_SR_UIF) != 0) {
    trace_printf("\n*** Overflow! ***\n");

    // Clear interrupt flag
    TIM2->SR &= ~TIM_SR_UIF;

    // Restart timer
    TIM2->CR1 |= TIM_CR1_CEN;
  }
}

// Handler for writing to lcd
void TIM16_IRQHandler() {
  // Make sure the flag is actually set
  if ((TIM16->SR & TIM_SR_UIF) != 0) {
    updateValues(1, frequency);
    updateValues(2, resistance);

    // Clear interrupt flag
    TIM16->SR &= ~(TIM_SR_UIF);

    // Restart timer
    TIM16->CR1 |= TIM_CR1_CEN;
  }
}

// Handler for starting timer and computing frequency of rising edges.
void EXTI0_1_IRQHandler() {
  // Make sure the flag is actually set
  if ((EXTI->PR & EXTI_PR_PR1) != 0) {
    uint16_t dip = (TIM2->CR1 & TIM_CR1_CEN);
    // If this is the first edge
    if (!dip) {
      // Clear count register (TIM2->CNT)
      TIM2->CNT = ((uint32_t)0x00000000);
      // Start timer (TIM2->CR1)
      TIM2->CR1 |= TIM_CR1_CEN;
    }
    // Else this is the second edge
    else {
      // Stop timer (TIM2->CR1).
      TIM2->CR1 &= ~TIM_CR1_CEN;
      // Read out count register (TIM2->CNT).
      uint32_t countVal = TIM2->CNT;
      // Calculate Period
      frequency = ((float)SystemCoreClock) / countVal;
      // Calculate Frequency
      period = 1.0 / frequency;
    }

    // Clear EXTI1 interrupt pending flag (EXTI->PR).
    EXTI->PR |= EXTI_PR_PR1;
  }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
