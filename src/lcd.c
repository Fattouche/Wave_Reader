
#include <stdio.h>
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"
#include "cmsis/cmsis_device.h"
#include "diag/Trace.h"
#include "assert.h"
#include "lcd.h"

#define ON 0x80
#define OFF 0x0
#define COMMAND 0x0
#define DATA 0x40

// LCD Config commands
#define CMD_MOVE_CURSOR 0x80
#define CMD_CLEAR 0x1

#define LCD_LCK_PIN GPIO_Pin_4

void myLCD_Init() {
  // Configure GPIOB to control LCD via SPI
  myGPIOB_Init();
  mySPI_Init();

  myDelay_Init();
  // Set the LCD to 4 bit mode which allows you to write 4 bits high and 4 bits
  // low
  // If the LCD is already in 4 bit mode, it simply returns the cursor home
  sendType(COMMAND, 0x2);

  // Sets the LCD to allow for 2 lines and sets the font.
  sendType(COMMAND, 0x28);

  // Turn off the cursor on display
  sendType(COMMAND, 0x0C);

  // Turn on auto shift for the cursor
  sendType(COMMAND, 0x06);

  // Make sure the LCD is clear before we start using
  sendType(COMMAND, CMD_CLEAR);

  // Let it finish clearing
  setDelay(2);

  // Write the hertz metric to the rightmost 2 blocks on row 1
  moveCursor(1, 6);
  sendString("Hz");

  // Write the ohms metric to the rightmost 2 blocks on row 2
  moveCursor(2, 6);
  sendString("Oh");
}

void mySPI_Init(void) {
  // Turn on the SPI clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  SPI_InitTypeDef SPI_InitStruct;
  SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_CRCPolynomial = 7;

  SPI_Init(SPI1, &SPI_InitStruct);
  SPI_Cmd(SPI1, ENABLE);
}

void myGPIOB_Init() {
  // Turn on the GPIOB clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  // PB3 -> SPI MOSI
  // PB5 -> SPI SCK
  // PB4 -> SPI LCK
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Let the LCK be manually controlled
  GPIO_InitStruct.GPIO_Pin = LCD_LCK_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Since our LCD is in 4 bit mode we need to send
// High(left 4) and low(right 4) separately.
void sendType(uint8_t type, uint8_t word) {
  uint8_t high, low;
  high = word >> 4;
  low = word & 0x0F;

  uint8_t highOff = OFF | type | high;
  uint8_t highON = ON | type | high;
  uint8_t LowOff = OFF | type | low;
  uint8_t lowON = ON | type | low;

  writeToLCD(highOff);
  writeToLCD(highON);
  writeToLCD(highOff);

  writeToLCD(LowOff);
  writeToLCD(lowON);
  writeToLCD(LowOff);
  setDelay(2);
}

void sendString(char* string) {
  // Sends the string to the LCD
  const char* imutStr = string;
  while (*imutStr) {
    sendType(DATA, (uint8_t)(*imutStr++));
  }
}

void writeToLCD(uint8_t word) {
  // LCK=0, hide register values from LCD
  GPIOB->BRR = LCD_LCK_PIN;

  // Wait till SPI is ready
  while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != SET) &&
         (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET));

  SPI_SendData8(SPI1, word);

  // Send the word to the LCD
  SPI_SendData8(SPI1, word);

  // Wait till SPI finished writing
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET);

  // LCK=1, expose register values from LCD
  GPIOB->BSRR = LCD_LCK_PIN;
}

void moveCursor(uint8_t row, uint8_t col) {
  // The LCD has a row offset for row1 and row2
  uint8_t rowOffset = 0x0;
  if (row == 2) {
    rowOffset = 0x40;
  }
  // First bit = cursor move
  // Second bit = (row 1:bit=0, row2:bit=1)
  // Rest bits = column to go to
  uint8_t moveCursorCommand = CMD_MOVE_CURSOR | rowOffset | col;
  sendType(COMMAND, moveCursorCommand);
}

void setDelay(uint32_t milliseconds) {
  // Clear timer
  TIM3->CNT |= 0x0;

  // Set timeout
  TIM3->ARR = milliseconds;

  // Update timer registers
  TIM3->EGR |= 0x0001;

  // Start timer
  TIM3->CR1 |= TIM_CR1_CEN;

  // Wait till timer overflow
  while (!(TIM3->SR & TIM_SR_UIF));

  // Stop timer
  TIM3->CR1 &= ~(TIM_CR1_CEN);

  // Reset interrupt
  TIM3->SR &= ~(TIM_SR_UIF);
}

void myDelay_Init() {
  // Enable timer clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // AutoReload+interupt on overflow
  TIM3->CR1 = ((uint16_t)0x8C);

  // 1KHZ clock prescaler
  TIM3->PSC = 47999;

  // Delay period
  TIM3->ARR = 100;

  // Update timer registers.
  TIM3->EGR |= 0x0001;
}

void updateValues(int row, float val) {
  // Format the string to write to the LCD to ensure that
  // the correct spacing and decimal places are used
  char* prefix;
  char* floatFormatter;
  if (val > 1000) {
    prefix = "k";
    floatFormatter = "%.3f";
    val = val / 1000;
  } else if (val < 10) {
    prefix = "";
    floatFormatter = "%.4f";
  } else {
    prefix = "";
    floatFormatter = "%.2f";
  }

  char stringToSend[5];
  sprintf(stringToSend, floatFormatter, val);
  moveCursor(row, 0);
  sendString(stringToSend);
  sendString(prefix);
}
