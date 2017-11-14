#ifndef __LCD_H
#define __LCD_H

//Initialize LCD - write the correct units
void myLCD_Init(void);

//PB3 -> SPI MOSI
//PB5 -> SPI SCK
//PB4 -> SPI LCK
void myGPIOB_Init(void);

//init SPI
void mySPI_Init(void);

//Use TIM3 as delay timer
void DELAY_Init(void);

//Format high/low bits for writing
void sendType(uint8_t type, uint8_t word);

//Write data to the LCD
void writeToLCD(uint8_t word);

//Send a string to the LCD
void sendString(char* string);

//Used to ensure the LCD does not get overwritten mid write
void setDelay(uint32_t milliseconds);

//Move the cursor
void moveCursor(uint8_t row, uint8_t col);

//Used to update resistance/frequency on LCD
void updateValues(int rowNum, float value);


#endif
