#ifndef SSD1306_SIMPLE_H
#define SSD1306_SIMPLE_H

#include "font_8x8.h"
#include "font_16x12.h"
#include <stdio.h>
#include <string.h>
#include "main.h"


#define SSD1306_128_64
//   #define SSD1306_128_32
//   #define SSD1306_96_16
/*=========================================================================*/

#define SSD1306_SLAVE_ADDR 0b01111000

#if defined SSD1306_128_64 && defined SSD1306_128_32
  #error "Only one SSD1306 display can be specified at once in SSD1306.h"
#endif
#if !defined SSD1306_128_64 && !defined SSD1306_128_32 && !defined SSD1306_96_16
  #error "At least one SSD1306 display must be specified in SSD1306.h"
#endif

#if defined SSD1306_128_64
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 64
#endif
#if defined SSD1306_128_32
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 32
#endif
#if defined SSD1306_96_16
  #define SSD1306_LCDWIDTH                  96
  #define SSD1306_LCDHEIGHT                 16
#endif

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A


#define DATA_MODE                  0x40


// Display dimensions
#define ROWS                       64
#define COLUMNS                    128
#define PAGES                      (ROWS / 8)
#define MAX_PAGE                   (PAGES - 1)
#define MAX_ROW                    (ROWS - 1)
#define MAX_COL                    (COLUMNS - 1)

// Character dimensions 8x8 font
#define CHARS                      (COLUMNS / FONT8x8_WIDTH)  // = 

// Command and Datamode 
#define COMMAND_MODE               0x80 // continuation bit is set! Just means we could stream cotinous coomands 
                                        // as is done in in triple comand mode


#define SET_COLUMN_ADDRESS         0x21 // takes two bytes, start address and end address of display data RAM
#define SET_PAGE_ADDRESS           0x22 // takes two bytes, start address and end address of display data RAM


#define FRAME_BUF_OFFSET 13

#define FONT_8x8 0
#define FONT_16x12_0 1
#define FONT_16x12_1 2

#define TIMEOUT_INIT 400


void SSD1306_InitScreen(SPI_HandleTypeDef*) ;
void SSD1306_writeString(uint8_t col, const char* text) ;
void SSD1306_writeInt( uint8_t col, int32_t num);
HAL_StatusTypeDef SSD1306_WriteRow( uint8_t page );
void SSD1306_ClearBuf();
HAL_StatusTypeDef SSD1306_IsReady();
void SSD1306_setFont( uint8_t font );
void SSD1306_setInvert( uint8_t invert );

// internal functions
void SSD1306_clearDisplay();
void SSD1306_ResetI2C();

HAL_StatusTypeDef SSD1306_spiWrite( uint8_t *buf, uint16_t num_bytes);
HAL_StatusTypeDef SSD1306_spiWriteDMA( uint8_t *buf, uint16_t num_bytes);


//HAL_StatusTypeDef SSD1306_i2cWrite( uint8_t *buf, uint16_t num_bytes);
//HAL_StatusTypeDef SSD1306_DMAi2cWrite( uint8_t *buf, uint16_t num_bytes);

HAL_StatusTypeDef SSD1306_writeFrameBufRow( uint8_t page );
void SSD1306_writeCharToBuf( uint8_t col, char chr );

HAL_StatusTypeDef SSD1306_command1(uint8_t c);
HAL_StatusTypeDef SSD1306_sendCommand(uint8_t command, uint8_t param1, uint8_t param2);
HAL_StatusTypeDef SSD1306_sendDataByte(uint8_t data);
HAL_StatusTypeDef SSD1306_sendData(uint8_t* data, uint16_t len);


HAL_StatusTypeDef SSD1306_setPageAddress(uint8_t start, uint8_t end);
HAL_StatusTypeDef SSD1306_setColumnAddress(uint8_t start, uint8_t end);
// uint32_t SSD1306_writeChar(char chr);
void SSD1306_SetupFrameBuf();


//extern I2C_HandleTypeDef *ssd1306_i2c;
extern SPI_HandleTypeDef *ssd1306_spi;
//extern uint8_t framebuf[COLUMNS+FRAME_BUF_OFFSET];   // add control commands. Only a single row is buffered at a time.
//extern int timeout_cnt;
//extern uint8_t _char_width;
//extern uint8_t _font;


//char *convert(unsigned int num, int base);



#endif
