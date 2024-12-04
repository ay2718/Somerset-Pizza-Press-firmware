/**
 * @file SSD1306.c
 * @author Austin Brown
 * @brief 760 Pizza Press SSD1306 display driver
 * @date 2022-08-05
 *
 * @copyright Copyright 2024 Boston Precision Motion LLC.
 * This project is released under the MIT License
 */

#include <SSD1306.h>

// I2C is stupid. We can only transfer upt to 255 bytes at a time, but we need header shit
// so we will transfer the data to the screen one row at a time
// luckily there are only 8 rows
// this is OK because it keeps the frame buf short anyways
// and makes less code per interrupt

uint8_t framebuf[COLUMNS+FRAME_BUF_OFFSET];   // add control commands. Only a single row is buffered at a time.
int timeout_cnt;
uint8_t _char_width;
uint8_t _font;
uint8_t _invert = 0;

//I2C_HandleTypeDef *ssd1306_i2c;
SPI_HandleTypeDef *ssd1306_spi;

void SSD1306_InitScreen(SPI_HandleTypeDef *hspi) {   // init screen

	ssd1306_spi = hspi;

	int _vccstate = 0;

	_vccstate = SSD1306_SWITCHCAPVCC;
	//GPIOA->ODR |= GPIO_ODR_8;

	// Init sequence
	if (SSD1306_command1(SSD1306_DISPLAYOFF) == HAL_OK) {                   // 0xAE
		// if first command doesn't work, give up

		SSD1306_command1(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
		SSD1306_command1(0x80);                                  // the suggested ratio 0x80

		SSD1306_command1(SSD1306_SETMULTIPLEX);                  // 0xA8
		SSD1306_command1(SSD1306_LCDHEIGHT - 1);

		SSD1306_command1(SSD1306_SETDISPLAYOFFSET);              // 0xD3
		SSD1306_command1(0x0);                                   // no offset
		SSD1306_command1(SSD1306_SETSTARTLINE | 0x0);            // line #0
		SSD1306_command1(SSD1306_CHARGEPUMP);                    // 0x8D

		if (_vccstate == SSD1306_EXTERNALVCC){
			SSD1306_command1(0x10);
		} else {
			SSD1306_command1(0x14);
		}

		SSD1306_command1(SSD1306_MEMORYMODE);                    // 0x20
		SSD1306_command1(0x00);                                  // 0x0 act like ks0108
		SSD1306_command1(SSD1306_SEGREMAP | 0x1);
		SSD1306_command1(SSD1306_COMSCANDEC);

		SSD1306_command1(SSD1306_SETCOMPINS);                    // 0xDA
		SSD1306_command1(0x12);
		SSD1306_command1(SSD1306_SETCONTRAST);                   // 0x81

		if (_vccstate == SSD1306_EXTERNALVCC){
			SSD1306_command1(0x9F);
		} else {
			SSD1306_command1(0xCF);
		}

		SSD1306_command1(SSD1306_SETPRECHARGE);                  // 0xd9

		if (_vccstate == SSD1306_EXTERNALVCC) {
			SSD1306_command1(0x22);
		} else {
			SSD1306_command1(0xF1);
		}

		SSD1306_command1(SSD1306_SETVCOMDETECT);                 // 0xDB
		SSD1306_command1(0x40);
		SSD1306_command1(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
		SSD1306_command1(SSD1306_NORMALDISPLAY);                 // 0xA6

		SSD1306_command1(SSD1306_DEACTIVATE_SCROLL);

		SSD1306_command1(SSD1306_DISPLAYON);//--turn on oled panel
		HAL_Delay(10);
		SSD1306_clearDisplay();


	}

	SSD1306_SetupFrameBuf();   // write initial conditions

	_char_width = FONT8x8_WIDTH;
	_font = FONT_8x8;
	timeout_cnt = TIMEOUT_INIT;
}

HAL_StatusTypeDef SSD1306_spiWriteDMA( uint8_t *buf, uint16_t num_bytes ) {
	return HAL_SPI_Transmit_DMA( ssd1306_spi, buf, num_bytes );

}

HAL_StatusTypeDef SSD1306_spiWrite( uint8_t *buf, uint16_t num_bytes) {
	return HAL_SPI_Transmit( ssd1306_spi, buf, num_bytes, HAL_MAX_DELAY );
//	return HAL_SPI_Transmit_DMA( ssd1306_spi, buf, num_bytes );

}

HAL_StatusTypeDef SSD1306_command1(uint8_t c) {
	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_RESET);
	return SSD1306_spiWrite(&c, 1);
}

HAL_StatusTypeDef SSD1306_sendCommand(uint8_t command, uint8_t param1, uint8_t param2) {
	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_RESET);
	//  Note continuationbit is set, so COMMAND_MODE must be
	//  repeated before each databyte that serves as parameter!

	uint8_t databytes[3];

	databytes[0] = command;
	databytes[1] = param1;
	databytes[2] = param2;
	return SSD1306_spiWrite(databytes, 3);    // Write command
}

HAL_StatusTypeDef SSD1306_sendDataByte(uint8_t data){
	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_SET);
	return SSD1306_spiWrite(&data, 1);
}

HAL_StatusTypeDef SSD1306_sendData(uint8_t* data, uint16_t len){
	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_SET);
	return SSD1306_spiWrite(data, len);
}

HAL_StatusTypeDef SSD1306_setPageAddress(uint8_t start, uint8_t end) {
	return SSD1306_sendCommand(SET_PAGE_ADDRESS, start, end);
}
HAL_StatusTypeDef SSD1306_setColumnAddress(uint8_t start, uint8_t end) {
	return SSD1306_sendCommand(SET_COLUMN_ADDRESS, start, end);
}


/*void SSD1306::writeChar(char chr) {

  const uint8_t char_index = chr - 0x20;
  if (font == 8x8) {
      for (uint8_t i = 0; i < _char_width; i++) {
           _sendData( font_8x8[char_index][i] );
      }
  } else if (font == 16x12_0) {
      for (uint8_t i = 0; i < 8; i++) {
           _sendData( font_16x12_0[char_index][i] );
      }
  } else if (font == 16x12_1) {
      for (uint8_t i = 0; i < 8; i++) {
           _sendData( font_16x12_1[char_index][i] );
      }
}*/


void SSD1306_writeString( uint8_t col, const char * text) {
	// screenbuffered version of above function
	// row is 0 thru 7, col is 0 thr 127
	uint16_t index = 0;
	uint16_t len = strlen(text);
	if (len > 32) {
		len = 32;
	}

	while ((col < MAX_COL) && (index < len)) {
		// write line, starting at given position
		// dont write if its too long
		SSD1306_writeCharToBuf(col, text[index++]);
		col+=_char_width;
	}

} 

void SSD1306_writeInt( uint8_t col, int32_t num) {
	//GPIOA->ODR |= GPIO_ODR_8;
	if(num<0)
	{
		num = -num;
		SSD1306_writeCharToBuf(col, '-');
		col+=_char_width;
	}

	char chr = 0;
	static char Representation[]= "0123456789ABCDEF";
	int base = 10;
	int mult = 10000;  // can print maximum of 5 digits

	num = num%100000;

	while (mult > num) { mult /= base; }

	int num2print = 0;

	do
	{
		num2print = (num - num%mult)/mult;
		chr = Representation[num2print];
		num -= num2print*mult;
		mult /= base;
		SSD1306_writeCharToBuf(col, chr);
		col+=_char_width;
	}while(mult != 0);

} 

void SSD1306_setFont( uint8_t font ) {

	_font = font;

	if (font == FONT_8x8) { _char_width = FONT8x8_WIDTH; }
	if (font == FONT_16x12_0) { _char_width = FONT16x12_WIDTH; }
	if (font == FONT_16x12_1) { _char_width = FONT16x12_WIDTH; }

}

void SSD1306_setInvert( uint8_t invert )
{
	_invert = invert;
}



void SSD1306_writeCharToBuf( uint8_t col, char chr ) {

	uint8_t char_index = chr - 0x20;

	uint16_t k = FRAME_BUF_OFFSET + col;

	if (col > MAX_COL) {col = MAX_COL;} //will overwrite but whatever



	if (_font == FONT_8x8) {
		for (uint8_t i = 0; i < FONT8x8_WIDTH; i++) {
			if (k >= (COLUMNS+FRAME_BUF_OFFSET)) { break; }
			framebuf[k] = font_8x8[char_index][i];
			k++;
		}
	} else if (_font == FONT_16x12_0) {
		for (uint8_t i = 0; i < FONT16x12_WIDTH; i++) {
			if (k >= (COLUMNS+FRAME_BUF_OFFSET)) { break; }
			framebuf[k] = font_16x12_0[char_index][i];
			k++;
		}
	} else if (_font == FONT_16x12_1) {
		for (uint8_t i = 0; i < FONT16x12_WIDTH; i++) {
			if (k >= (COLUMNS+FRAME_BUF_OFFSET)) { break; }
			framebuf[k] = font_16x12_1[char_index][i];
			k++;
		}
	}
}

HAL_StatusTypeDef SSD1306_writeFrameBufRow( uint8_t page ) {

	framebuf[9] = page;

	//SSD1306_i2cWrite(framebuf, COLUMNS+FRAME_BUF_OFFSET);   // takes about 3.2ms to write whole thing
//	return SSD1306_i2cWrite(framebuf, COLUMNS+FRAME_BUF_OFFSET);  // takes 18 MICROSECONDS!!! sheeeeeet
	SSD1306_setPageAddress(page, MAX_PAGE);
	SSD1306_setColumnAddress(0, MAX_COL);

	for (int i = FRAME_BUF_OFFSET; i <= MAX_COL+FRAME_BUF_OFFSET; i++)
	{
		if (_invert) { framebuf[i] = ~framebuf[i]; }
	}

	HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, GPIO_PIN_SET);
	return SSD1306_sendData(framebuf + FRAME_BUF_OFFSET, COLUMNS);
}

HAL_StatusTypeDef SSD1306_WriteRow( uint8_t page ) {
	return SSD1306_writeFrameBufRow( page );
}

void SSD1306_ClearBuf() {
	for (uint8_t j = FRAME_BUF_OFFSET; j < COLUMNS+FRAME_BUF_OFFSET; j++) {
		framebuf[j] = 0;
	}

}
// Standard version
void SSD1306_clearDisplay() {

	//setDisplayOff();
	SSD1306_setPageAddress(0, MAX_PAGE);  // all pages
	SSD1306_setColumnAddress(0, MAX_COL); // all columns

	for (uint8_t page = 0; page < PAGES; page++) {
		for (uint8_t col = 0; col < COLUMNS; col++) {
			SSD1306_sendDataByte(0x00);
		}
	}

}


void SSD1306_SetupFrameBuf() {

	for (uint8_t i = 0; i < PAGES; i++) {

		framebuf[0] = COMMAND_MODE;
		framebuf[1] = SET_COLUMN_ADDRESS;
		framebuf[2] = COMMAND_MODE;
		framebuf[3] = 0;
		framebuf[4] = COMMAND_MODE;
		framebuf[5] = MAX_COL;
		framebuf[6] = COMMAND_MODE;
		framebuf[7] = SET_PAGE_ADDRESS;
		framebuf[8] = COMMAND_MODE;
		framebuf[9] = 0;
		framebuf[10] = COMMAND_MODE;
		framebuf[11] = MAX_PAGE;
		framebuf[12] = DATA_MODE;
	}
}
