/*
*
* Graphic display driver for LCD with ST7565 controller
*
* Originally by Sebastian, see http://www.mikrocontroller.net/topic/75589
*
* Reorganized and modified by Markus Baertschi
*/
#ifndef F_CPU
#define F_CPU  18432000UL
#endif

#include <stdlib.h>
#include <avr/io.h>
#include "st7565-driver.h"
//#include "Picture_data.h" //mastercontroller image Melt
//#include "addidas_logo.h"
#include "wiselord/fonts.h"          //
#include <avr/pgmspace.h>            //           
#define STR_BUFSIZE		16
uint8_t strbuf[STR_BUFSIZE + 1] = "                ";	/* String buffer */
const uint8_t *_font;                //  добавил я
static uint8_t fp[FONT_PARAM_COUNT]; //
static uint8_t _x, _y;               //

extern uint8_t *p_b_4;		    // Указатель адреса отдельного байта массива logo
//extern uint8_t logo [1024];
//p_b_4  = &logo;

// RAM data (top to bottom; left to right)
// Pixel(x,y):          index = y/8 + x*8
//                            = (y >> 3) + (x << 3)
//                      bit   = y%8
//                            = (y & 0x07)
// Set pixel (x,y):     disp_ram[(y >> 3) + (x << 3)] |=  (1 << (y & 0x07));
// Clear pixel (x,y):   disp_ram[(y >> 3) + (x << 3)] &= ~(1 << (y & 0x07));
uint8_t disp_ram[DISP_RAM_SIZE];


// hardware functions (only used by "disp_..."-functions)

void dogm_send(unsigned char spi_data, unsigned char a0)
{
//	printf("dogm_send: %d:%02x\n",a0,spi_data);
	//a0 = 0: Command
	//a0 = 1: Display data

	//set / clear A0-Bit
	if (a0) {
//		A0_PORT |= (1<<A0_BIT);
		lcd_senddata(spi_data);
	} else {
//		A0_PORT &= ~(1<<A0_BIT);
		lcd_sendcmd(spi_data);
	}
	//Byte senden
//	spi_master_send_char(spi_data);
}
#define dogm_send_command(command)      dogm_send(command, 0)
#define dogm_send_display_data(data)    dogm_send(data, 1)


// void dogm_reset() // я отключил
// {
// 	//Reset display (50ms)
// 	RES_PORT &= ~(1<<RES_BIT); // RES\ = LOW (установить Reset)
// 	_delay_ms(50);
// 	RES_PORT |= (1<<RES_BIT); // RES\ = HIGH (снять Reset)
// }


// basic implementations (used by graphics functions)

// void disp_init() // я отключил
// {
// 	//uart_comment("[DOGM128] Initializing...");
// 
// 	//SET DATA DIRECTION REGISTER
// 	//Outputs: (DDR=1)
// 	A0_DDR |= (1<<A0_BIT); //Output: A0
// 	RES_DDR |= (1<<RES_BIT); //Output: RES
// 	//Inputs: (DDR=0)
// 	//(none)
// 
// 
// 	//RESET
// 	dogm_reset(); //(100ms)
// 
// 
// 	//Display start line
// 	dogm_send_command(0x40); // Display start line 0
// 
// 	//Bottom view
// 	dogm_send_command(0xA1); // ADC reverse
// 	dogm_send_command(0xC0); // Normal COM0~COM63
// 
// 	//Normal / Inverted
// 	dogm_send_command(0xA6); // Display normal
// 
// 	//Hardware options
// 	dogm_send_command(0xA2); // Set bias 1/9 (Duty 1/65)
// 	dogm_send_command(0x2F); // Booster, Regulator and Follower on
// 	dogm_send_command(0xF8); // Set internal Booster to 4x
// 	dogm_send_command(0x00);
// 
// 	//Contrast options
// 	dogm_send_command(0x27); // Contrast set
// 	dogm_send_command(0x81);
// 	dogm_send_command(0x16);
// 
// 	//Indicator options
// 	dogm_send_command(0xAC); // No indicator
// 	dogm_send_command(0x00);
// 
// 	//(Init done)
// 	dogm_send_command(0xAF); // Display on
// 
// 	//uart_comment("[DOGM128] Ready.");
// 
// 
// 	disp_clear();
// }



        // Big Thanks to David <david@edeca.net>
        // Reset column to the left side.  The internal memory of the
        // screen is 132*64, we need to account for this if the display
        // is flipped.
        //
        // Some screens seem to map the internal memory to the screen
        // pixels differently, the ST7565_REVERSE define allows this to
        // be controlled if necessary.
//         #ifdef ST7565_REVERSE
//         if (!glcd_flipped) {
// 	        #else
// 	        if (glcd_flipped) {
// 		        #endif
// 		        glcd_command(GLCD_CMD_COLUMN_LOWER | 4);
// 		        } else {
// 		        glcd_command(GLCD_CMD_COLUMN_LOWER

void disp_send_frame()
{
	uint8_t page;
	for (page = 0; page < 8; page++)
	{
		dogm_send_command(0xB0 + page); //Set page address to <page>
		dogm_send_command(0x10 + 0); //Set column address to 0 (4 MSBs) COLUMN_UPPER
//		dogm_send_command(0x00 + 4); //Set column address to 0 (4 LSBs) COLUMN_LOWER все остальные нормальные случаи
		dogm_send_command(0x00 + 0); //Set column address to 0 (4 LSBs) COLUMN_LOWER если активирован режим "экран перевернут" и изображение ненормально сдвигается на 4 пикселя вправо...
		
		uint8_t column;
		for (column = 0; column < 128; column++)
			dogm_send_display_data(disp_ram[page + (column << 3)]);
	}
}

void disp_clear()
{
	//clears the local RAM and send this cleared frame
	uint16_t d;
	for (d = 0; d < 1024; d++)
		disp_ram[d] = 0x00;
	disp_send_frame();
}

void disp_clear_dont_refresh() // я отключил
{
	//clears the local RAM but don't send it
	uint16_t d;
	for (d = 0; d < 1024; d++)
		disp_ram[d] = 0x00;
}


/*
void disp_frame_begin()
{
	//not relevant for this display
}
*/
//ignore "disp_frame_begin()" in source code
#define disp_frame_begin()


// void disp_frame_end() // я отключил
// {
// 	//send the display data
// 	disp_send_frame();
// }



void disp_set_pixel(uint8_t x, uint8_t y, uint8_t pixel_status)
{
	if (x < DISP_WIDTH && y < DISP_HEIGHT) {
		if (pixel_status != 0)
			disp_ram[(y >> 3) + (x << 3)] |=  (1 << (y & 0x07));
		else
			disp_ram[(y >> 3) + (x << 3)] &= ~(1 << (y & 0x07));
        }
}








// Дальше пошли функции от Владимира wiselord


void gdDrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
//	ks0108DrawPixel(x, y, color);
    disp_set_pixel(x, y, color);

	return;
}



void gdSetXY(uint8_t x, uint8_t y)
{
	_x = x;
	_y = y;

	return;
}


void gdLoadFont(const uint8_t *font, uint8_t color, uint8_t direction)
{
	uint8_t i;

	_font = font + 5;
	for (i = 0; i < FONT_PARAM_COUNT - 1; i++)
	fp[i] = pgm_read_byte(font + i);
	fp[FONT_COLOR] = color;
	fp[FONT_DIRECTION] = direction;
}

void gdWriteChar(uint8_t code)
{
	uint8_t i;
	uint8_t j;
	uint8_t k;

	uint8_t pgmData;

	uint8_t spos = code - ((code >= 128) ? fp[FONT_OFTNA] : fp[FONT_OFTA]);

	uint16_t oft = 0;	/* Current symbol offset in array*/
	uint8_t swd = 0;	/* Current symbol width */

	for (i = 0; i < spos; i++) {
		swd = pgm_read_byte(_font + i);
		oft += swd;
	}
	swd = pgm_read_byte(_font + spos);

	oft *= fp[FONT_HEIGHT];
	oft += fp[FONT_CCNT];

	for (j = 0; j < fp[FONT_HEIGHT]; j++) {
		for (i = 0; i < swd; i++) {
			pgmData = pgm_read_byte(_font + oft + (swd * j) + i);
			if (!fp[FONT_COLOR])
			pgmData = ~pgmData;
			for (k = 0; k < 8; k++) {
				switch (fp[FONT_DIRECTION]) {
					case FONT_DIR_0:
					gdDrawPixel(_x + i, _y + (8 * j + k), pgmData & (1<<k));
					break;
					case FONT_DIR_90:
					gdDrawPixel(_x + (8 * j + k), _y - i, pgmData & (1<<k));
					break;
					case FONT_DIR_180:
					gdDrawPixel(_x - i, _y - (8 * j + k), pgmData & (1<<k));
					break;
					case FONT_DIR_270:
					gdDrawPixel(_x - (8 * j + k), _y + i, pgmData & (1<<k));
					break;
				}
			}
		}
	}
	switch (fp[FONT_DIRECTION]) {
		case FONT_DIR_0:
		gdSetXY(_x + swd, _y);
		break;
		case FONT_DIR_90:
		gdSetXY(_x, _y - swd);
		break;
		case FONT_DIR_180:
		gdSetXY(_x - swd, _y);
		break;
		case FONT_DIR_270:
		gdSetXY(_x, _y + swd);
		break;
	}
//    disp_send_frame();
	return;
}

void gdWriteString(uint8_t *string)
{
	if (*string)
	gdWriteChar(*string++);
	while(*string) {
		gdWriteChar(fp[FONT_LTSPPOS]);
		gdWriteChar(*string++);
	}

	return;
}

void gdWriteIcon24(const uint8_t *icon)
{
	uint8_t i, j, k;
	uint8_t pgmData;

	if (icon) {
		for (j = 0; j < 3; j++) {
			for (i = 0; i < 24; i++) {
				pgmData = pgm_read_byte(icon + 24 * j + i);
				for (k = 0; k < 8; k++) {
					gdDrawPixel(_x + i, _y + 8 * j + k, pgmData & (1<<k));
				}
			}
		}
	}

	return;
}

void gdWriteIcon32(const uint8_t *icon)
{
	uint8_t i, j, k;
	uint8_t pgmData;

	if (icon) {
		for (j = 0; j < 4; j++) {
			for (i = 0; i < 32; i++) {
				pgmData = pgm_read_byte(icon + 32 * j + i);
				for (k = 0; k < 8; k++) {
					gdDrawPixel(_x + i, _y + 8 * j + k, pgmData & (1<<k));
				}
			}
		}
	}

	return;
}


void gdWriteIcon(const uint8_t *icon)
{
	uint8_t i, j, k;
	uint8_t pgmData;

	if (icon) {
		for (j = 0; j < 8; j++) {
			for (i = 0; i < 128; i++) {
				pgmData = pgm_read_byte(icon + 128 * j + i);
				for (k = 0; k < 8; k++) {
					gdDrawPixel(_x + i, _y + 8 * j + k, pgmData & (1<<k));
				}
			}
		}
	}

	return;
}


void gdWriteIcon_EEPROM (uint8_t *icon)
{
	uint16_t i, j, k;
	uint16_t Data;

	if (icon) 
     {
		for (j = 0; j < 8; j++) 
           {
			  for (i = 0; i < 128; i++) 
              {
//				Data = p_b_4 [j * 128 + i];
				Data = icon [j * 128 + i];
 				for (k = 0; k < 8; k++) 
                   {
				    gdDrawPixel(_x + i, _y + 8 * j + k, Data & (1<<k));
				   }
			   }
		  }
	  }

	return;
}









uint8_t *mkNumString(int16_t number, uint8_t width, uint8_t lead, uint8_t radix)
{
	uint8_t numdiv;
	uint8_t sign = lead;
	int8_t i;

	if (number < 0) {
		sign = '-';
		number = -number;
	}

	for (i = 0; i < width; i++)
	strbuf[i] = lead;
	strbuf[width] = '\0';
	i = width - 1;

	while (number > 0 || i == width - 1) {
		numdiv = number % radix;
		strbuf[i] = numdiv + 0x30;
		if (numdiv >= 10)
		strbuf[i] += 7;
		i--;
		number /= radix;
	}

	if (i >= 0)
	strbuf[i] = sign;

	return strbuf;
}


static void showParLabel(const uint8_t *parLabel)
{
	gdLoadFont(font_ks0066_ru_24, 1, FONT_DIR_0);
	gdSetXY(0, 0);
//	writeStringEeprom(parLabel);
	gdLoadFont(font_ks0066_ru_08, 1, FONT_DIR_0);

	return;
}

static void showParIcon(const uint8_t *icon)
{
	gdSetXY(104, 2);
	gdWriteIcon24(icon);

	return;
}



/*************************************************************************
Display string without auto linefeed
Input:    string to be displayed
Returns:  none
*************************************************************************/

uint8_t lcd_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{
	register char c;
	register uint8_t cnt = 0;

	while ( (c = *s++) ) {
		gdWriteChar(c);
		cnt++;
	}
	
	return cnt;

	}/* lcd_puts */


	/*************************************************************************
	Display string from program memory without auto linefeed
	Input:     string from program memory be be displayed
	Returns:   none
	*************************************************************************/

	uint8_t	lcd_puts_p(const char *progmem_s)
	/* print string from program memory on lcd (no auto linefeed) */
	{
		register uint8_t cnt = 0;
		register char c;

		while ( (c = pgm_read_byte(progmem_s++)) ) {
		gdWriteChar(c);
			cnt++;
		}

		return cnt;

		}/* lcd_puts_p */

