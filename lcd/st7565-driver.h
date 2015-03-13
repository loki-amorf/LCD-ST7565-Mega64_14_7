/*
*
* Graphic display driver for LCD with ST7565 controller
*
* Originally by Sebastian, see http://www.mikrocontroller.net/topic/75589
*
* Reorganized and modified by Markus Baertschi
*/

#ifndef ST7565_DRIVER_H
#define ST7565_DRIVER_H

// Display properties (for graphics functions)
#define DISP_WIDTH   128L
#define DISP_HEIGHT  64L    // 65 lines broken !
#define DISP_DEPTH   1		// monochrome
//#define DISP_RAM_SIZE 1152	// 9 x 128
#define DISP_RAM_SIZE 1024	// 8 x 128

// Hardware pin definitions

#define CS_DDR DDRB
#define CS_PIN PINB
#define CS_PORT PORTB
#define CS_BIT 0

#define A0_DDR DDRB
#define A0_PIN PINB
#define A0_PORT PORTB
#define A0_BIT 4

#define RES_DDR DDRE
#define RES_PIN PINE
#define RES_PORT PORTE
#define RES_BIT 2

// #define SID_DDR DDRB  // закомментировано потому что реализация СПИ аппаратная
// #define SID_PIN PINB
// #define SID_PORT PORTB
// #define SID_BIT 2
// 
// #define SCLK_DDR DDRB
// #define SCLK_PIN PINB
// #define SCLK_PORT PORTB
// #define SCLK_BIT 1


extern void lcd_sendcmd (unsigned char);
extern void lcd_senddata (unsigned char);
//void disp_init(); //-
void disp_send_frame();//+
void disp_clear();//+
void disp_clear_dont_refresh();
void disp_set_pixel(uint8_t,uint8_t,uint8_t);//+
void dogm_send(unsigned char spi_data, unsigned char a0);
//void dogm_reset(); //-
//void disp_frame_end(); //-



// ниже добавил я..
#define FONT_PARAM_COUNT			7
enum {
	FONT_HEIGHT,
	FONT_LTSPPOS,
	FONT_CCNT,
	FONT_OFTA,
	FONT_OFTNA,
	FONT_COLOR,
	FONT_DIRECTION
};

enum {
	FONT_DIR_0,
	FONT_DIR_90,
	FONT_DIR_180,
	FONT_DIR_270
};



// Дальше пошли функции от Владимира

//void gdSetBrightness(uint8_t br);

void gdDrawPixel(uint8_t x, uint8_t y, uint8_t color);
//void gdDrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
//void gdDrawCircle(uint8_t x0, uint8_t y0, int16_t radius, uint8_t color);

void gdSetXY(uint8_t x, uint8_t y);
void gdLoadFont(const uint8_t *font, uint8_t color, uint8_t direction);

void gdWriteChar(uint8_t code);
void gdWriteString(uint8_t *string);

void gdWriteIcon24(const uint8_t *icon);
void gdWriteIcon32(const uint8_t *icon);
void gdWriteIcon(const uint8_t *icon);
//void gdWriteIcon_EEPROM(uint8_t *icon);
void LoadBitmap_1(const unsigned char *bitmap);

uint8_t *mkNumString(int16_t number, uint8_t width, uint8_t lead, uint8_t radix);

uint8_t lcd_puts(const char *s);

uint8_t	lcd_puts_p(const char *progmem_s);




#endif /* ST7565_DRIVER_H */