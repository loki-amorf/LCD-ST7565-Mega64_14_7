/*
*
* Graphic display driver for LCD with ST7565 controller
*
* Originally by Sebastian, see http://www.mikrocontroller.net/topic/75589
*
* Reorganized and modified by Markus Baertschi
*/

#define g_draw_line_h(x,y,len) g_draw_line(x,y,len,0)
#define g_draw_line_v(x,y,len) g_draw_line(x,y,len,1)

void g_draw_line(uint8_t x, uint8_t y, int16_t len, uint8_t direction);
void g_draw_string(uint8_t,uint8_t, char *,uint8_t);
//uint8_t g_draw_char_clearBG(uint8_t x, uint8_t y, uint8_t c); //-
extern void lcd_init ();
