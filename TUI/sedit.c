/** \file sedit.c Simple Editors
 * ������ � ������� ������� ��� �������������� ����������.
 * ������������ ��� ������� ��������� � ������� ���������� ���� mmenu.h  
 * 
 * \author ARV
 */

#include <avr/pgmspace.h>
#include <stdlib.h>
#include "events.h"
#include "sedit.h"
#include "../lcd/st7565-driver.h"
#include "../lcd/wiselord/fonts.h"          //

#define FIRST_USER_SYMBOL 1

/// ������� ������ �������, ������������� ��� "��������" ���������� �������������� �����
PROGMEM const uint8_t h_symb[] = {
		0b00000000, // 0
		0b00000000,
		0b00000000,
		0b00011111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000
};

// ��������������� ����������
static uint8_t offset_first;

/** ������������� ����������������� ��������������� ��� ����������� ���� � ���� ������������� ��������
 * 
 */
// void init_v_scale_sym(void){
// //    lcd_command(0x48);								// ������� ������ � ��� ���������������
// 	for(uint8_t i=0; i<8; i++){						// ������� 8 ��������
// 		for(uint8_t y=0; y<8; y++){					// ��������� �� 8 �������������� �����
// 			// ������ ������ �������� �� ����������� ����������� ������ ����� �����
// 			if(y >= (7-i))							
// //				lcd_data(0xFF);
// 			else
// 				// ������� 0 �� �������� 0b00100 ������� ������ ������� ��� "������" �����
// 				lcd_data(0);
// 		}
// 	}
// }

/** ����� �����-�������� ������� � ���� ����������.
 * ����� ���������� � ������� ������ ���� ������������������ ���������������� ��������������
 * ��� ������ ������� init_v_scale_sym() 
 * @param val ��������� �� ����� ��������
 * @param base ��������, ��������������� ��������� �����
 * \note ������� ������� ������� � ������� ������� "�������".
 */
void v_bar(uint8_t val, uint8_t base){
	if(val > base) val = base;						// �� ��������� ���������� �����
	gdWriteChar((val * 8)/base + FIRST_USER_SYMBOL);	// ��������� � ������� ������ ������
}

/** ����� �����-�������� ������� � ��� ����������.
 * ����� ���������� � ������� ������ ���� ������������������ ���������������� ��������������
 * ��� ������ ������� init_v_scale_sym() 
 * @param pos ������� �������� �� �����������
 * @param val ��������� ��������
 * @param base �������� �����
 */
void v2_bar(uint8_t pos, uint8_t val, uint8_t base){
	uint8_t tmp = val * 15 / base + FIRST_USER_SYMBOL;// ��������� ������������� �������� �����
	if(tmp > 15) tmp = 15; 							// �� ��������� ��������� ��������
	if(tmp > 8){
		// ���� ����� ����, ��� ���� ����������
		gdSetXY(pos,1);				
		gdWriteChar(8);
		gdSetXY(pos,0);
		tmp -= 7;
	} else {
		// ���� ����� ����, ��� ���� ����������
		gdSetXY(pos,0);
		gdWriteChar(' ');
		gdSetXY(pos,1);
	}
	gdWriteChar(tmp);
    disp_send_frame();		
}

/** ������������� ����������������� ��������������� ��� ����������� ���� � ���� ��������������� ��������
 * 
 */
void init_h_scale_sym(void){
	uint8_t mask = 0b00100000;
//    lcd_command(0x48);									// ������� ������ � ��������������
    for(uint8_t i=0; i<6; i++){							// ������� 6 ��������
    	for(uint8_t y=0; y<8; y++){						// ��������� �� 8 �����
//    		lcd_data(pgm_read_byte(&h_symb[y]) | mask);	// ������� ������ ��������� �� �������
    	}
    	mask |= (mask >> 1);							// � ��������� ������������ �������
														// ����������� ����� ������� ������
    }
    offset_first = 0;									// ������ �������� ��������������� ����������
}

/** ������������� ����������������� ��������������� ��� ����������� ���� � ���� ��������������� ��������.
 * ������� �������������� �������������� ��� ������ ����� ���� "��������� ���������", �.�. ���
 * ����������� ������������� ������ � ������ ����� �����. 
 */
void init_hbar_scale_sym(void){
	uint8_t mask = 0b00100000;
//    lcd_command(0x48);
    for(uint8_t i=0; i<6; i++){
    	mask >>= 1; // ������� �� init_h_scale_sym() ������ � ���� ������ 
    	for(uint8_t y=0; y<8; y++){
//    		lcd_data(pgm_read_byte(&h_symb[y]) | mask);
    	}
    }
    offset_first = 5; // � � ���� ;-)
}

/** ������� ������� ������ ����������������� ��������������� �� ������ �� flash.
 * 
 * @param symbol_count ���������� ��������
 * @param symbols ������ ������� ������� ������� ��������������� �� flash 
 */
void se_init_spec_symbols(uint8_t symbol_count, uint8_t *symbols){
//    lcd_command(0x48);
    for(;symbol_count; symbol_count--){
    	for(uint8_t i=0; i<8; i++){
    	    lcd_data(pgm_read_byte(symbols));
    	    symbols++;
    	}
    }
}

/** ����������� �������� �������� � ���� �������������� �����.
 * ������ ����� � ������� ������� ���, ����� ����� - 10 ���������.
 * @param percent ��������� �����
 */
void se_scale_percent(uint8_t percent){
	uint8_t pos = 0;
	uint8_t t;
	if(percent > 100) percent = 100;				// ������ 100% ���� �� �����
	while(percent > 10){							// ������ ������� ����� �����
		gdWriteChar(FIRST_USER_SYMBOL+5);
		percent -= 10;
		pos++;
	}
	if(offset_first){
		if(!percent) percent++;
		t = FIRST_USER_SYMBOL + ((percent-1) >> 1);
	} else {
		t = FIRST_USER_SYMBOL + (percent >> 1);
	}
	gdWriteChar(t);	// ������ ��������� �����
	pos++;
	while(pos < 10){								// ������ ������ ����� �����
		gdWriteChar(FIRST_USER_SYMBOL+offset_first);
		pos++;
	}
}

/** ����������� ��������� �������� � ��������� � �� �������������� �����
 * 
 * @param x ������� ������ ������ �� �����������
 * @param y ����� ������ ���
 * @param percent ��������� ��������
 */
void se_percent(uint8_t x, uint8_t y, uint8_t percent){
	uint8_t s[4];
	
	itoa(percent,s,10);								// ����������� ����� � ������
	gdSetXY(x,y);								// ������������� ������
	lcd_puts_p("    ");								// ������� 4 ����������
	gdSetXY(x,y);								// ����� �������������
	lcd_puts(s);									// ������� �����
	gdWriteChar('%');									// ������� ������ ��������
	gdSetXY(x+4,y);								// ������������� �� ������ �����
	se_scale_percent(percent);						// � ������� �����
	disp_send_frame();	
}

/** �������������� ������������ ������ � ������ �������� �����.
 * ��������� ������������� ����� ����������� �� ������� ���������� ��������
 * ����������� ������ �����.
 * @param s ��������� �� �������� ������ (������ ������ ���� �����������!)
 * @param val ������������� �����
 * @param digits ���������� ������� � ������
 */
static
void ui2s(char* s, uint16_t val, uint8_t digits){
	for(uint8_t i = 1; i <= digits; i++){
		s[digits-i] = '0' + val % 10;
		val /= 10;
	}
	s[digits] = 0;
}

/** �������������� ������ ����� �� ������ � ������ �������� �����.
 * ��������� ������������� ����� ����������� �� ������� ���������� ��������
 * ����������� ������ �����.
 * @param s ��������� �� �������� ������ (������ ������ ���� �����������!)
 * @param val ������������� �����
 * @param digits ���������� ������� � ������
 */
static
void si2s(char* s, int16_t val, uint8_t digits){
	uint8_t sign = val < 0;
	if(sign) val *= -1;
	for(uint8_t i = 1; i < digits; i++){
		s[digits-i] = '0' + val % 10;
		val /= 10;
	}
	if(sign)
		s[0] = '-';
	else
		s[0] = ' ';
	s[digits] = 0;
}

/** ������� ����� ������, ����������� ��� ������������� ����� � ���� ������.
 *
 * @param v ������� �����
 * @return ����� ����������� ������
 */
static
uint8_t number_digit(int32_t v){
	uint8_t result=0;
	uint32_t tv = v < 0 ? -v : v;

	for(; tv; tv /= 10) result++;
	if(!result || (v < 0)) result++;
	return result;
}

/// ������� �� ����
#define MAX(x,y) ((x) < (y) ? (y) : (x))

/** ����������� �������������� ����� �� ������.
 * ������� ��������� ������� ��������� � �������� �������� ���������� � ���������� ������.
 * @param y ����� ������ ������� ��� ��������������
 * @param msg ��������� �� ������ �� FLASH � ����������
 * @param val �������� ����������
 * @param min ���������� ���������� �������� ����������
 * @param max ����������� ���������� �������� ����������
 * @return ����� ��������
 */

//int16_t edit_sint(uint8_t y, prog_char *msg, int16_t val, int16_t min, int16_t max){
int16_t edit_sint(uint8_t y, const char *msg, int16_t val, int16_t min, int16_t max){	
	int16_t res = val;
	uint8_t edit_pos=0,						// ����� ������������� �������
			pos=0,							// ������� ����� � ������ �������
			digits;							// ���������� ������� � �����
	uint8_t ev = EV_NONE;					// �������
	char vstr[7];							// ��������������� ������

	//��������� ����� ������ � �����
	digits = MAX(number_digit(max), number_digit(min));
	gdSetXY(0,y);						// ������� ��������� � ������ ������
	lcd_puts_p(msg);
	if(msg)									// ���� ���� ���������, ��
		while(pgm_read_byte(&msg[pos])) pos++;// ��������� ������� �����

	// ����������� �������������� �����
	do{
		si2s(vstr,res,digits);				// �������� ���������� ������������� �����
//		lcd_command(LCD_DISP_ON);
		gdSetXY(pos,y);					// ������������� � ����� ������ �����
		lcd_puts(vstr);						// ������� ������������� �����
//		lcd_command(LCD_DISP_ON_CURSOR_BLINK);
        gdLoadFont(font_ks0066_ru_08, 0, FONT_DIR_0);
		gdSetXY(pos+edit_pos,y);			// ������ ������ �� ������������� �������
		do {
			ev = get_event();
		}while(ev == EV_NONE);				// �������� �������
		switch(ev){
		case EV_ENTER: // ������ ��������� �������
			if((min < 0) && (edit_pos == 0)){
				// ���� ����� ����� ���� � ������� � ������������� ������� ��� ��� ��������
				if(vstr[edit_pos] == ' ')	// ������ ���� �� ���������������
					vstr[edit_pos] = '-';
				else
					vstr[edit_pos] = ' ';
			} else {
				// ���� ����� ������ ������������� ��� ����������� �� ����
				vstr[edit_pos]++;				// ���������� ������
				res = atoi(vstr);					// ������� ������������� ��������� � �����
				if((vstr[edit_pos] > '9') 
					|| (res > max)
					|| (res < min)
					) 
					vstr[edit_pos] = '0';
			}
			break;
		case EV_NEXT: // ������ �������� � ��������� �������
			if(++edit_pos >= digits)		// ���������� �������, ���� �� ������ �� �������
				edit_pos = 0;				// � ��� ������ - �� ����� � ������ ���������
			break;
		case EV_PREV: // ������ �������� � ���������� �������
			if(--edit_pos > digits)			// ��������� �������, ���� �� ������������ � 255
				edit_pos = digits-1;		// ��� ������������ - ��������� � ��������� �������
			break;
		}
		res = atoi(vstr);					// ������� ������������� ��������� � �����
		disp_send_frame();			
	} while(ev != EV_ESCAPE); // ���� ��������� �� ������� ������ �� ���������
	gdLoadFont(font_ks0066_ru_08, 1, FONT_DIR_0);
//    disp_send_frame();		
//	lcd_command(LCD_DISP_ON);
	return res;						// ���������� ������� ��������
}

/** ����������� �������������� ����� ��� �����.
 * ������� ��������� ������� ��������� � �������� �������� ���������� � ���������� ������.
 * @param y ����� ������ ������� ��� ��������������
 * @param msg ��������� �� ������ �� FLASH � ����������
 * @param val �������� ����������
 * @param min ���������� ���������� �������� ����������
 * @param max ����������� ���������� �������� ����������
 * @return ����� ��������
 */

//uint16_t edit_uint(uint8_t y, prog_char *msg, uint16_t val, uint16_t min, uint16_t max){
uint16_t edit_uint(uint8_t y, const char *msg, uint16_t val, uint16_t min, uint16_t max){	
	uint16_t res = val;
	uint8_t edit_pos=0,						// ����� ������������� �������
			pos=0,							// ������� ����� � ������ �������
			digits;							// ���������� ������� � �����
	uint8_t ev = EV_NONE;					// �������
	char vstr[6];							// ��������������� ������

	digits = number_digit(max);				//��������� ����� ������ � �����
	gdSetXY(0,y);						// ������� ��������� � ������ ������
	lcd_puts_p(msg);
	if(msg)									// ���� ���� ���������, ��
		while(pgm_read_byte(&msg[pos])) pos++;	// ��������� ������� �����

	// ����������� �������������� �����
	do{

		if(res < min) res = min;

		ui2s(vstr,res,digits);				// �������� ���������� ������������� �����
//		lcd_command(LCD_DISP_ON);
		gdSetXY(pos,y);					// ������������� � ����� ������ �����
		lcd_puts(vstr);						// ������� ������������� �����
//		lcd_command(LCD_DISP_ON_CURSOR_BLINK);
        gdLoadFont(font_ks0066_ru_08, 0, FONT_DIR_0);
		gdSetXY(pos+edit_pos,y);			// ������ ������ �� ������������� �������
		do {
			ev = get_event();
		}while(ev == EV_NONE);				// �������� �������
			
		switch(ev){
		case EV_ENTER: // ������ ��������� �������
			vstr[edit_pos]++;				// ���������� ������
			res = atol(vstr);					// ������� ������������� ��������� � �����
			if((vstr[edit_pos] > '9') || (res > max)) 
				vstr[edit_pos] = '0';
			break;
		case EV_NEXT: // ������ �������� � ��������� �������
			if(++edit_pos >= digits)		// ���������� �������, ���� �� ������ �� �������
				edit_pos = 0;				// � ��� ������ - �� ����� � ������ ���������
			break;
		case EV_PREV: // ������ �������� � ���������� �������
			if(--edit_pos > digits)			// ��������� �������, ���� �� ������������ � 255
				edit_pos = digits-1;		// ��� ������������ - ��������� � ��������� �������
			break;
		}
		res = atol(vstr);					// ������� ������������� ��������� � �����
		disp_send_frame();	//
	} while(ev != EV_ESCAPE); // ���� ��������� �� ������� ������ �� ���������
	 gdLoadFont(font_ks0066_ru_08, 1, FONT_DIR_0);
//	lcd_command(LCD_DISP_ON);
//     disp_send_frame();	//
	return res;						// ���������� ������� ��������
}
