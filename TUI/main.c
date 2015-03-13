/**
 * \p ������ ��� ������������ ��������� ������������ ����������� ������� S-LCD TUI
 * (Small-LCD Text User Interface), �.�. �������� ���������������� ��������� ��� 
 * ��������� �������������������� ��������.
 * \p ��� ��������� S-LCD TUI ������������ ��������� ������:
 * \p LCD.H - ��������� 2-�������� ���
 * \p MMENU.H - ���������� ������� ����
 * \p BUTTONS.H - ������ � �������� ����������
 * \p EVENTS.H - ��������� �������
 * \p SEDIT.H - ������� ���������
 * 
 * \author: ARV
 * 
 * � ������� ����������� ��������� ����:
 *    MENU MODE
 *       ONE LINE
 *       MULTI LINE
 *    EDIT
 *       UINT EDIT *
 *       INT EDIT *
 *       SCALE HBAR *
 *       SCALE VBAR *
 *    LEDS
 *       LED1 *
 *       LED2 *
 *       LED3 *
 *       LED4 *
 *    TEST
 *
 * ������ ��������� � ��������������� ������� ��������� �������� ���������.
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sedit.h"
#include "buttons.h"
#include "mmenu.h"
#include "lcd.h"

#define menu_one 1	// ��� ������ ������ ���� "������������ ����"
#define menu_multi 2	// ��� ������ ������ ���� "������������� ����"
#define menu_test 3	// ��� ������ ������ ���� "����"

PROGMEM char mmm1[] = "ONE LINE";	// ������ ������ ���� "������������ ����"
PROGMEM char mmm2[] = "MULTI LINE";	// ������ ������ ���� "������������� ����"

PROGMEM char mm1[] = "MENU MODE";	// ������ ������ ���� "����� ����"
PROGMEM char mm2[] = "EDIT";		// ������ ������ ���� "��������������"
PROGMEM char mm3[] = "LEDS";		// ������ ������ ���� "����������"
PROGMEM char mm4[] = "TEST";		// ������ ������ ���� "����"

static	char	tr[16];		// ��������������� ������-������

static uint16_t number1 = 100;	// ���������� ��� ������������ ��������� ����������� �����
static int16_t  number2 = -12;	// ���������� ��� ������������ ��������� ����� �� ������
static uint8_t	number3 = 10;	// ���������� ��� ������������ �������������� ����������� ������
static uint8_t	number4 = 20;	// ���������� ��� ������������ �������������� ������� �� �����

// ����������� ������� ��� �������������� ��������� ����� � ���� (��� �� ������ �������� ����������)

// ����� � ���� �������� ������
static pchar ed_scal1(void* data, uint8_t v, uint8_t ch);
// ����� � ���� ��������
static pchar ed_scal2(void* data, uint8_t v, uint8_t ch);
// �������������� ������������ �����
static pchar ed_uint(void* data, uint8_t v, uint8_t ch);
// �������������� ����� �� ������
static pchar ed_int(void* data, uint8_t v, uint8_t ch);
// �������������� ������� �� ����� �
static pchar port_val(void* data, uint8_t v, uint8_t ch);

// ������ ���� ��������� ������ ����
PROGMEM t_menu m_mode[] = {
	item_sim(mmm1,menu_one),
	item_sim(mmm2,menu_multi)
};

// ������ ���� ��������������
PROGMEM t_menu m_edit[] = {
	item_opt(ed_uint, &number1, 0),
	item_opt(ed_int, &number2, 0),
	item_opt(ed_scal1, &number3, 0),
	item_opt(ed_scal2, &number4, 0)
};

// ������ ���� ���������� ������������
PROGMEM t_menu m_flags[] = {
	item_opt(port_val, &PORTC, 0x01),
	item_opt(port_val, &PORTC, 0x02),
	item_opt(port_val, &PORTC, 0x04),
	item_opt(port_val, &PORTC, 0x08)
};

// ������ ��������� ����
PROGMEM t_menu m_menu[] = {
	item_sub(mm1,m_mode),	// ������� ������� ����������� ����
	item_sub(mm2,m_edit),	// ������� ����������
	item_sub(mm3,m_flags),	// ������� ������� 
	item_sim(mm4,menu_test)			// ������� ����� - ��������� ��������
};

// �������, ��������������� ����� ��������� ���������
static void test(void){
	init_v_scale_sym();			// �������������� ��������������
	lcd_clrscr();				// ������� �������
	for(uint8_t i = 0; i < 50; i++){	// ������������ ������� �� 50 ��������
		for(uint8_t x=0; x<16;x++){	// � ������ ������� �������
			lcd_gotoxy(x,0);		// � ������ ������
			v_bar(rand(),255);	// ������� ��������� ������ �������
			lcd_gotoxy(x,1);		// � �� ������ ������
			v_bar(rand(),255);	// �� �� �����
		}
		_delay_ms(100);			// ������ ���� ������������ ������ 0,1 ���
	}
}

// ������� �������
int main(void){
	char s[16];
	EVENT ev = EV_NONE;
	uint8_t mode=0;
	uint8_t menu_r;
	PORTB = 0xFF;				// ���� � �������� - ���. ��������
	DDRC = 0x0F;				// ���� �� ������������
	PORTC = DDRC;				

	lcd_init(LCD_DISP_ON);			// ������������� LCD
	while(1){
		// ������� ���������
		lcd_clrscr();
		// ������� "��������"
		lcd_puts_P("TUI DEMO\nPRESS * TO MENU");

		// ���� ����������� ������� - ������� ������ ��������� ����
		do {
			ev = get_event();		// �������� �������
		}while(ev == EV_NONE);
		
		switch(ev){
		case EV_ENTER:
			// ������������ ���� � ����������� �� �������� ������
			menu_r = (mode==0) ? do_menu1(m_menu, sizeof(m_menu)/sizeof(t_menu)) : do_menu2(m_menu, sizeof(m_menu)/sizeof(t_menu));

			// ��������� �������� � ����������� �� ����������� ���� ����
			switch(menu_r){
			case menu_multi:
				mode = 1;		// �������� ������������� ����� ����
				break;
			case menu_one:
				mode = 0;		// �������� ������������ ����� ����
				break;
			case menu_test:
				test();		// ��������� ���������������� �������
				break;
			}
			break;
		}
	}
}

// �������������� ������� �� ����� � - ���������� ������������
static pchar port_val(void* data, uint8_t v, uint8_t ch){
	pchar	ts;
		
	if(ch){ // �������� ch �� ������ ���� ���������, ��� ���� �������� ���������
		// ��� ���� ��������� data ������� ���������� �� ����
		*((uint8_t*)data) ^= v;
	}
		
	// ������� ����� � ����������� �� ����, ����� ��������� ��������������
	switch(v){
	case 1: ts = PSTR("LED 1"); break;
	case 2: ts = PSTR("LED 2"); break;
	case 4: ts = PSTR("LED 3"); break;
	case 8: ts = PSTR("LED 4"); break;
	}
	// ������� ������ ��� ��������
	strcpy_P(tr,ts);				// �������� ������ ����� ������	
	if(*((uint8_t*)data) & v)		// ��������� ��������� ����� �
							// ���������� ��������������� �����
		strcat_P(tr,PSTR(" ON"));	
	else
		strcat_P(tr,PSTR(" OFF"));
	// ���������� ���������� ������
	return tr;	
}

// �������������� ������������ ������ �����
static pchar ed_uint(void* data, uint8_t v, uint8_t ch){
	char ts[6];
	if(ch){ // ���� ����� �������� - ������ �������� ������ ��������
		lcd_clrscr();
		*(uint16_t*)data = edit_uint(0,PSTR("Unsigned: "),*(uint16_t*)data,50,20000);
	}
	// �������������� ������, ������� ����� � ����
	strcpy_P(tr,PSTR("UINT EDIT "));
	ltoa((uint16_t)(*(uint16_t*)data),ts,10);
	strcat(tr,ts);
	return tr;
}

// �������������� ������ �� ������ - ���������� �����������
static pchar ed_int(void* data, uint8_t v, uint8_t ch){
	char ts[7];
	if(ch){
		lcd_clrscr();
		*(int16_t*)data = edit_sint(0,PSTR("Signed: "),*(int16_t*)data,-1500,20000);
	}
	strcpy_P(tr,PSTR("INT EDIT "));
	itoa(*(uint16_t*)data,ts,10);
	strcat(tr,ts);
	return tr;
}

// ����� � ���� ��������
static pchar ed_scal2(void* data, uint8_t v, uint8_t ch){
	uint8_t tmp = *(uint8_t*)data;
	uint8_t old = tmp;
	uint8_t event;
	char ts[5];
	
	if(ch){ // �������������� ����������� ����� � ���� �������
		init_hbar_scale_sym();		// �������������� ��������������
		// �����������
		lcd_clrscr();
		lcd_gotoxy(0,0);
		lcd_puts_p(PSTR("SCALE 2"));
		do{
			se_percent(0,1,tmp);	// ������ ������ ������� ��������� �����
			// ������� �������
			do{
				event = get_event();
			} while(!event);
			// ��������� ��������� ������������ �������
			switch(event){
			case EV_ESCAPE:
				tmp = old;		// ��� ������ ��������������� �������� ��������
			case EV_ENTER:
				*(uint8_t*)data = tmp; // �� ENTER ��������� ������� ������� ��������
				goto exit;
			// ������ ������-������ ������ ������ �������� � �������� ��������
			case EV_NEXT:
				if(++tmp > 100) tmp = 100;
				break;
			case EV_PREV:
				if(tmp > 0) tmp--;
			}
		}while(1);
	} 
	// ������� ������, ������� ����� � ����
exit:
	strcpy_P(tr,PSTR("SCALE VBAR "));
	itoa(tmp,ts,10);
	strcat(tr,ts);
	return tr;
}


// ����� � ���� �������� ������ - ���������� ����������
static pchar ed_scal1(void* data, uint8_t v, uint8_t ch){
	uint8_t tmp = *(uint8_t*)data;
	uint8_t old = tmp;
	uint8_t event;
	char ts[5];
	
	if(ch){
		// �����������
		lcd_init(LCD_DISP_OFF);
		init_h_scale_sym();
		lcd_init(LCD_DISP_ON);
		lcd_clrscr();
		lcd_gotoxy(0,0);
		lcd_puts_p(PSTR("SCALE 1"));
		do{
			se_percent(0,1,tmp);
			do{
				event = get_event();
			} while(!event);
			switch(event){
			case EV_ESCAPE:
				tmp = old;
			case EV_ENTER:
				*(uint8_t*)data = tmp;
				goto exit;
			case EV_NEXT:
				if(++tmp > 100) tmp = 100;
				break;
			case EV_PREV:
				if(tmp > 0) tmp--;
			}
		}while(1);
	} 
exit:
	strcpy_P(tr,PSTR("SCALE HBAR "));
	itoa(tmp,ts,10);
	strcat(tr,ts);
	return tr;
}

