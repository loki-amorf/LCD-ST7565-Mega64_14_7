/**
 * \p Проект для демонстрации некоторых возможностей программных модулей S-LCD TUI
 * (Small-LCD Text User Interface), т.е. тектовый пользовательский интерфейс для 
 * небольших жидкокристаллических дисплеев.
 * \p Для поддержки S-LCD TUI используются следующие модули:
 * \p LCD.H - поддержка 2-строчных ЖКИ
 * \p MMENU.H - реализация системы меню
 * \p BUTTONS.H - работа с кнопками управления
 * \p EVENTS.H - обработка событий
 * \p SEDIT.H - простые редакторы
 * 
 * \author: ARV
 * 
 * В проекте реализуется следующее меню:
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
 * Вместо звездочки в соответствующих пунктах выводится значение параметра.
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

#define menu_one 1	// код выбора пункта меню "однострочное меню"
#define menu_multi 2	// код выбора пункта меню "многострочное меню"
#define menu_test 3	// код выбора пункта меню "тест"

PROGMEM char mmm1[] = "ONE LINE";	// строка пункта меню "однострочное меню"
PROGMEM char mmm2[] = "MULTI LINE";	// строка пункта меню "многострочное меню"

PROGMEM char mm1[] = "MENU MODE";	// строка пункта меню "режим меню"
PROGMEM char mm2[] = "EDIT";		// строка пункта меню "редактирование"
PROGMEM char mm3[] = "LEDS";		// строка пункта меню "светодиоды"
PROGMEM char mm4[] = "TEST";		// строка пункта меню "тест"

static	char	tr[16];		// вспомогательный массив-строка

static uint16_t number1 = 100;	// переменная для демонстрации редактора беззнаковых чисел
static int16_t  number2 = -12;	// переменная для демонстрации редактора чисел со знаком
static uint8_t	number3 = 10;	// переменная для демонстрации редактирования заполненной шкалой
static uint8_t	number4 = 20;	// переменная для демонстрации редактирования штрихом на шкале

// определение функций для автоматической генерации строк в меню (они же меняют значения переменных)

// шкала в виде сплошной полосы
static pchar ed_scal1(void* data, uint8_t v, uint8_t ch);
// шкала в виде верньера
static pchar ed_scal2(void* data, uint8_t v, uint8_t ch);
// редактирование беззнакового числа
static pchar ed_uint(void* data, uint8_t v, uint8_t ch);
// редактирование числа со знаком
static pchar ed_int(void* data, uint8_t v, uint8_t ch);
// редкатирование уровней на порту С
static pchar port_val(void* data, uint8_t v, uint8_t ch);

// массив меню изменения режима меню
PROGMEM t_menu m_mode[] = {
	item_sim(mmm1,menu_one),
	item_sim(mmm2,menu_multi)
};

// массив меню редактирования
PROGMEM t_menu m_edit[] = {
	item_opt(ed_uint, &number1, 0),
	item_opt(ed_int, &number2, 0),
	item_opt(ed_scal1, &number3, 0),
	item_opt(ed_scal2, &number4, 0)
};

// массив меню управления светодиодами
PROGMEM t_menu m_flags[] = {
	item_opt(port_val, &PORTC, 0x01),
	item_opt(port_val, &PORTC, 0x02),
	item_opt(port_val, &PORTC, 0x04),
	item_opt(port_val, &PORTC, 0x08)
};

// массив основного меню
PROGMEM t_menu m_menu[] = {
	item_sub(mm1,m_mode),	// подменю режимов отображения меню
	item_sub(mm2,m_edit),	// подменю редакторов
	item_sub(mm3,m_flags),	// подменю флажков 
	item_sim(mm4,menu_test)			// простой пункт - небольшая анимация
};

// функция, демонстрирующая вывод небольших столбиков
static void test(void){
	init_v_scale_sym();			// инициализируем знакогенератор
	lcd_clrscr();				// очищаем дисплей
	for(uint8_t i = 0; i < 50; i++){	// демонстрация состоит из 50 повторов
		for(uint8_t x=0; x<16;x++){	// в каждой позиции дисплея
			lcd_gotoxy(x,0);		// в первой строке
			v_bar(rand(),255);	// выводим случайной высоты столбик
			lcd_gotoxy(x,1);		// и во второй строке
			v_bar(rand(),255);	// то же самое
		}
		_delay_ms(100);			// каждый цикл демонстрации длится 0,1 сек
	}
}

// главная функция
int main(void){
	char s[16];
	EVENT ev = EV_NONE;
	uint8_t mode=0;
	uint8_t menu_r;
	PORTB = 0xFF;				// порт с кнопками - вкл. подтяжки
	DDRC = 0x0F;				// порт со светодиодами
	PORTC = DDRC;				

	lcd_init(LCD_DISP_ON);			// инициализация LCD
	while(1){
		// очищаем индикатор
		lcd_clrscr();
		// выводим "заставку"
		lcd_puts_P("TUI DEMO\nPRESS * TO MENU");

		// ждем поступления события - нажатия кнопки включения меню
		do {
			ev = get_event();		// получаем событие
		}while(ev == EV_NONE);
		
		switch(ev){
		case EV_ENTER:
			// обрабатываем меню в зависимости от текущего режима
			menu_r = (mode==0) ? do_menu1(m_menu, sizeof(m_menu)/sizeof(t_menu)) : do_menu2(m_menu, sizeof(m_menu)/sizeof(t_menu));

			// выполняем действие в зависимости от полученного кода меню
			switch(menu_r){
			case menu_multi:
				mode = 1;		// включаем многострочный режим меню
				break;
			case menu_one:
				mode = 0;		// включаем однострочный режим меню
				break;
			case menu_test:
				test();		// выполняем демонстрационную функцию
				break;
			}
			break;
		}
	}
}

// редкатирование уровней на порту С - управление светодиодами
static pchar port_val(void* data, uint8_t v, uint8_t ch){
	pchar	ts;
		
	if(ch){ // параметр ch не равный нулю указывает, что надо изменить состояние
		// при этом указатель data считаем указателем на байт
		*((uint8_t*)data) ^= v;
	}
		
	// готовим текст в зависимости от того, какой светодиод обрабатывается
	switch(v){
	case 1: ts = PSTR("LED 1"); break;
	case 2: ts = PSTR("LED 2"); break;
	case 4: ts = PSTR("LED 3"); break;
	case 8: ts = PSTR("LED 4"); break;
	}
	// готовим строку для возврата
	strcpy_P(tr,ts);				// копируем первую часть строки	
	if(*((uint8_t*)data) & v)		// проверяем состояние флага и
							// прибавляем соответствующий текст
		strcat_P(tr,PSTR(" ON"));	
	else
		strcat_P(tr,PSTR(" OFF"));
	// возвращаем полученную строку
	return tr;	
}

// редактирование беззнакового целого числа
static pchar ed_uint(void* data, uint8_t v, uint8_t ch){
	char ts[6];
	if(ch){ // если наджо изменить - просто вызываем нужный редактор
		lcd_clrscr();
		*(uint16_t*)data = edit_uint(0,PSTR("Unsigned: "),*(uint16_t*)data,50,20000);
	}
	// подготавливаем строку, которую видно в меню
	strcpy_P(tr,PSTR("UINT EDIT "));
	ltoa((uint16_t)(*(uint16_t*)data),ts,10);
	strcat(tr,ts);
	return tr;
}

// редактирование целого со знаком - аналогично предыдущему
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

// шкала в виде верньера
static pchar ed_scal2(void* data, uint8_t v, uint8_t ch){
	uint8_t tmp = *(uint8_t*)data;
	uint8_t old = tmp;
	uint8_t event;
	char ts[5];
	
	if(ch){ // редактирование выполняется прямо в этой функции
		init_hbar_scale_sym();		// подготавливаем знакогенератор
		// редактируем
		lcd_clrscr();
		lcd_gotoxy(0,0);
		lcd_puts_p(PSTR("SCALE 2"));
		do{
			se_percent(0,1,tmp);	// просто рисуем текущее состояние шкалы
			// ожидаем события
			do{
				event = get_event();
			} while(!event);
			// выполняем обработку поступившего события
			switch(event){
			case EV_ESCAPE:
				tmp = old;		// при отмене восстанавливаем исходное значение
			case EV_ENTER:
				*(uint8_t*)data = tmp; // по ENTER готовимся вернуть текущее значение
				goto exit;
			// снопки БОЛЬШЕ-МЕНЬШЕ просто меняют значение в заданных пределах
			case EV_NEXT:
				if(++tmp > 100) tmp = 100;
				break;
			case EV_PREV:
				if(tmp > 0) tmp--;
			}
		}while(1);
	} 
	// готовим строку, которую видно в меню
exit:
	strcpy_P(tr,PSTR("SCALE VBAR "));
	itoa(tmp,ts,10);
	strcat(tr,ts);
	return tr;
}


// шкала в виде сплошной полосы - аналогично предыдущей
static pchar ed_scal1(void* data, uint8_t v, uint8_t ch){
	uint8_t tmp = *(uint8_t*)data;
	uint8_t old = tmp;
	uint8_t event;
	char ts[5];
	
	if(ch){
		// редактируем
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

