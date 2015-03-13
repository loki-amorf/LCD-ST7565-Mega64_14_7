/** \file sedit.c Simple Editors
 * Модуль с набором функций для редактирования параметров.
 * Предназначен для раборты совместно с модулем текстового меню mmenu.h  
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

/// Битовый массив символа, используемого для "нулевого" знакоместа горизонтальной шкалы
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

// вспомогательная переменная
static uint8_t offset_first;

/** Инициализация пользовательского знакогенератора для отображения шкал в виде вертикального столбика
 * 
 */
// void init_v_scale_sym(void){
// //    lcd_command(0x48);								// команда записи в ОЗУ знакогенератора
// 	for(uint8_t i=0; i<8; i++){						// запишем 8 символов
// 		for(uint8_t y=0; y<8; y++){					// состоящих из 8 горизонтальных строк
// 			// каждый символ содержит по нарастающей заполненные строки снизу вверх
// 			if(y >= (7-i))							
// //				lcd_data(0xFF);
// 			else
// 				// заменив 0 на значение 0b00100 получим тонкий столбик для "пустой" шкалы
// 				lcd_data(0);
// 		}
// 	}
// }

/** Вывод шкалы-столбика высотой в одно знакоместо.
 * Перед обращением к функции должен быть проинициализирован пользовательский знакогенератор
 * при помощи функции init_v_scale_sym() 
 * @param val выводимое на шкалу значение
 * @param base значение, соответствующее максимуму шкалы
 * \note Функция выводит столбик в текущей позиции "курсора".
 */
void v_bar(uint8_t val, uint8_t base){
	if(val > base) val = base;						// не допускаем превышения шкалы
	gdWriteChar((val * 8)/base + FIRST_USER_SYMBOL);	// вычисляем и выводим нужный символ
}

/** Вывод шкалы-столбика высотой в два знакоместа.
 * Перед обращением к функции должен быть проинициализирован пользовательский знакогенератор
 * при помощи функции init_v_scale_sym() 
 * @param pos позиция столбика по горизонтали
 * @param val выводимое значение
 * @param base максимум шкалы
 */
void v2_bar(uint8_t pos, uint8_t val, uint8_t base){
	uint8_t tmp = val * 15 / base + FIRST_USER_SYMBOL;// вычисляем относительное значение шкалы
	if(tmp > 15) tmp = 15; 							// не позволяем превысить максимум
	if(tmp > 8){
		// если шкала выше, чем одно знакоместо
		gdSetXY(pos,1);				
		gdWriteChar(8);
		gdSetXY(pos,0);
		tmp -= 7;
	} else {
		// если шкала ниже, чем одно знакоместо
		gdSetXY(pos,0);
		gdWriteChar(' ');
		gdSetXY(pos,1);
	}
	gdWriteChar(tmp);
    disp_send_frame();		
}

/** Инициализация пользовательского знакогенератора для отображения шкал в виде горизонтального столбика
 * 
 */
void init_h_scale_sym(void){
	uint8_t mask = 0b00100000;
//    lcd_command(0x48);									// команда записи в знакогенератор
    for(uint8_t i=0; i<6; i++){							// запишем 6 символов
    	for(uint8_t y=0; y<8; y++){						// состоящих из 8 строк
//    		lcd_data(pgm_read_byte(&h_symb[y]) | mask);	// битовые образы загружаем из массива
    	}
    	mask |= (mask >> 1);							// и добавляем вертикальную полоску
														// нарастающей слева направо ширины
    }
    offset_first = 0;									// задаем значение вспомогательной переменной
}

/** Инициализация пользовательского знакогенератора для отображения шкал в виде горизонтального столбика.
 * Функция подготавливает знакогенератор для вывода шкалы типа "настройка приемника", т.е. для
 * отображения вертикального штриха в нужном месте шкалы. 
 */
void init_hbar_scale_sym(void){
	uint8_t mask = 0b00100000;
//    lcd_command(0x48);
    for(uint8_t i=0; i<6; i++){
    	mask >>= 1; // отличие от init_h_scale_sym() только в этой строке 
    	for(uint8_t y=0; y<8; y++){
//    		lcd_data(pgm_read_byte(&h_symb[y]) | mask);
    	}
    }
    offset_first = 5; // и в этой ;-)
}

/** Функция задания нового пользовательского знакогенератора из образа во flash.
 * 
 * @param symbol_count количество символов
 * @param symbols начало массива битовых образов знакогенератора во flash 
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

/** Отображение величины процента в виде горизонтальной шкалы.
 * Начало шкалы в текущей позиции ЖКИ, длина шкалы - 10 знакомест.
 * @param percent выводимое число
 */
void se_scale_percent(uint8_t percent){
	uint8_t pos = 0;
	uint8_t t;
	if(percent > 100) percent = 100;				// больше 100% быть не может
	while(percent > 10){							// рисуем толстую часть шкалы
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
	gdWriteChar(t);	// рисуем огрызочек шкалы
	pos++;
	while(pos < 10){								// рисуем тонкую часть шкалы
		gdWriteChar(FIRST_USER_SYMBOL+offset_first);
		pos++;
	}
}

/** Отображение числового значения в процентах и на горизонтальной шкале
 * 
 * @param x позиция начала вывода по горизонтали
 * @param y номер строки ЖКИ
 * @param percent выводимое значение
 */
void se_percent(uint8_t x, uint8_t y, uint8_t percent){
	uint8_t s[4];
	
	itoa(percent,s,10);								// преобразуем число в строку
	gdSetXY(x,y);								// позиционируем курсор
	lcd_puts_p("    ");								// очищаем 4 знакоместа
	gdSetXY(x,y);								// снова позиционируем
	lcd_puts(s);									// выводим число
	gdWriteChar('%');									// выводим символ процента
	gdSetXY(x+4,y);								// позиционируем на начало шкалы
	se_scale_percent(percent);						// и выводим шкалу
	disp_send_frame();	
}

/** Преобразование беззнакового целого в строку заданной длины.
 * Строковое представление числа дополняется до нужного количества символов
 * незначащими нулями слева.
 * @param s указатель на выходную строку (размер должен быть достаточным!)
 * @param val преобразуемое число
 * @param digits количество позиций в строке
 */
static
void ui2s(char* s, uint16_t val, uint8_t digits){
	for(uint8_t i = 1; i <= digits; i++){
		s[digits-i] = '0' + val % 10;
		val /= 10;
	}
	s[digits] = 0;
}

/** Преобразование целого числа со знаком в строку заданной длины.
 * Строковое представление числа дополняется до нужного количества символов
 * незначащими нулями слева.
 * @param s указатель на выходную строку (размер должен быть достаточным!)
 * @param val преобразуемое число
 * @param digits количество позиций в строке
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

/** Подсчет числа знаков, необходимых для представления числа в виде строки.
 *
 * @param v входное число
 * @return число необходимых знаков
 */
static
uint8_t number_digit(int32_t v){
	uint8_t result=0;
	uint32_t tv = v < 0 ? -v : v;

	for(; tv; tv /= 10) result++;
	if(!result || (v < 0)) result++;
	return result;
}

/// большее из двух
#define MAX(x,y) ((x) < (y) ? (y) : (x))

/** Позиционное редактирование числа со знаком.
 * Функция позволяет вывести подсказку и изменить значение переменной в диалоговом режиме.
 * @param y номер строки дисплея для редактирования
 * @param msg указатель на строку во FLASH с подсказкой
 * @param val значение переменной
 * @param min минимально допустимое значение переменной
 * @param max максимально допустимое значение переменной
 * @return новое значение
 */

//int16_t edit_sint(uint8_t y, prog_char *msg, int16_t val, int16_t min, int16_t max){
int16_t edit_sint(uint8_t y, const char *msg, int16_t val, int16_t min, int16_t max){	
	int16_t res = val;
	uint8_t edit_pos=0,						// номер редактируемой позиции
			pos=0,							// позиция числа в строке дисплея
			digits;							// количество позиций в числе
	uint8_t ev = EV_NONE;					// событие
	char vstr[7];							// вспомогательная строка

	//вычисляем число знаков в числе
	digits = MAX(number_digit(max), number_digit(min));
	gdSetXY(0,y);						// выводим сообщение с начала строки
	lcd_puts_p(msg);
	if(msg)									// если есть сообщение, то
		while(pgm_read_byte(&msg[pos])) pos++;// вычисляем позицию числа

	// позиционное редактирование числа
	do{
		si2s(vstr,res,digits);				// получаем символьное представление числа
//		lcd_command(LCD_DISP_ON);
		gdSetXY(pos,y);					// позиционируем в место вывода числа
		lcd_puts(vstr);						// выводим представление числа
//		lcd_command(LCD_DISP_ON_CURSOR_BLINK);
        gdLoadFont(font_ks0066_ru_08, 0, FONT_DIR_0);
		gdSetXY(pos+edit_pos,y);			// ставим курсор на редактируемую позицию
		do {
			ev = get_event();
		}while(ev == EV_NONE);				// получаем событие
		switch(ev){
		case EV_ENTER: // кнопка изменения позиции
			if((min < 0) && (edit_pos == 0)){
				// если число может быть с минусом и редактируемая позиция как раз знаковая
				if(vstr[edit_pos] == ' ')	// меняем знак на противоположный
					vstr[edit_pos] = '-';
				else
					vstr[edit_pos] = ' ';
			} else {
				// если число только положительное или редактируем НЕ знак
				vstr[edit_pos]++;				// наращиваем символ
				res = atoi(vstr);					// текущее представление конвертим в число
				if((vstr[edit_pos] > '9') 
					|| (res > max)
					|| (res < min)
					) 
					vstr[edit_pos] = '0';
			}
			break;
		case EV_NEXT: // кнопка перехода к следующей позиции
			if(++edit_pos >= digits)		// наращиваем позицию, пока не выйдет за пределы
				edit_pos = 0;				// а как выйдет - по кругу к первой переходим
			break;
		case EV_PREV: // кнопка перехода к предыдущей позиции
			if(--edit_pos > digits)			// уменьшаем позицию, пока не перекрутится в 255
				edit_pos = digits-1;		// как перекрутится - переходим к последней позиции
			break;
		}
		res = atoi(vstr);					// текущее представление конвертим в число
		disp_send_frame();			
	} while(ev != EV_ESCAPE); // надо проверить на условие выхода из редактора
	gdLoadFont(font_ks0066_ru_08, 1, FONT_DIR_0);
//    disp_send_frame();		
//	lcd_command(LCD_DISP_ON);
	return res;						// возвращаем текущее значение
}

/** Позиционное редактирование числа без знака.
 * Функция позволяет вывести подсказку и изменить значение переменной в диалоговом режиме.
 * @param y номер строки дисплея для редактирования
 * @param msg указатель на строку во FLASH с подсказкой
 * @param val значение переменной
 * @param min минимально допустимое значение переменной
 * @param max максимально допустимое значение переменной
 * @return новое значение
 */

//uint16_t edit_uint(uint8_t y, prog_char *msg, uint16_t val, uint16_t min, uint16_t max){
uint16_t edit_uint(uint8_t y, const char *msg, uint16_t val, uint16_t min, uint16_t max){	
	uint16_t res = val;
	uint8_t edit_pos=0,						// номер редактируемой позиции
			pos=0,							// позиция числа в строке дисплея
			digits;							// количество позиций в числе
	uint8_t ev = EV_NONE;					// событие
	char vstr[6];							// вспомогательная строка

	digits = number_digit(max);				//вычисляем число знаков в числе
	gdSetXY(0,y);						// выводим сообщение с начала строки
	lcd_puts_p(msg);
	if(msg)									// если есть сообщение, то
		while(pgm_read_byte(&msg[pos])) pos++;	// вычисляем позицию числа

	// позиционное редактирование числа
	do{

		if(res < min) res = min;

		ui2s(vstr,res,digits);				// получаем символьное представление числа
//		lcd_command(LCD_DISP_ON);
		gdSetXY(pos,y);					// позиционируем в место вывода числа
		lcd_puts(vstr);						// выводим представление числа
//		lcd_command(LCD_DISP_ON_CURSOR_BLINK);
        gdLoadFont(font_ks0066_ru_08, 0, FONT_DIR_0);
		gdSetXY(pos+edit_pos,y);			// ставим курсор на редактируемую позицию
		do {
			ev = get_event();
		}while(ev == EV_NONE);				// получаем событие
			
		switch(ev){
		case EV_ENTER: // кнопка изменения позиции
			vstr[edit_pos]++;				// наращиваем символ
			res = atol(vstr);					// текущее представление конвертим в число
			if((vstr[edit_pos] > '9') || (res > max)) 
				vstr[edit_pos] = '0';
			break;
		case EV_NEXT: // кнопка перехода к следующей позиции
			if(++edit_pos >= digits)		// наращиваем позицию, пока не выйдет за пределы
				edit_pos = 0;				// а как выйдет - по кругу к первой переходим
			break;
		case EV_PREV: // кнопка перехода к предыдущей позиции
			if(--edit_pos > digits)			// уменьшаем позицию, пока не перекрутится в 255
				edit_pos = digits-1;		// как перекрутится - переходим к последней позиции
			break;
		}
		res = atol(vstr);					// текущее представление конвертим в число
		disp_send_frame();	//
	} while(ev != EV_ESCAPE); // надо проверить на условие выхода из редактора
	 gdLoadFont(font_ks0066_ru_08, 1, FONT_DIR_0);
//	lcd_command(LCD_DISP_ON);
//     disp_send_frame();	//
	return res;						// возвращаем текущее значение
}
