/**
 * \file mmenu.c Модуль mini-menu
 * \brief Для создания системы меню на малоразмерных символьных индикаторах 
 * Created on: 28.10.2009
 * \author: ARV
 */

#include <avr/pgmspace.h>
#include <stdio.h>
#include "../lcd/st7565-driver.h"
#include "buttons.h"
#include "mmenu.h"
#define LCD_LINES          8 
static	uint8_t 	current = 0;	// текущий выбранный пункт

// uint8_t do_menu1(t_menu *m, uint8_t cnt){
// 	uint8_t 	result;		// результат для возврата
// 	uint8_t		first = 0;		// первый видимый на дисплее пункт
// 	EVENT		ev;
// 	f_get_txt	gt;
// 	char		*str;
// 	register uint8_t flags;
// 
// 	current = 0;
// 	while(1)
// 	{
//          lcd_clearscreen ();
//          disp_clear();
// 		// выводим строки пунктов меню
// 		for(uint8_t y=0; y < LCD_LINES; y++){
// 			if(y >= cnt) {
// 				break;
// 			}
// 		
// 			uint8_t z=0;
// 			z=y*8;
// 			gdSetXY(0,z);
// 			flags = 0;
// 			if(pgm_read_byte(&m[first+y].id) & M_OPTMENU){	// пункт-опция
// 				flags |= 1;
// 			}
// 			if((first+y) == current){
// 				flags |= 2;
// 			}
// 	
// 			if(flags & 1){
// 				// извлекаем функцию получения текста меню-опции
// 				gt = (void*)pgm_read_word(&m[first+y].item.omi.get_text);
// 				// выводим то, что эта функция вернет
// 				str = gt((void*)pgm_read_word(&m[first+y].item.omi.data),pgm_read_byte(&m[first+y].item.omi.optid),0);
// 			} else { // пункт-субменю или простой
// 				// выводим текст пункта меню
// 				str = (void*)pgm_read_word(&m[first+y].item.smi.text);
// 			}
// 			
// 			if(flags & 2){
// 				// вывод выделенного пункта
// 				gdWriteChar(SELECTED);
// 			} else {
// 				gdWriteChar(' ');
// 			}
// 			
// 			if(flags & 1){
// 				lcd_puts(str);
// 			} else {
// 				lcd_puts_p(str);
// 			}
// 			
// #if INDICATE_SUBMENU != 0
// 			// дополняем пункт с подменю индикатором
// 			if(!(flags & 1) && pgm_read_byte(&m[first+y].item.smi.sub_cnt)){
// 				// вывод выделенного пункта
// 				gdWriteChar(SUB_SYMBOL);
// 			}
// #endif
// 		}
// 		// цикл обработки событий
// 		do{
// 			// получаем событие
// 			ev = get_event();
// 		switch(ev){
// 		case EV_NEXT:
// 			// переход к следующему пункту меню
// 			if(current < (cnt-1)){
// 				current++;
// 				if((current-first) >= LCD_LINES){
// 					first++;
// 				}
// 			}
// 			break;
// 		case EV_PREV:
// 			// переход к предыдущему пункту меню
// 			if(current){ 
// 				current--;
// 				if(current < first){
// 					first--;
// 				}
// 			}
// 			break;
// 		case EV_ENTER:
// 			// выбор пункта обрабатывается по-разному в зависимости от его типа
// 			if(flags & 1){
// 				// пункт-опция: просто установим признак модификации
// 				gt = (void*)pgm_read_word(&m[current].item.omi.get_text);
// 				gt((void*)pgm_read_word(&m[current].item.omi.data),pgm_read_byte(&m[current].item.omi.optid),1);
// 			} else {
// 				if(pgm_read_byte(&m[current].item.smi.sub_cnt)){
// 					// пункт-субменю: погружаемся в субменю
// 					result = do_menu2((void*)pgm_read_word(&m[current].item.smi.subm), 
// 										pgm_read_byte(&m[current].item.smi.sub_cnt));
// 					// если возвращен не ноль - считаем, что меню надо закрыть
// 					if(result) return result;
// 				} else {
// 					// простой пункт: вернем идентификатор пункта
// 					return pgm_read_byte(&m[current].id);
// 				}
// 			}
// 			break;
// 		case EV_ESCAPE:
// 			// при выходе из меню вернем 0
// 			disp_clear();
// 			return 0;
// 		case EV_NONE:
// 			// отсутствие события не обрабатывается никак
// 			continue;
// 		}
// 		// пока не поступит какое-либо событие, цикл не завершается, т.е. обновления дисплея нет
// 		} while(ev == EV_NONE);
// 	}
// }


uint8_t do_menu2(t_menu *m, uint8_t cnt){
	uint8_t 	result;		// результат для возврата
	uint8_t		first = 0;		// первый видимый на дисплее пункт
	EVENT		ev;
	f_get_txt	gt;
	char		*str;
	register uint8_t flags;

	current = 0;
	while(1)
	{
//		lcd_clearscreen ();
		disp_clear();
		// выводим строки пунктов меню
		for(uint8_t y=0; y < LCD_LINES; y++){
			if(y >= cnt) {
				break;
			}
			
			uint8_t z=0;
			z=y*8;
			gdSetXY(0,z);
			flags = 0;
			if(pgm_read_byte(&m[first+y].id) & M_OPTMENU){	// пункт-опция
				flags |= 1;
			}
			if((first+y) == current){
				flags |= 2;
			}
			
			if(flags & 1){
				// извлекаем функцию получения текста меню-опции
				gt = (void*)pgm_read_word(&m[first+y].item.omi.get_text);
				// выводим то, что эта функция вернет
				str = gt((void*)pgm_read_word(&m[first+y].item.omi.data),pgm_read_byte(&m[first+y].item.omi.optid),0);
				} else { // пункт-субменю или простой
				// выводим текст пункта меню
				str = (void*)pgm_read_word(&m[first+y].item.smi.text);
			}
			
			if(flags & 2){
				// вывод выделенного пункта
				gdWriteChar(SELECTED);
				} else {
				gdWriteChar(' ');
			}
			
			if(flags & 1){
				lcd_puts(str);
				} else {
				lcd_puts_p(str);
			}
			
			#if INDICATE_SUBMENU != 0
			// дополняем пункт с подменю индикатором
			if(!(flags & 1) && pgm_read_byte(&m[first+y].item.smi.sub_cnt)){
				// вывод выделенного пункта
				gdWriteChar(SUB_SYMBOL);
			}
			#endif
			
		    disp_send_frame();	
			
		}
		// цикл обработки событий
		do{
			// получаем событие
			ev = get_event();
			switch(ev){
				case EV_NEXT:
				// переход к следующему пункту меню
				if(current < (cnt-1)){
					current++;
					if((current-first) >= LCD_LINES){
						first++;
					}
				}
				break;
				case EV_PREV:
				// переход к предыдущему пункту меню
				if(current){
					current--;
					if(current < first){
						first--;
					}
				}
				break;
				case EV_ENTER:
				// выбор пункта обрабатывается по-разному в зависимости от его типа
				if(flags & 1){
					// пункт-опция: просто установим признак модификации
					gt = (void*)pgm_read_word(&m[current].item.omi.get_text);
					gt((void*)pgm_read_word(&m[current].item.omi.data),pgm_read_byte(&m[current].item.omi.optid),1);
					} else {
					if(pgm_read_byte(&m[current].item.smi.sub_cnt)){
						// пункт-субменю: погружаемся в субменю
						result = do_menu2((void*)pgm_read_word(&m[current].item.smi.subm),
						pgm_read_byte(&m[current].item.smi.sub_cnt));
						// если возвращен не ноль - считаем, что меню надо закрыть
						if(result) return result;
						} else {
						// простой пункт: вернем идентификатор пункта
						return pgm_read_byte(&m[current].id);
					}
				}
				break;
				case EV_ESCAPE:
				// при выходе из меню вернем 0
				disp_clear();
				return 0;
				case EV_NONE:
				// отсутствие события не обрабатывается никак
				continue;
			}
			// пока не поступит какое-либо событие, цикл не завершается, т.е. обновления дисплея нет
		} while(ev == EV_NONE);
	}
}

