/**
 * \file mmenu.h
 *
 *  Created on: 28.10.2009
 *      Author: ARV
 * 
 */

#ifndef MMENU_H_
#define MMENU_H_

#include "events.h"
#include <avr/pgmspace.h>

#ifndef NULL
#define NULL (void*)(0)
#endif

/// установить макрос в 1, если нужно иметь систему МНОГОСТРОЧНОГО меню, т.е. без строки-подсказки
#define MULTILINE_MENU 1

/// установить в 0, если не надо индицировать наличие подменю в пункте
#define INDICATE_SUBMENU 1

/// код символа для индикации наличия подменю в пункте
//#define SUB_SYMBOL 126
#define SUB_SYMBOL 126

/// код символа для выделения текущего пункта (имеет смысл только для многострочного меню)
//#define SELECTED 126
#define SELECTED 62

/// макрос для описания пункта-опции: f - указатель на функцию, d - указатель на данные, о - вспомогательный байт
#define item_opt(f, d, o)	{	\
	.id = M_OPTMENU,			\
	.item.omi.get_text = f,		\
	.item.omi.data = d,	 		\
	.item.omi.optid = o}
	
/// макрос для описания пункта-субменю: t - текст, s - указатель на подменю
#define item_sub(t, s)		{	\
	.id = M_SUBMENU, 			\
	.item.smi.text = t, 		\
	.item.smi.sub_cnt = sizeof(s)/sizeof(t_menu),\
	.item.smi.subm = s}
	
/// макрос для описания обычного пункта: t - текст, v - возвращаемое число
#define item_sim(t, v) 	{	\
	 .id = M_SUBMENU + v,	 	\
	 .item.smi.text = t, 		\
	 .item.smi.sub_cnt = 0,		\
	 .item.smi.subm = NULL}

#if MULTILINE_MENU == 1
/// обращение к ГЛАВНОМУ меню в ОДНУ СТРОКУ
#define menu_result(x) do_menu2(x, sizeof(x)/sizeof(t_menu))
#else
/// обращение к ГЛАВНОМУ меню в НЕСКОЛЬКО СТРОК
#define menu_result(x) do_menu1(x, sizeof(x)/sizeof(t_menu))
#endif

/// указатель на функцию получения текста меню-опции
/// функция получает указатель на данные и некое число
/// возвращает указатель на строку в ОЗУ (!!!)
typedef char* (*f_get_txt)(void*, uint8_t, uint8_t);

/// определение указателя на структуру элемента меню
typedef struct Node* tag_menu;

/// указатель на строку в сегменте памяти
typedef PROGMEM char* pchar;

/// варианты пунктов меню
typedef enum {
	M_SUBMENU = 0x00,	/// пункт с подменю
	M_OPTMENU = 0x80	/// пункт-опция
} MENU_ITEM_TYPE;

/// определение типа для пункта "опция"
typedef struct {
	f_get_txt get_text;	/// указатель на функцию запроса текста
	void* data;			/// указатель на данные
	uint8_t optid;		/// число-идентификатор опции
} MENU_OPT_ITEM;

/// определение типа для пункта "подменю"
typedef struct {
	pchar text;			/// текст
	uint8_t sub_cnt;	/// количество подпунктов
	tag_menu subm;		/// ссылка на подменю
} MENU_SUB_ITEM;

/// определение типа для элемента меню
typedef union{
	MENU_SUB_ITEM smi;	/// либо пункт с подменю (или простой пункт)
	MENU_OPT_ITEM omi;	/// либо пункт-опция
} MENU_ITEM;

/**
 * Каждый пункт меню имеет обязательный идентификатор, который равен:
 * (M_SUBMENU + число) для пункта с подменю, причем число (не более 0x7F) - это
 *                     код возврата из меню (имеет смысл только для простых пунктов)
 * (M_OPTMENU) для пунктов-опций
 *
 */

/// структура элемента меню
typedef struct Node{
	MENU_ITEM_TYPE id;	/// идентификатор меню
	MENU_ITEM item;		/// собственно элемент меню
} t_menu;

extern t_menu m_file;

/**
 * Система меню строится из массивов пунктов меню. "Главное" меню (или корневое)
 * это массив пунктов, среди которых могут быть пункты типа "подменю" со ссылками
 * на другие массивы пунктов и т.д.
 */

uint8_t do_menu1(t_menu *m, uint8_t cnt);	// однострочное меню со строкой-подсказкой
uint8_t do_menu2(t_menu *m, uint8_t cnt);	// многострочное меню


#endif /* MMENU_H_ */
