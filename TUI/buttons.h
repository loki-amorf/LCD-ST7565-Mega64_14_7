/** \file buttons.h
 * \brief Заголовочный файл модуля обработки кнопок
 * содержит константы и макросы, используемые в программе
 * \par
 * \author ARV
 * \note
 * \n Схема:
 * \n \date	14.11.2008 ... 	__.__.2008
 * \par
 * \version <версия>.	\n
 * Copyright 2008 © ARV. All rights reserved. </b>
 * \par
 * Для сборки проекта требуется:\n
 * 	-# WinAVR-20080411 или более новая версия
 *
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

/// число десятков миллисекунд от момента начала удержания до автоповтора
#define FIRST_RPT 100
/// число десятков миллисекунд до автоповтора нажатой кнопки
#define RPT_DELAY 5

#define KEYPIN		PD5

/// код отсутствия нажатых кнопок
#define k_none		0
/// код кнопки SET
#define k_set		_BV(0)
/// код кнопки ВВЕРХ
#define k_up		_BV(1)
/// код кнопки MODE
#define k_mode		_BV(2)
/// код кнопки ВНИЗ
#define k_dn 		_BV(3)

/// функция возврата кода нажатой кнопки
uint8_t get_key(void);

#endif /* BUTTONS_H_ */
