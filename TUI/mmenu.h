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

/// ���������� ������ � 1, ���� ����� ����� ������� �������������� ����, �.�. ��� ������-���������
#define MULTILINE_MENU 1

/// ���������� � 0, ���� �� ���� ������������ ������� ������� � ������
#define INDICATE_SUBMENU 1

/// ��� ������� ��� ��������� ������� ������� � ������
//#define SUB_SYMBOL 126
#define SUB_SYMBOL 126

/// ��� ������� ��� ��������� �������� ������ (����� ����� ������ ��� �������������� ����)
//#define SELECTED 126
#define SELECTED 62

/// ������ ��� �������� ������-�����: f - ��������� �� �������, d - ��������� �� ������, � - ��������������� ����
#define item_opt(f, d, o)	{	\
	.id = M_OPTMENU,			\
	.item.omi.get_text = f,		\
	.item.omi.data = d,	 		\
	.item.omi.optid = o}
	
/// ������ ��� �������� ������-�������: t - �����, s - ��������� �� �������
#define item_sub(t, s)		{	\
	.id = M_SUBMENU, 			\
	.item.smi.text = t, 		\
	.item.smi.sub_cnt = sizeof(s)/sizeof(t_menu),\
	.item.smi.subm = s}
	
/// ������ ��� �������� �������� ������: t - �����, v - ������������ �����
#define item_sim(t, v) 	{	\
	 .id = M_SUBMENU + v,	 	\
	 .item.smi.text = t, 		\
	 .item.smi.sub_cnt = 0,		\
	 .item.smi.subm = NULL}

#if MULTILINE_MENU == 1
/// ��������� � �������� ���� � ���� ������
#define menu_result(x) do_menu2(x, sizeof(x)/sizeof(t_menu))
#else
/// ��������� � �������� ���� � ��������� �����
#define menu_result(x) do_menu1(x, sizeof(x)/sizeof(t_menu))
#endif

/// ��������� �� ������� ��������� ������ ����-�����
/// ������� �������� ��������� �� ������ � ����� �����
/// ���������� ��������� �� ������ � ��� (!!!)
typedef char* (*f_get_txt)(void*, uint8_t, uint8_t);

/// ����������� ��������� �� ��������� �������� ����
typedef struct Node* tag_menu;

/// ��������� �� ������ � �������� ������
typedef PROGMEM char* pchar;

/// �������� ������� ����
typedef enum {
	M_SUBMENU = 0x00,	/// ����� � �������
	M_OPTMENU = 0x80	/// �����-�����
} MENU_ITEM_TYPE;

/// ����������� ���� ��� ������ "�����"
typedef struct {
	f_get_txt get_text;	/// ��������� �� ������� ������� ������
	void* data;			/// ��������� �� ������
	uint8_t optid;		/// �����-������������� �����
} MENU_OPT_ITEM;

/// ����������� ���� ��� ������ "�������"
typedef struct {
	pchar text;			/// �����
	uint8_t sub_cnt;	/// ���������� ����������
	tag_menu subm;		/// ������ �� �������
} MENU_SUB_ITEM;

/// ����������� ���� ��� �������� ����
typedef union{
	MENU_SUB_ITEM smi;	/// ���� ����� � ������� (��� ������� �����)
	MENU_OPT_ITEM omi;	/// ���� �����-�����
} MENU_ITEM;

/**
 * ������ ����� ���� ����� ������������ �������������, ������� �����:
 * (M_SUBMENU + �����) ��� ������ � �������, ������ ����� (�� ����� 0x7F) - ���
 *                     ��� �������� �� ���� (����� ����� ������ ��� ������� �������)
 * (M_OPTMENU) ��� �������-�����
 *
 */

/// ��������� �������� ����
typedef struct Node{
	MENU_ITEM_TYPE id;	/// ������������� ����
	MENU_ITEM item;		/// ���������� ������� ����
} t_menu;

extern t_menu m_file;

/**
 * ������� ���� �������� �� �������� ������� ����. "�������" ���� (��� ��������)
 * ��� ������ �������, ����� ������� ����� ���� ������ ���� "�������" �� ��������
 * �� ������ ������� ������� � �.�.
 */

uint8_t do_menu1(t_menu *m, uint8_t cnt);	// ������������ ���� �� �������-����������
uint8_t do_menu2(t_menu *m, uint8_t cnt);	// ������������� ����


#endif /* MMENU_H_ */
