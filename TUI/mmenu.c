/**
 * \file mmenu.c ������ mini-menu
 * \brief ��� �������� ������� ���� �� ������������� ���������� ����������� 
 * Created on: 28.10.2009
 * \author: ARV
 */

#include <avr/pgmspace.h>
#include <stdio.h>
#include "../lcd/st7565-driver.h"
#include "buttons.h"
#include "mmenu.h"
#define LCD_LINES          8 
static	uint8_t 	current = 0;	// ������� ��������� �����

// uint8_t do_menu1(t_menu *m, uint8_t cnt){
// 	uint8_t 	result;		// ��������� ��� ��������
// 	uint8_t		first = 0;		// ������ ������� �� ������� �����
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
// 		// ������� ������ ������� ����
// 		for(uint8_t y=0; y < LCD_LINES; y++){
// 			if(y >= cnt) {
// 				break;
// 			}
// 		
// 			uint8_t z=0;
// 			z=y*8;
// 			gdSetXY(0,z);
// 			flags = 0;
// 			if(pgm_read_byte(&m[first+y].id) & M_OPTMENU){	// �����-�����
// 				flags |= 1;
// 			}
// 			if((first+y) == current){
// 				flags |= 2;
// 			}
// 	
// 			if(flags & 1){
// 				// ��������� ������� ��������� ������ ����-�����
// 				gt = (void*)pgm_read_word(&m[first+y].item.omi.get_text);
// 				// ������� ��, ��� ��� ������� ������
// 				str = gt((void*)pgm_read_word(&m[first+y].item.omi.data),pgm_read_byte(&m[first+y].item.omi.optid),0);
// 			} else { // �����-������� ��� �������
// 				// ������� ����� ������ ����
// 				str = (void*)pgm_read_word(&m[first+y].item.smi.text);
// 			}
// 			
// 			if(flags & 2){
// 				// ����� ����������� ������
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
// 			// ��������� ����� � ������� �����������
// 			if(!(flags & 1) && pgm_read_byte(&m[first+y].item.smi.sub_cnt)){
// 				// ����� ����������� ������
// 				gdWriteChar(SUB_SYMBOL);
// 			}
// #endif
// 		}
// 		// ���� ��������� �������
// 		do{
// 			// �������� �������
// 			ev = get_event();
// 		switch(ev){
// 		case EV_NEXT:
// 			// ������� � ���������� ������ ����
// 			if(current < (cnt-1)){
// 				current++;
// 				if((current-first) >= LCD_LINES){
// 					first++;
// 				}
// 			}
// 			break;
// 		case EV_PREV:
// 			// ������� � ����������� ������ ����
// 			if(current){ 
// 				current--;
// 				if(current < first){
// 					first--;
// 				}
// 			}
// 			break;
// 		case EV_ENTER:
// 			// ����� ������ �������������� ��-������� � ����������� �� ��� ����
// 			if(flags & 1){
// 				// �����-�����: ������ ��������� ������� �����������
// 				gt = (void*)pgm_read_word(&m[current].item.omi.get_text);
// 				gt((void*)pgm_read_word(&m[current].item.omi.data),pgm_read_byte(&m[current].item.omi.optid),1);
// 			} else {
// 				if(pgm_read_byte(&m[current].item.smi.sub_cnt)){
// 					// �����-�������: ����������� � �������
// 					result = do_menu2((void*)pgm_read_word(&m[current].item.smi.subm), 
// 										pgm_read_byte(&m[current].item.smi.sub_cnt));
// 					// ���� ��������� �� ���� - �������, ��� ���� ���� �������
// 					if(result) return result;
// 				} else {
// 					// ������� �����: ������ ������������� ������
// 					return pgm_read_byte(&m[current].id);
// 				}
// 			}
// 			break;
// 		case EV_ESCAPE:
// 			// ��� ������ �� ���� ������ 0
// 			disp_clear();
// 			return 0;
// 		case EV_NONE:
// 			// ���������� ������� �� �������������� �����
// 			continue;
// 		}
// 		// ���� �� �������� �����-���� �������, ���� �� �����������, �.�. ���������� ������� ���
// 		} while(ev == EV_NONE);
// 	}
// }


uint8_t do_menu2(t_menu *m, uint8_t cnt){
	uint8_t 	result;		// ��������� ��� ��������
	uint8_t		first = 0;		// ������ ������� �� ������� �����
	EVENT		ev;
	f_get_txt	gt;
	char		*str;
	register uint8_t flags;

	current = 0;
	while(1)
	{
//		lcd_clearscreen ();
		disp_clear();
		// ������� ������ ������� ����
		for(uint8_t y=0; y < LCD_LINES; y++){
			if(y >= cnt) {
				break;
			}
			
			uint8_t z=0;
			z=y*8;
			gdSetXY(0,z);
			flags = 0;
			if(pgm_read_byte(&m[first+y].id) & M_OPTMENU){	// �����-�����
				flags |= 1;
			}
			if((first+y) == current){
				flags |= 2;
			}
			
			if(flags & 1){
				// ��������� ������� ��������� ������ ����-�����
				gt = (void*)pgm_read_word(&m[first+y].item.omi.get_text);
				// ������� ��, ��� ��� ������� ������
				str = gt((void*)pgm_read_word(&m[first+y].item.omi.data),pgm_read_byte(&m[first+y].item.omi.optid),0);
				} else { // �����-������� ��� �������
				// ������� ����� ������ ����
				str = (void*)pgm_read_word(&m[first+y].item.smi.text);
			}
			
			if(flags & 2){
				// ����� ����������� ������
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
			// ��������� ����� � ������� �����������
			if(!(flags & 1) && pgm_read_byte(&m[first+y].item.smi.sub_cnt)){
				// ����� ����������� ������
				gdWriteChar(SUB_SYMBOL);
			}
			#endif
			
		    disp_send_frame();	
			
		}
		// ���� ��������� �������
		do{
			// �������� �������
			ev = get_event();
			switch(ev){
				case EV_NEXT:
				// ������� � ���������� ������ ����
				if(current < (cnt-1)){
					current++;
					if((current-first) >= LCD_LINES){
						first++;
					}
				}
				break;
				case EV_PREV:
				// ������� � ����������� ������ ����
				if(current){
					current--;
					if(current < first){
						first--;
					}
				}
				break;
				case EV_ENTER:
				// ����� ������ �������������� ��-������� � ����������� �� ��� ����
				if(flags & 1){
					// �����-�����: ������ ��������� ������� �����������
					gt = (void*)pgm_read_word(&m[current].item.omi.get_text);
					gt((void*)pgm_read_word(&m[current].item.omi.data),pgm_read_byte(&m[current].item.omi.optid),1);
					} else {
					if(pgm_read_byte(&m[current].item.smi.sub_cnt)){
						// �����-�������: ����������� � �������
						result = do_menu2((void*)pgm_read_word(&m[current].item.smi.subm),
						pgm_read_byte(&m[current].item.smi.sub_cnt));
						// ���� ��������� �� ���� - �������, ��� ���� ���� �������
						if(result) return result;
						} else {
						// ������� �����: ������ ������������� ������
						return pgm_read_byte(&m[current].id);
					}
				}
				break;
				case EV_ESCAPE:
				// ��� ������ �� ���� ������ 0
				disp_clear();
				return 0;
				case EV_NONE:
				// ���������� ������� �� �������������� �����
				continue;
			}
			// ���� �� �������� �����-���� �������, ���� �� �����������, �.�. ���������� ������� ���
		} while(ev == EV_NONE);
	}
}

