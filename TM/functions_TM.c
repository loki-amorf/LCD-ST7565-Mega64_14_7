//*****************************************************************************************
// функции обслуживания ключа Touch Memory
//*****************************************************************************************
#include <avr/io.h>
#include "functions_TM.h"
//#include "master_controller_r.h"
//#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <compat/ina90.h>	// cli();sei();wdt_reset();asm("nop");_EEPUT(ADR,VALoad);_EEGET(VARead,ADR);
#include <avr/pgmspace.h>
// на линии логическая "1"
void inp(void) //изначально у автора использовался порт PD2, я заменяю на PG3
{
//    PORTD   = 0x00;     // 0000 0000
//    DDRD    = 0xF0;     // 1111 0000
	
//	  PORTG   = 0x00;     // 0000 0000
//	  DDRG    = 0x00;     // 0000 0000
	                      //      |
						
	PORTG &= ~(1<<PG4);       //setup on PORTG (Disable pull UPs)							
	DDRG  &= ~(1<<DDG4);      //setup on PORTG (Enable as input)					
}

//*****************************************************************************************
// на линии логический "0"
void out(void)
{
//    PORTD   = 0x00;     // 0000 0000
//    DDRD    = 0xF4;     // 1111 0100
	
//    PORTG   = 0x00;     // 0000 0000
//    DDRG    = 0x08;     // 0000 1000
	                      //      |	
	PORTG &= ~(1<<PG4);       //setup on PORTG (Disable pull UPs)
	DDRG  |=  (1<<DDG4);      //setup on PORTG (Enable as output)						
}

//*****************************************************************************************
void write_TM_manag(void)
{
	static unsigned char a;
	for (a = 0; a < 8; a ++){
	_EEPUT(EE_AAD_MANAG + a, data_ROM[a]);
	}
}

//*****************************************************************************************
void write_TM_oper(void)
{
	static unsigned char a;
	for (a = 0; a < 8; a ++){
	_EEPUT(EE_AAD_OPER + a, data_ROM[a]);
	}
}

//*****************************************************************************************
void read_TM_manag(void)
{
	static unsigned char a;
	for (a = 0; a < 8; a ++){
	_EEGET(tm_manag[a], EE_AAD_MANAG + a);
	}
}

//*****************************************************************************************
void read_TM_oper(void)
{
	static unsigned char a;
	for (a = 0; a < 8; a ++){
	_EEGET(tm_oper[a], EE_AAD_OPER + a);
	}
}

//*****************************************************************************************
// функция передает ключу Touch Memory команду чтения ПЗУ

void ROM_33(void)
{
	static unsigned char rrom, rep;
    rrom = 0x33;
    rep = 8;
RD_B:
    out();				// подача в линию нулевого бита
    if (rrom & 0x01)
        goto RD_1;
	_delay_loop_1(80);	// ~ 60 mks
    goto RD_2;
RD_1:
    _delay_loop_1(5);	// ~ 5 mks
    inp();				// подача в линию единичного бита
	_delay_loop_1(80);	// ~ 60 mks
    goto RD_3;
RD_4:
    _delay_loop_1(5);	// ~ 5 mks
    goto RD_B;
RD_2:
    inp();				// подача в линию единичного бита
RD_3:
	rrom >>= 1;
	if (--rep)
        goto RD_4;
	_delay_loop_1(80);	// ~ 60 mks
}

//*****************************************************************************************
// функция читает из ПЗУ ключа Touch Memory 8 байт

void rd_data_ROM(void)
{
	static unsigned char a, rep, byte_rom;

    for (a = 0; a < 8; a ++){
        for (rep = 0; rep < 8; rep ++){
            out();				// подача в линию нулевого бита
		    _delay_loop_1(5);	// ~ 5 mks
            inp();				// подача в линию единичного бита
		    _delay_loop_1(20);	// ~ 15 mks
            if (!(PIND & 0x04))
                byte_rom &= 0x7F;
            else
                byte_rom |= 0x80;
            if (rep < 7)
                byte_rom >>= 1;
		    _delay_loop_1(80);	// ~ 60 mks
        }
        data_ROM[a] = byte_rom;
    }
}

//*****************************************************************************************
// функция подсчета CRC
// для последовательности 0x01, 0x09, 0xC8, 0x26, 0x0B, 0x00, 0x00 сигнатура 0x0F
// и соответственно для 0x01, 0x09, 0xC8, 0x26, 0x0B, 0x00, 0x00, 0x0F сигнатура 0x00
// 0x01 - код семейсева ключа Touch Memory,
// 0x09, 0xC8, 0x26, 0x0B, 0x00, 0x00 - 48 битный серийный номер
// 0x0F - сигнатура
//                             0F		 01	 гравировка на
//                             00000B26C809  корпусе ключа

int calc_CRC(void)
{
    static unsigned char a, b, crc, byte_crc, temp_crc;

	crc = 0;
    for (b = 0; b < 8; b++){
        byte_crc = data_ROM[b];
        for (a = 0; a <8; a ++){
            temp_crc = byte_crc;
            temp_crc ^= crc;
            if (temp_crc & 0x01){
                crc ^= 0x18;
                crc >>= 1;
                crc |= 0x80;
            }
            else
                crc >>= 1;
            byte_crc >>= 1;
        }
    }
        return crc;
}

//*****************************************************************************************
// проверка ключей №1 и №2

void chec_data_TM(void)
{
    static unsigned char a;

	p_b  = &tm_manag[0];
//	read_FM25640(0, 0);
	read_TM_manag();
	p_b = &tm_oper[0];
//	read_FM25640(0, 10);
	read_TM_oper();

	chec_TM = 1;
    for (a = 0; a <8; a ++){
        if (data_ROM[a] != tm_manag[a])
            chec_TM = 0;
    }
	if (chec_TM == 1)
		return;

	chec_TM = 2;
	for (a = 0; a <8; a ++){
		if (data_ROM[a] != tm_oper[a])
			chec_TM = 0;
	}
}

//*****************************************************************************************
// функция проверяет наличие ключа Touch Memory на линии и читает его код спроверкой CRC

void TM_line(void)
{
	cli();

    inp();
    _delay_loop_2(200); // ~ 200 mks
    out();				// подача в линию нулевого бита
    _delay_loop_2(480); // ~ 480 mks
	inp();				// подача в линию единичного бита
	_delay_loop_1(20);	// ~ 15 mks
    if (!(PIND & 0x04))
        goto INI_2;
	_delay_loop_1(80);	// ~ 60 mks
    if (PIND & 0x04)
        goto INI_2;
    _delay_loop_2(200); // ~ 200 mks
    if (!(PIND & 0x04))
        goto INI_2;
	
    ROM_33();
    rd_data_ROM();
    if (calc_CRC())
        goto INI_2;

    chec_data_TM();

// 	// если коснулись ключом №1
//     if ((chec_TM == 1) && (menu_flag != KEY_MANAG)
// 		&& (menu_flag != KEY_OPER))
// 		{
//           clean_screen();
// 		  key_1();
// 		  alarm(1);
// 		  chec_TM = 0;
//           touch = 0;
//           menu_flag = NONE_MENU;
//           cursor_flag = 0;
//           p = 0;
//           delay_c(2);
//           clean_screen();
// 		  menu_simbol();
// 		  column_1 = 1;
// 		  string = 4;
//           goto INI_2;
// 	    }
// 	
// 	// если коснулись ключом №2
//     if ((chec_TM == 2) && (menu_flag != KEY_MANAG)
// 		&& (menu_flag != KEY_OPER))
// 		{
//           clean_screen();
// 		  key_2();
// 		  alarm(2);
//           chec_TM = 0;
//           touch = 0;
//           menu_flag = NONE_MENU;
//           cursor_flag = 0;
//           p = 0;
//           delay_c(2);
//           clean_screen();
// 		  menu_simbol();
// 		  column_1 = 1;
// 		  string = 4;
// 		}
INI_2:
    out();
	sei();
}

//*****************************************************************************************
void TM_write_manag(void)
{
//    wdt_reset();

    inp();				// подача в линию единичного бита
    _delay_loop_2(200); // ~ 200 mks
    out();				// подача в линию нулевого бита
    _delay_loop_2(480); // ~ 480 mks
	inp();				// подача в линию единичного бита
	_delay_loop_1(20);	// ~ 15 mks
    if (!(PIND & 0x04))
        goto INI_3;
	_delay_loop_1(80);	// ~ 60 mks
    if (PIND & 0x04)
        goto INI_3;
    _delay_loop_2(200); // ~ 200 mks
    if (!(PIND & 0x04))
        goto INI_3;
	
    ROM_33();
    rd_data_ROM();
    if (calc_CRC())
        goto INI_3;
	p_b = &data_ROM[0];
//	write_FM25640(0, 0);
	write_TM_manag();
	cli();
//	clean_screen();
//	write_end_manag();  //message succesful operation
//	alarm(1);
	sei();

INI_3:
    out();
}

//*****************************************************************************************
void TM_write_oper(void)
{
    wdt_reset();

    inp();				// подача в линию единичного бита
    _delay_loop_2(200); // ~ 200 mks
    out();				// подача в линию нулевого бита
    _delay_loop_2(480); // ~ 480 mks
	inp();				// подача в линию единичного бита
	_delay_loop_1(20);	// ~ 15 mks
    if (!(PIND & 0x04))
        goto INI_3;
	_delay_loop_1(80);	// ~ 60 mks
    if (PIND & 0x04)
        goto INI_3;
    _delay_loop_2(200); // ~ 200 mks
    if (!(PIND & 0x04))
        goto INI_3;
	
    ROM_33();
    rd_data_ROM();
    if (calc_CRC())
        goto INI_3;
	p_b = &data_ROM[0];
//	write_FM25640(0, 10);
	write_TM_oper();
	cli();
//	clean_screen();
//	write_end_oper(); //message succesful operation
//	alarm(2);
	sei();

INI_3:
    out();
}
//*****************************************************************************************

