/*
 * LCD_ST7565_Mega64_5.c
 *
 * Created: 03.02.2015 14:55:33
 * Author: Sergey
 * Board - ATMEGABOARD Atmega64 with EXMEM, AT45DB321D
 * Modernized library uart.c, changed function uart1_putc, 
 * returned it to its original form, at the same time and uncomment uart1_puts uart1_puts_p. Instead, use a new function uart1_putc usart1_put_char.
 * Работоспособность сохранена.
 * Implementation DataFlash AT45DB321D  library - done
 * Implementation FatFS от Chan - done (FatFs - FAT file system module  R0.05 (C)ChaN, 2007)
 * Trying to put the files in the different directories of the project - successfully
 * Implemetation library I2C clock DS1307 - done
 * Added function to read I2C temperature sensor LM75A - done
 * Added function to read I2C EEPROM AT24C256 - done
 * Display  ST7565 library handed in directory "lcd"
 * Added setting time
 * Added the function of the original initialization time DS1307
 * Release Flash=19 300b; RAM=2000b
 * 16.02.2015 Added library TUI_lib_0_03 Small-LCD Text User Interface author: ARV
 * Slightly brushed display library ST7565, Text output is implemented on the basis of libraries Wiselord with downloadable and scalable fonts.
 * In the library TWI added functions to read and write arrays external EEPROM.
 * 16.02.2015 
 * Added displaying graphical splash at the start
 * Implemetation Touch Memory library. Author - S.Surov, Radio magazine 2009 №1,2 Master_controller - successfully implemented, but not tested
 * Added the function Buzzer without generator on port PG3
 * Unwired PB4  port from RDY/BUSY AT45DB321D. RDY/BUSY read from register AT45DB321D.
 * Read / write arrays of EEPROM AT24C256 is implemented through a pointer - done.
 * ...need to do exactly the same job through the structure.
 * It is based on version LCD ST7565 Mega64_38. Modified driver dataflesh from LUFA library with FatFS by Chan R0.09a
 * A sort of a downgrade. But LUFA driver have a read/write buffer 
 * FatFS library updated to version R0.11 - done 
 * Added the function of data logger: File type: 19022015.CSV, every 5 minutes written string type: 19.2.2015 16:38:35, 23.7 Degrees.
 * ФFunction prototype by LUFA
 * Trying to place a large array in external EEPROM.
 * 09.03.2015 Screwed up function scrolling text on display. Works very crooked 
  
  ST7565 pinout:
    CS   PB0
    A0   PE3- wired on PB4
    A0   PB4	
    RES  PE2
    SID  PB2
    SCLK PB1
 
 AT45DB321D pinout:
    SCK  PB1
    SI   PB2
    SO   PB3
    CS   PB5
 */ 



//Atmega64 - microcontroller

/*The frequency of the clock generator */
#ifndef F_CPU
#define F_CPU  18432000UL
#endif

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>
#include "drivers/twim.h"
#include "lcd/lcd.h"
#include "lcd/st7565.h"
#include "lcd/st7565-graphics.h"
#include "lcd/st7565-driver.h"
#include "drivers/uart.h"
#include "drivers/spi.h"
#include "LUFA/DataflashManager.h"
#include "file.system/rtc.h"
#include "lcd/wiselord/fonts.h" 
#include "TUI/sedit.h"
#include "TUI/buttons.h"
#include "TUI/mmenu.h"
#include "TM/functions_TM.h"
#include "lcd/logo/addidas_logo.h"
//#include "lcd/logo/nike.h"
//#include "lcd/logo/picture_data.h"
//#include "lcd/logo/sailor_man.h"
//#include "lcd/logo/Bitmap.h"

#define STR_BUFSIZE		   16

#define LCD_LINES           8     /**< number of visible lines of the display */   //For TUI
#define LCD_DISP_LENGTH    20     /**< visibles characters per line of the*/



// set up external SRAM prior to anything else to make sure malloc() has access to it
void EnableExternalSRAM (void) __attribute__ ((naked)) __attribute__ ((section (".init1")));
//__attribute__ ((section (".init1"))) EnableExternalSRAM(void)
void EnableExternalSRAM(void)
{
     MCUCR |= (1<<SRE);	 
//     MCUCR = (1<<SRE)|(0<<SRW10);   /* External memory interface enable */
     XMCRA &= ~(1<<SRL2)|(1<<SRL1)|(1<<SRL0)|(1<<SRW01)|(1<<SRW00)|(1<<SRW11);
     /*
      * Lower sector = N/A
      * Upper sector = 0x1100 - 0xFFFF
      * No wait cycles for upper sector
      */
     XMCRB &= ~(1<<XMBK)|(1<<XMM2)|(1<<XMM1)|(1<<XMM0);
     /* External Memory Bus-keeper Disabled;
      * External Memory High Mask - no released port pins*/
}






#define DS1307_ADR    104
uint8_t	LM75_buff_1;
uint8_t	LM75_buff_2;
//float LM75_buff;
uint8_t EEPROM_AT24_buff_read[64];
uint8_t EEPROM_AT24_buff_write[64];

//void     *p_m_4;            // Промежуточный указатель начального адреса массива logo
//uint8_t *p_b_4;		    // Указатель адреса отдельного байта массива logo


uint8_t *p_b_write;		// указатель на массив EEPROM

uint8_t *p_b_read;		// указатель на массив EEPROM

uint8_t *p_b_4;		    // указатель на массив 


uint16_t pos_in_string; //Позиция в строке, с которой начинается отображение (позиция пикселя, а не символа)

char *message = " ******Welcome******";
char *message1 = " Very long long super long string...";

//uint8_t logo [128];
//p_b_4  = &logo[0];


#define FALSE	0
#define TRUE	1
 
#define DISK_SEC_SIZE	512	/* sector area size */

 

//uint8_t media;
//unsigned int dannie, i, adres;
//uint8_t masiv [4] = {100,150,200,150};

// uint8_t  save_8  EEMEM = 2;                    //internal EEPROM_Example
// uint16_t save_16 EEMEM = 234;
// uint32_t save_32 EEMEM = 10000;
// unsigned char save[4] EEMEM;


uint8_t *mem;                                     //EXMEM_test Example_1

//int massiv[100][100];                          // !!! MEMORY EATER !!! EXMEM_test Example_2

volatile uint8_t K;
volatile int ADCvalue, ADCvalue1, ADCvalue2;
volatile int adc_read1, adc_read2;
unsigned char pause, read, adc_read1_1, adc_read1_2, adc_read2_1, adc_read2_2;
volatile int datalog;

/* System time for FatFs */
volatile unsigned long g_SysTIME;

volatile unsigned long start;



// DWORD get_fattime (void)                      //RTC заглушка
// {
// 	/* Pack date and time into a DWORD variable */
// 	return	  ((DWORD)(2015 - 1980) << 25)
// 	| ((DWORD)2 << 21)
// 	| ((DWORD)15 << 16)
// 	| ((DWORD)16 << 11)
// 	| ((DWORD)53 << 5)
// 	| ((DWORD)6>> 1);
// }


RTC rtc;

BYTE RtcOk;					/* RTC is available */


DWORD get_fattime (void)
{
	RTC rtc;

	if (!RtcOk) return 0;

	/* Get local time */
	rtc_gettime(&rtc);

	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)(rtc.year - 1980) << 25)
	        | ((DWORD)rtc.month << 21)
	        | ((DWORD)rtc.mday << 16)
	        | ((DWORD)rtc.hour << 11)
	        | ((DWORD)rtc.min << 5)
	        | ((DWORD)rtc.sec >> 1);
}

unsigned int NUM;	            // variable for HeartBeat LED
unsigned int c;

 //For UART
 // Define size of memory block
 #define BUFFER_SIZE         512        //исходный код

/* 9600 baud */                         //Peter Fleury
 #define UART_BAUD_RATE      115200

 
// Macro to send strings stored in program memory space
#define PRINTF(format, ...) printf_P(PSTR(format), ## __VA_ARGS__)
	 
int usart1_put_char(unsigned char data, FILE *stream);                               //My	 
	 
// Create PRINTF Stream structure
FILE printf_stream = FDEV_SETUP_STREAM (usart1_put_char, NULL, _FDEV_SETUP_WRITE);   //исходный код_	



/*****************************************************************************
file system object Protoss
******************************************************************************/

/** FAT Fs structure to hold the internal state of the FAT driver for the Dataflash contents. */
FATFS fs;
FATFS *p_fs = &fs;
char buff[128];	// Read-write buffer
	
/** FAT Fs structure to hold a FAT file handle for the log data write destination. */
FIL fil_obj;

unsigned int ByteWrite = 0;
char* var="Hello! ";



FRESULT scan_files (char* path)	                           //Scan files and directories on path  avrlab.com
{
	FRESULT res;
	FILINFO fno;
	DIR dir;
//	int i;
	char *fn;
	
	res = f_opendir(&dir, path);
	if (res == FR_OK) {
//		i = strlen(path);
		for (;;)
		{
			res = f_readdir(&dir, &fno);	              //Read dir items
			if (res != FR_OK || fno.fname[0] == 0) break;
			if (fno.fname[0] == '.') continue;
			fn = fno.fname;
			if (fno.fattrib & AM_DIR) {	                  // Check item type, DIR or FILE
				PRINTF ("DIR: ");
				uart1_puts (fn);
				PRINTF ("\r\n");
			}
			else {
				PRINTF ("FILE: ");
				uart1_puts (fn);
				PRINTF ("\r\n");
			}
		}
	}
	return res;
}





       //TUI Start
#define menu_one 1	// код выбора пункта меню "однострочное меню"
#define menu_multi 2	// код выбора пункта меню "многострочное меню"
#define menu_test 3	// код выбора пункта меню "тест"
#define menu_tes 10	// код выбора пункта меню "тест1"

PROGMEM const char mmm1[] = "ONE LINE";	// строка пункта меню "однострочное меню"
PROGMEM const char mmm2[] = "MULTI LINE";	// строка пункта меню "многострочное меню"

PROGMEM const char mm1[] = "MENU MODE";	// строка пункта меню "режим меню"
PROGMEM const char mm2[] = "EDIT";		// строка пункта меню "редактирование"
PROGMEM const char mm3[] = "LEDS";		// строка пункта меню "светодиоды"
PROGMEM const char mm4[] = "TEST";		// строка пункта меню "тест"
PROGMEM const char mm5[] = "TEST1";		// строка пункта меню "тест"

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
PROGMEM t_menu const m_mode[] = {
	item_sim(mmm1,menu_one),
	item_sim(mmm2,menu_multi)
};

// массив меню редактирования
PROGMEM t_menu const m_edit[] = {
	item_opt(ed_uint, &number1, 0),
	item_opt(ed_int, &number2, 0),
	item_opt(ed_scal1, &number3, 0),
	item_opt(ed_scal2, &number4, 0)
};

// массив меню управления светодиодами
// PROGMEM t_menu const m_flags[] = {
// 	item_opt(port_val, &PORTF, (0x01)<<4),
// 	item_opt(port_val, &PORTF, (0x02)<<4),
// 	item_opt(port_val, &PORTF, (0x04)<<4),
// 	item_opt(port_val, &PORTF, (0x08)<<4)
// };

// PROGMEM t_menu const m_flags[] = {
// 	item_opt(port_val, &PORTF, 0x01),
// 	item_opt(port_val, &PORTF, 0x02),
// 	item_opt(port_val, &PORTF, 0x04),
// 	item_opt(port_val, &PORTF, 0x08)
// };

PROGMEM t_menu const m_flags[] = {
	item_opt(port_val, &PORTF, 0x10),
	item_opt(port_val, &PORTF, 0x20),
	item_opt(port_val, &PORTF, 0x40),
	item_opt(port_val, &PORTF, 0x80)
};


// массив основного меню
PROGMEM t_menu const m_menu[] = {
	item_sub(mm1,m_mode),	// подменю режимов отображения меню
	item_sub(mm2,m_edit),	// подменю редакторов
	item_sub(mm3,m_flags),	// подменю флажков
	item_sim(mm4,menu_test),			// простой пункт - небольшая анимация
	item_sim(mm5,menu_tes)			// простой пункт - небольшая анимация
};





// редактирование уровней на порту С - управление светодиодами
static pchar port_val(void* data, uint8_t v, uint8_t ch){
	pchar	ts;
	
	if(ch){ // параметр ch не равный нулю указывает, что надо изменить состояние
		    // при этом указатель data считаем указателем на байт
		*((uint8_t*)data) ^= v;
	}
	
	// готовим текст в зависимости от того, какой светодиод обрабатывается
	switch(v){
		case 16: ts = PSTR("LED 1"); break;
		case 32: ts = PSTR("LED 2"); break;
		case 64: ts = PSTR("LED 3"); break;
		case 128: ts = PSTR("LED 4"); break;
	}
	// готовим строку для возврата
	strcpy_P(tr,ts);				// копируем первую часть строки
	if(*((uint8_t*)data) & v)		// проверяем состояние флага и
	// прибавляем соответствующий текст
	strcat_P(tr,PSTR(" ON"));
	else
	strcat_P(tr,PSTR(" OFF"));
	// возвращаем полученную строку
	disp_send_frame();	
	return tr;
}

// редактирование беззнакового целого числа
static pchar ed_uint(void* data, uint8_t v, uint8_t ch){
	char ts[6];
	if(ch){ // если надо изменить - просто вызываем нужный редактор
		disp_clear();
		*(uint16_t*)data = edit_uint(0,PSTR("Unsigned: "),*(uint16_t*)data,50,20000);
	}
	// подготавливаем строку, которую видно в меню
	strcpy_P(tr,PSTR("UINT EDIT "));
	ltoa((uint16_t)(*(uint16_t*)data),ts,10);
	strcat(tr,ts);
    disp_send_frame();	
	return tr;
}

// редактирование целого со знаком - аналогично предыдущему
static pchar ed_int(void* data, uint8_t v, uint8_t ch){
	char ts[7];
	if(ch){
		disp_clear();
		*(int16_t*)data = edit_sint(0,PSTR("Signed: "),*(int16_t*)data,-1500,20000);
	}
	strcpy_P(tr,PSTR("INT EDIT "));
	itoa(*(uint16_t*)data,ts,10);
	strcat(tr,ts);
	disp_send_frame();	
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
		disp_clear();
		gdSetXY(0,0);
		lcd_puts_p(PSTR("SCALE 2"));
		disp_send_frame();	
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
		    disp_send_frame();				
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
		//		lcd_init(LCD_DISP_OFF);
		init_h_scale_sym();
		//		lcd_init(LCD_DISP_ON);
		disp_clear();
		gdSetXY(0,0);
		lcd_puts_p(PSTR("SCALE 1"));
		disp_send_frame();	
		do{
			//			se_percent(0,1,tmp);
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
		 disp_send_frame();	
		}while(1);
	}
	exit:
	strcpy_P(tr,PSTR("SCALE HBAR "));
	itoa(tmp,ts,10);
	strcat(tr,ts);
	return tr;
}
       //TUI Stop




       //LUFA Data Logger

/** Opens the log file on the Dataflash's FAT formatted partition according to the current date */
void OpenLogFile(void)
{
	char LogFileName[12];

	/* Get the current date for the filename as "DDMMYY.csv" */
//	TimeDate_t CurrentTimeDate;
//	RTC_GetTimeDate(&CurrentTimeDate);
	sprintf(LogFileName, "%02d%02d%02d.csv", rtc.mday, rtc.month, rtc.year);

	/* Mount the storage device, open the file */
    f_mount(&fs, "", 0);	
	f_open(&fil_obj, LogFileName, FA_OPEN_ALWAYS | FA_WRITE);
	f_lseek(&fil_obj, fil_obj.fsize);
}

/** Closes the open data log file on the Dataflash's FAT formatted partition */
void CloseLogFile(void)
{
	/* Sync any data waiting to be written, unmount the storage device */
	f_sync(&fil_obj);
	f_close(&fil_obj);
}


void LogFileSave (void)
	{
	  if (datalog > 27000)          //5400 = 1 munute???
		{		
//		TimeDate_t CurrentTimeDate;
//		RTC_GetTimeDate(&CurrentTimeDate);

		char     LineBuffer[100];
		uint16_t BytesWritten;

		BytesWritten = sprintf(LineBuffer, "%02d/%02d/20%02d, %02d:%02d:%02d, %d Degrees\r\n",
		                       rtc.mday, rtc.month, rtc.year,
		                       rtc.hour, rtc.min, rtc.sec,
		                       LM75_buff_1);

		f_write(&fil_obj, LineBuffer, BytesWritten, &BytesWritten);
		f_sync(&fil_obj);
		
		PRINTF("In Log: %u.%u.%u %02u:%02u:%02u, %d.%d Degrees\r", rtc.mday, rtc.month, rtc.year, rtc.hour, rtc.min, rtc.sec, LM75_buff_1,LM75_buff_2);
		datalog = 0;		
		}
	}

       //LUFA Data Logger


void ADC_init()
{                                                                //Настройка АЦП	
	ADCSRA |=(1<<ADEN);                                          //Задействовать АЦП
//	ADCSRA |=(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);                   //Делитель 64 - 125 кГц  при частоте процессора 8 MHz
	ADCSRA |=(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);                   //Делитель 128 - 125 кГц  при частоте процессора 16 MHz
    ADCSRB &= ~(1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0);                 // Set free running mode
	ADMUX |= (1<<REFS1)|(1<<REFS0);                              //Опорное напряжение 2.56 В внутреннее
//	ADMUX |= (1<<ADLAR);                                                //С помощью этого бита делаем левое выравнивание результата и читаем результат из ADCH (АЦП 8бит)
	ADMUX &= ~(1<<ADLAR);                                        //С помощью этого бита делаем правое выравнивание результата (АЦП 10бит)	
	ADMUX &= ~(1<<MUX4)|(1<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0); //Выбор вывода

    ADCSRA |= (1 << ADIE);                                       // Enable Interrupts	
	ADCSRA |= (1 << ADIF);                                       // Interrupt Flag - write 1 to clear!
    ADCSRA |= (1 << ADATE);                                      // ADATE = Auto Trigger Enable
    ADCSRA |= (1 << ADSC);                                       // Start the ADC conversion				
}

void Timer_init()
{
	//Настройка 8-разрядного Таймера TC0
	OCR0=200; // регистр   порога   сравнения - задаем вершину счета счетчика,
	// если значение счетчика (TCNT0) совпадает со значением регистра OCR0 то счетчик обнуляется (TCNT0=0)
	TCCR0 |= (0<<FOC0)|(0<<WGM00)|(0<<COM01)|(0<<COM00)|(1<<WGM01)|(1<<CS02)|(1<<CS01)|(1<<CS00); //сброс счетчика при совпадении,
	//функция обычного порта ввода-вывода OC0 отключен.           |<-   предделитель /1024   ->|
	TIMSK|=(1<<OCIE0);	//Разрешение прерывания по результату сравнения таймера-счетчика 0
	
	 //Настройка 8-разрядного Таймера TC2
	TCCR2 = ((1 << CS22)|(0 << CS21)|(1 << CS20)|	/* clkT0S/1024 */
	(1 << WGM21)|(0 << WGM20)|		                /* Waveform Generation Mode - CTC */
	(0 << COM21)|(0 << COM20));	                    /* Nothing Toggle on compare match */
	OCR2 = 180; // регистр порога сравнения-задаем вершину счета счетчика TC2 (setup output compare unit)
	TIMSK |= (1 << OCIE2); //Разрешение прерывания по результату сравнения таймера-счетчика TC2
	
/*****************************************************************************
System timer (TC0) configuration !!! Важно !!! с файла Sys.TIME.cfg Protoss
******************************************************************************/
//Расчет системных тиков в зависимости от частоты системного генератора 
/* TC0 compare value for create periodic interrupt 10 mc - 100Hz*/
//#define TC0_INCLUDE_32768_OSC		1	/* include external quartz 32.768 Hz */
//#if (1 == TC0_INCLUDE_32768_OSC)
//	#define TC0_COMPARE_CONST		2	/* for external 32.768 osc
//										32768 / 128(prescaler) = 256;
//										(14400 - 1)/143 = 100 tick per secs */
//#else
//	#define TC0_COMPARE_CONST		143 /* for CPU_CLOCK = 14.745600 hz
//										CPU_CLOCK / 1024(prescaler) = 14400;
//										(14400 - 1)/143 = 100 tick per secs */
//#endif	/* #if (1 == TC0_INCLUDE_32768_QRZ) */	

//	                              		143 /* for CPU_CLOCK = 18.432000 hz
//										CPU_CLOCK / 1024(prescaler) = 18000;
//										(18000 - 1)/180 = 100 tick per secs */    
}


									
void mig_mig()
{
	if ((PIND & (1 << PD6))==0)  // Если светодиод потушен
	{
    	if (pause > 180)
     	{
 		 			PORTD |=  (1 << PD6); // Лог. 1 на выводе PD6 - зажигаем светодиод
					pause = 0;  
    	}
	}
	else                         // Если светодиод горит
	{
		if (pause > 20)
		{
 		 			PORTD &= ~(1 << PD6); // Лог. 0 на выводе PD6 - тушим светодиод	
					pause = 0;
		}
	}	
}


void read_parametr()
{
		if (read > 90)
		{
			rtc_gettime(&rtc);
			LM75_Read ();
			adc_read1 = ((ADCvalue1*2.56*(5/2.56))/1023)*10;
			adc_read2 = ((ADCvalue2*2.56*(5/2.56))/1023)*10;
			adc_read1_1 = adc_read1 / 10;
			adc_read1_2 = adc_read1 % 10;
			adc_read2_1 = adc_read2 / 10;
			adc_read2_2 = adc_read2 % 10;			 			
			read = 0;
		}
}


void EXMEM_example ()
{
	  uint16_t index;
			 
      PRINTF("\nExternal Memory Example\n");
      
      // Allocate external memory
      mem = malloc(BUFFER_SIZE);

      PRINTF("%d Byte buffer (starting at 0x%04X) filled with 0's:\n",BUFFER_SIZE,mem);

      // Fill memory with 0's
      for(index = 0; index < BUFFER_SIZE; index++)
      {
	      mem[index] = 0x00;
      }
      
      for(index = 0; index < BUFFER_SIZE; index++)
      {
	      PRINTF("%02X ",mem[index]);
	      if((index&0x0F) == 0x0F)
	      {
		      PRINTF("\n");
	      }
      }

      PRINTF("\n%d Byte buffer (starting at 0x%04X) filled with incrementing numbers:\n",
      BUFFER_SIZE, mem);

      // Fill memory with incrementing numbers
      for(index = 0; index < BUFFER_SIZE; index++)
      {
	      mem[index] = index;
      }
      
      for(index = 0; index < BUFFER_SIZE; index++)
      {
	      PRINTF("%02X ", mem[index]);
	      if((index&0x0F) == 0x0F)
	      {
		      PRINTF("\n");
	      }
      }
      PRINTF("\n");
}



/*            buzzer(frequency, duration);
Sound a tone, this time without interrupts. CPU intensive!
call arguments: frequency is in Hz, usable range 100-10,000 Hz, duration is in milliseconds
This routine uses _delay_loop_2() to generate timing (4 cpu cycles per loop)
For correct operation, be sure to define F_CPU in Hz! */

void buzzer(unsigned int frequency, unsigned int duration)
{
	unsigned int wait_units, loops, i;

//	DDRG  |=  (1<<DDG4);							        //make PG4 output
	wait_units = F_CPU/(8UL * (long int) frequency);		//half of the tone period, in delay loop units
	loops = duration*F_CPU/(8000UL*wait_units);			    //number of loops to execute
	for (i=1; i<=loops; i++) {					            //sound the buzzer
		PORTG |=  (1<<PG3);						            //on
		_delay_loop_2(wait_units);				            //wait 1/2 tone period
		PORTG &= ~(1<<PG3);						            //off
		_delay_loop_2(wait_units);				            //wait 1/2 tone period
	}
}




//#define NULL_PIXEL 2
#define SCREEN_WIDTH 190

int string_width (char s[]) 
{
	int i;
	
	i = 0;
	while (s[i] != '\0')
	++i;
	
	return i*6;
}

	
	
void scrolling(char *Buffer)
{
//	char tmp[50];                    //массив для временного хранения выводимой строки
	uint16_t bufferlen;
	//uint16_t len;
   //	uint16_t pos = *ScrlPos;
//uint16_t pos = pos_in_string;	
//	uint8_t index = 0, offset = 0;

	bufferlen = string_width(Buffer); // длинна попиксельно строки источника
 // 	char tmp[bufferlen]; 
	
//	tmp[0] = 0;

//	if(bufferlen > pos) //если попиксельная длинна строки больше позиционного номера
//	{
  // 		do
  // 		{
  // 			tmp[offset+1] = Buffer[offset]; //переписываем строку в массив для временного хранения выводимой строки
  // 			offset++;
  // 		}while((string_width(tmp) - 1) < pos); // пока попиксельная длинна массива временного хранения строки меньше позиционного номера
//		while(string_width(tmp) < pos) // пока попиксельная длинна массива временного хранения строки меньше позиционного номера
//		{
//			tmp[offset] = Buffer[offset]; //переписываем строку в массив для временного хранения выводимой строки
//			offset++;
//		}		
		
			
//		offset--;
  //		tmp[offset] = 0;
  //		pos = pos_in_string - string_width(tmp);
//		memset(tmp, 0, sizeof(tmp));
// 		while(string_width(tmp) < (SCREEN_WIDTH + pos))
// 		{
// 			if((index + offset) >= strlen(Buffer))		
// 			break;
// 			tmp[index] = Buffer[index + offset];
// 			index++;
// 		}
		//tmp[index] = 0;
//	}
//	  else
//	      {
		  // pos -= bufferlen;
//		   pos = pos - bufferlen;
//		   memset(tmp, 0, sizeof(tmp));
//	      }
	
//	len = string_width(tmp); // длинна попиксельно строки временного хранения
	
//	if(len < (SCREEN_WIDTH + pos))
//	{
	//	tmp[index++] = (char)'-';
	//	tmp[index++] = (char)'-';
	//	tmp[index++] = (char)'-';
//		tmp[index] = 0;

   //		len = string_width(tmp, NULL_PIXEL);
//   		len = string_width(tmp);
				
		
//		if(len < (SCREEN_WIDTH + pos))
//		offset = 0;

// 		while(string_width(tmp) < (SCREEN_WIDTH + pos))
// 		{
// 			tmp[index] = Buffer[offset];
// 			index++;
// 			offset++;							
// 		}
//		tmp[index] = 0;		
//	}

               //	string_print(tmp, pos);
//    gdSetXY(pos, 2);
    gdSetXY(pos_in_string, 2);	
              //    gdSetXY(2, 2);	
//    gdWriteString(tmp);	
    gdWriteString(Buffer);
	
// 	gdSetXY(60,10);
// 	gdWriteString(mkNumString(bufferlen, 3, ' ', 10));	 	
// 	gdSetXY(80,10);
// 	gdWriteString(mkNumString(offset, 3, ' ', 10));
// 	gdSetXY(100,10);
// 	gdWriteString(mkNumString(pos, 3, ' ', 10));	
// 	gdSetXY(80,18);
// 	gdWriteString(mkNumString(index, 3, ' ', 10));	
// 	gdSetXY(100,18);
// 	gdWriteString(mkNumString(pos_in_string, 3, ' ', 10));	
	
              //	*(ScrlPos)++;
	pos_in_string++;

   //	if(*ScrlPos > (bufferlen + string_width(" - ")))
   //	if(pos_in_string > (bufferlen + string_width(" - ")))
	if(pos_in_string > bufferlen+36)			
   //	*ScrlPos = 0;
	pos_in_string = 0;
}	



	

int main( void )
{
	
    /*  Initialize UART library, pass baudrate and AVR cpu clock
     *  with the macro UART_BAUD_SELECT() (normal speed mode )
     *  or UART_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)*/
		
      uart1_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );  //Peter Fleury
	  
	  // Route stdout stream to printf_stream
	  stdout = &printf_stream;               //исходный код
	  
      ADC_init();
	  Timer_init();                        //initialize system timer
	  TWI_MasterInit(400);	               //Инициализируем модуль TWI с частотой обмена 100 кГц 
	  SPI_Init();
	  Dataflash_Init();
	  sei();
	  lcd_init();       	               //Инициализируем дисплей


	  DDRE  &= ~(1<<DDE4)|(1<<DDE5)|(1<<DDE6)|(1<<DDE7);   //setup on PORTE (Enable as input for buttons)
	  PORTE |=  (1<<PE4)|(1<<PE5)|(1<<PE6)|(1<<PE7);       //setup on PORTE (Enable pull UPs for buttons)	  	  
	  DDRD  |=  (1<<DDD6)|(1<<DDD7);                       //setup on PORTD (Enable as output for Heart Beat & Info LEDs)	
	  DDRF  |=  (1<<DDF4)|(1<<DDF5)|(1<<DDF6)|(1<<DDF7);   //setup on PORTF (Enable as output for LEDs)		  
//	  PORTF = DDRF;
	  PORTG &= ~(1<<PG3);                                  //setup on PORTG (Disable pull UPs)
	  DDRG  |=  (1<<DDG3);                                 //setup on PORTG (Enable as output for Buzzer)
	  
       //TUI Start
	char s[16];
	EVENT ev = EV_NONE;
//	uint8_t mode=1;
	uint8_t menu_r;
       //TUI Stop


//uint16_t pos_in_string; //бегущая строка
//uint8_t pos_in_string; //бегущая строка

    disp_clear();


//   uint8_t logo [1024];
//   p_b_write  = &logo[0x3C0];


p_b_read  = &EEPROM_AT24_buff_read[0];
//p_b_4  = &logo[0];




//      for(uint8_t i=0; i < 16; i++)
//      {
//       EEReadArray(64, i*64, EEPROM_AT24_buff_read[i*64]);
//         //     EEReadArray(64, i*64, p_b_read + i * 64);
// 
//       _delay_ms(200);
//      }




    gdWriteIcon(addidas_logo);
//	gdWriteIcon_EEPROM(logo);
//	gdWriteIcon_EEPROM(EEPROM_AT24_buff_read);
//	gdWriteIcon_EEPROM(logo);
	disp_send_frame();
	_delay_ms(5000);
    disp_clear();
	
	buzzer(200,1000); //beep @ 200 Hz for 1 second	
	
	
	p_b  = &tm_manag[0];
	//read_FM25640(0, 0);
	read_TM_manag();
	p_b = &tm_oper[0];
	//read_FM25640(0, 10);
	read_TM_oper();
	
	


	if (rtc_gettime(&rtc))                  //Initialize RTC
	{
		RtcOk = 1;		
		PRINTF("\n \nCurrent time: %u.%u.%u %02u:%02u:%02u\r\n", rtc.mday, rtc.month, rtc.year, rtc.hour, rtc.min, rtc.sec);
	} 
	else 
		{
		PRINTF("RTC is not available.\n");
     	}
		 
//    first_init_DS1307 (); // Расскоментировать в случае остановки часов!!!


		 
//	     PRINTF("\n%d Byte buffer (starting at 0x%04X) filled with incrementing numbers:\n",
//         BUFFER_SIZE, massiv);


/* initialize file system */
FRESULT res = f_mount(&fs, "", 0);
PRINTF("f_mount return: ");
f_PrintError(res);

unsigned long cls = 0;
DWORD  fre_sect, tot_sect;

res = f_getfree("", &cls, &p_fs);
PRINTF("f_getfree return: ");
f_PrintError(res);

if(FR_NO_FILESYSTEM == res)
{
	res = f_mkfs(0, 0, 4 * DISK_SEC_SIZE);
	PRINTF("f_mkfs return: ");
	f_PrintError(res);
	
    res = f_getfree("", &cls, &p_fs);
	PRINTF("f_getfree return: ");
    f_PrintError(res);	
}

if(res)
{
	PRINTF("main: File System inited failed!\r\n");
	return FALSE;
}
PRINTF("main: File System inited OK!\r\n");


/* Get total sectors and free sectors */
tot_sect = (fs.n_fatent - 2) * fs.csize;
fre_sect = cls * fs.csize;

/* Print the free space (assuming 512 bytes/sector) */
PRINTF("%10lu KiB total drive space.\n%10lu KiB available.\r\n",
tot_sect / 2, fre_sect / 2);


/*create new file foo.txt*/
// res = f_open(&fil_obj, "foo.txt", FA_CREATE_NEW);
// PRINTF("f_open return: ");
// if(res==0) PRINTF ("OK\r\n");
// else PRINTF("FAIL\r\n");

/*open*/
res = f_open(&fil_obj, "foo.txt", FA_WRITE);
PRINTF("f_open return: ");
if(res==0) PRINTF ("OK\r\n");
else PRINTF("FAIL\r\n");

/*write*/
res = f_lseek(&fil_obj,fil_obj.fsize);                              //смещаем указатель на кол-во байт, равное размеру файла. Т.е. переходим в конец файла.
PRINTF("f_lseek return: %X, size of file: %d \r\n", res,(int)fil_obj.fsize);

res = f_write(&fil_obj,var,strlen(var),&ByteWrite);                 //записываем в конец файла строку var
PRINTF("f_write return: ");
if(res==0) PRINTF ("OK\r\n");
else PRINTF("FAIL\r\n");

PRINTF("byte wrote: %d \r\n", ByteWrite);                          //выводим сколько байтов удалось записать

/*close*/
f_close(&fil_obj);                                                 //закрываем файл*/


res=f_open(&fil_obj, "foo.txt",FA_READ);	                       //Open newfile for reading
PRINTF ("open foo.txt ");
if(res==0) PRINTF ("OK\r\n");
else PRINTF("FAIL\r\n");

PRINTF ("data in foo.txt:\r\n ");
uart1_puts (f_gets (buff,128,&fil_obj));                          //Read data from newfile

res=f_close(&fil_obj);
PRINTF ("\n closing foo.txt ");	                                  //Close newfile
if(res==0) PRINTF ("OK\r\n");
else PRINTF ("FAIL\r\n");

PRINTF("scaning drive:\r\n");
scan_files ("0:/");                                                //Scan drive
PRINTF("scaning ended\r\n");

res=f_mount(NULL, "", 0);	                                       //Unmount Fat Fs
PRINTF ("unmounting FAT ");
if(res==0) PRINTF ("OK\r\n");
else PRINTF("FAIL\r\n");

// На будущее эти переменные пригодятся для изменяемого периода записи лога !!!
/* Fetch logging interval from EEPROM */
//LoggingInterval500MS_SRAM = eeprom_read_byte(&LoggingInterval500MS_EEPROM);

/* Check if the logging interval is invalid (0xFF) indicating that the EEPROM is blank */
//if (LoggingInterval500MS_SRAM == 0xFF)
//LoggingInterval500MS_SRAM = DEFAULT_LOG_INTERVAL;


/* Mount and open the log file on the Dataflash FAT partition */
OpenLogFile();




//   EEWriteByte(0x00,0x00);
//   _delay_ms(20);
//   EEWriteByte(0x01,0x01);
//   _delay_ms(20);
//   EEWriteByte(0x02,0x02);
//   _delay_ms(20);
//   EEWriteByte(0x03,0x03);
//   _delay_ms(20);
//   EEWriteByte(0x04,0x04);
//   _delay_ms(20);
//   EEWriteByte(0x05,0x05);
//   _delay_ms(20);
//   EEWriteByte(0x06,0x06);
//   _delay_ms(20);
//   EEWriteByte(0x07,0x07);
//   _delay_ms(20);
//    EEWriteByte(0x08,0x08);
//    _delay_ms(20);

// EEReadByte(0x00);
// EEReadByte(0x01);
// EEReadByte(0x02);
// EEReadByte(0x03);
// EEReadByte(0x04);
// EEReadByte(0x05);
// EEReadByte(0x06);
// EEReadByte(0x07);
// EEReadByte(0x08);

//unsigned char EEPROM_AT24_buff_write [32] = {0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7};

// EEPROM_AT24_buff_1[0] = 0xE0;
// EEPROM_AT24_buff_1[1] = 0xE1;
// EEPROM_AT24_buff_1[2] = 0xE2;
// EEPROM_AT24_buff_1[3] = 0xE3;
// EEPROM_AT24_buff_1[4] = 0xE4;
// EEPROM_AT24_buff_1[5] = 0xE5;
// EEPROM_AT24_buff_1[6] = 0xE6;
// EEPROM_AT24_buff_1[7] = 0xE7;
// EEPROM_AT24_buff_1[8] = 0xE8;
// 
//p_b_write  = &EEPROM_AT24_buff_write[0];

//EEWriteArray (32, 0x00, *p_b_2);

// _delay_ms(1000);
// EEWriteArray (64, 0x40, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x80, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0xC0, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x100, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x140, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x180, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x1C0, *p_b_2);
// _delay_ms(1000);
//EEWriteArray (64, 0x200, *p_b_write); 
//_delay_ms(1000);
// EEWriteArray (64, 0x240, *p_b_write);
// _delay_ms(1000);
// EEWriteArray (64, 0x280, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x2C0, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x300, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x340, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x380, *p_b_2);
// _delay_ms(1000);
// EEWriteArray (64, 0x3C0, *p_b_2);
// _delay_ms(1000);

// p_b_write  = &EEPROM_AT24_buff_1[0];
// 
// 
// for (uint8_t i = 0; i < 16; i++)
// {
// EEWriteArray (32, i*32, p_b_4 + i * 32);
// _delay_ms(1000);
// }
// 
// _delay_ms(500);

//p_b_read  = &EEPROM_AT24_buff_read[0];

uint8_t number = 64;
EEReadArray(number, 0x00, p_b_read);

_delay_ms(500);

for(uint8_t i=0; i < number; i++)
{
PRINTF ("EEPROM Read %02X Byte : %02X\n", i, EEPROM_AT24_buff_read [i]);
}
PRINTF("\r\n");


 
   while(1){
 
        mig_mig ();       // мигаем светодиодом сердцебиения "Heart Beat"
        read_parametr (); // читаем значения часов и температуры
		LogFileSave ();   // пишем лог температуры
		TM_line();        // читаем порт TouchMemory
		
        /* Get received character from ringbuffer
         * uart1_getc() returns in the lower byte the received character and 
         * in the higher byte (bitmask) the last receive error
         * UART_NO_DATA is returned when no data is available. */
		
        c = uart1_getc();

        if ( c & UART_NO_DATA )
        {
            /*  no data available from UART */
        }
        else
        {
            /* new data available from UART
             * check for Frame or Overrun error */
			
            if ( c & UART_FRAME_ERROR )
            {
                /* Framing Error detected, i.e no stop bit detected */
                PRINTF("UART Frame Error: ");
            }
            if ( c & UART_OVERRUN_ERROR )
            {
                /* Overrun, a character already present in the UART UDR register was 
                 * not read by the interrupt handler before the next character arrived,
                 * one or more received characters have been dropped */
                PRINTF("UART Overrun Error: ");
            }
            if ( c & UART_BUFFER_OVERFLOW )
            {
                /* We are not reading the receive buffer fast enough,
                 * one or more received character have been dropped */
                PRINTF("Buffer overflow error: ");
            }
            /* send received character back */
 //          uart1_putc( (unsigned char)NUM);
        }



		
     //Опрос кнопки входа в меню
     if((PINE & (1 << PE6))==0)                // Если нажата кнопка
     {
	     buzzer(800,1000); //beep @ 800 Hz for 1 second
	     _delay_ms(5);
	     buzzer(800,1000); //beep @ 800 Hz for 1 second
	     _delay_ms(5);
	     buzzer(800,1000); //beep @ 800 Hz for 1 second
		 		 
		 while((PINE & (1 << PE6))==0){}           // Ждем отпускания кнопки
				 
		 menu_r = do_menu2(m_menu, sizeof(m_menu)/sizeof(t_menu));
		 
	   //  while((PINE & (1 << PE6)&&(1 << PE7))==0){}        // Ждем отпускания кнопки
//	for(uint8_t i=0; i<20; i++)
//	  {		
		     
//             //TUI Start
// 		// очищаем индикатор
//         disp_clear();
// 		// выводим "заставку"
// 		gdSetXY(0, 0);
// 		gdWriteString((uint8_t*) "TUI DEMO");
// 		gdSetXY(0, 8);
// 		gdLoadFont(font_ks0066_ru_08, 0, FONT_DIR_0);
// 		gdWriteString((uint8_t*) "PRESS * TO MENU");
// 		gdLoadFont(font_ks0066_ru_08, 1, FONT_DIR_0);
// 		
// 		// ждем поступления события - нажатия кнопки включения меню
// 		do {
// 			ev = get_event();		// получаем событие
// 			
// 
// 			
// 		    }while(ev == EV_NONE);
// 		
// 		switch(ev){
// 			case EV_ENTER:
// 			// обрабатываем меню в зависимости от текущего режима
// 			//			menu_r = (mode==0) ? do_menu1(m_menu, sizeof(m_menu)/sizeof(t_menu)) : do_menu2(m_menu, sizeof(m_menu)/sizeof(t_menu));
// 			menu_r = do_menu2(m_menu, sizeof(m_menu)/sizeof(t_menu));
// 
// 			// выполняем действие в зависимости от полученного кода меню
// 			switch(menu_r)
// 			{
// 				case menu_multi:
// 				mode = 1;		// включаем многострочный режим меню
// 				break;
// 				case menu_one:
// 				//				mode = 0;		// включаем однострочный режим меню
// 				break;
// 				case menu_test:
// 				//test();		// выполняем демонстрационную функцию
// 				break;
// 				case menu_tes:
// 				//			    test1();		// выполняем демонстрационную функцию
// 				break;
// 				
// 			}
// 			break;
// 			
// 			case EV_ESCAPE:
// 			// при выходе из меню вернем 0
// 			//return 0;
//             disp_clear();
// 			break;
// 			
// 			
// 		          }

      }






    //disp_clear();

    g_draw_line_h(0, 0, 127);
    g_draw_line_h(0, 63, 127);
    g_draw_line_h(0, 53, 127);
    g_draw_line_v(0, 0, 63);
    g_draw_line_v(127, 0, 64);

    //применение функций Wiselord
//    gdLoadFont(font_ks0066_ru_24, 1, FONT_DIR_0);
//    gdSetXY(70, 2);
//    gdWriteString((uint8_t*)"Good");

    gdLoadFont(font_ks0066_ru_08, 1, FONT_DIR_0);
    gdSetXY(90, 36);
    gdWriteString((uint8_t*)"Good");


scrolling(message1);


//	gdSetXY(2,2);
	gdSetXY(2,10);	
	gdWriteString((uint8_t*)"Temp: ");
	gdWriteString(mkNumString(LM75_buff_1, 2, ' ', 10));
	gdWriteString((uint8_t*)"\x7F.\x7F");
	gdWriteString(mkNumString((uint16_t) LM75_buff_2, 1, ' ', 10));
	gdWriteString((uint8_t*)"C");
//	gdSetXY(2,10);
//	gdWriteString((uint8_t*)"ADC:");		
 	gdSetXY(2,18);
    gdWriteChar ((uint8_t)NUM);
	
    gdSetXY(50,18);
	gdWriteString(mkNumString((uint8_t) NUM, 3, ' ', 10));
	
	/*выводим значение напряжений на LCD*/ 
	gdSetXY(2,27);
	gdWriteString((uint8_t*)"ADC1: ");	
	gdWriteString(mkNumString(adc_read1_1, 1, ' ', 10));	
	gdWriteString((uint8_t*)"\x7F.\x7F");
	gdWriteString(mkNumString((uint16_t) adc_read1_2, 1, ' ', 10));
	gdWriteString((uint8_t*)"V");
	
	gdSetXY(2,36);
	gdWriteString((uint8_t*)"ADC2: ");
	gdWriteString(mkNumString(adc_read2_1, 1, ' ', 10));
	gdWriteString((uint8_t*)"\x7F.\x7F");
	gdWriteString(mkNumString((uint16_t) adc_read2_2, 1, ' ', 10));
	gdWriteString((uint8_t*)"V");		

	
    gdSetXY(62,27);
	gdWriteString((uint8_t*)"EEPROM: ");	
    gdWriteString(mkNumString(EEPROM_AT24_buff_read[1], 2, '0', 16));
	
    gdSetXY(2,45);
    gdWriteString((uint8_t*)"Pause: ");
    gdWriteString(mkNumString(pause, 3, '0', 10));

    gdSetXY(62,45);
    gdWriteString((uint8_t*)"Read: ");
    gdWriteString(mkNumString(read, 2, '0', 10));	
	

    gdSetXY(2, 55);
	gdWriteString(mkNumString(rtc.mday, 2, '0', 10));
 	gdWriteString((uint8_t*)"\x7F.\x7F");
	gdWriteString(mkNumString(rtc.month, 2, '0', 10));
	gdWriteString((uint8_t*)"\x7F.\x7F");
	gdWriteString(mkNumString(rtc.year, 4, '0', 10));
		
    gdSetXY(80, 55);
	gdWriteString(mkNumString(rtc.hour, 2, '0', 10));
	gdWriteString((uint8_t*)"\x7F:\x7F");
	gdWriteString(mkNumString(rtc.min, 2, '0', 10));
	gdWriteString((uint8_t*)"\x7F:\x7F");
	gdWriteString(mkNumString(rtc.sec, 2, '0', 10));			
		
    disp_send_frame();
		
     //Опрос кнопок установки времени		
	 if((PINE & (1 << PE4))==0)                 // Если нажата кнопка
	 {
		 while((PINE & (1 << PE4))==0){}        // Ждем отпускания кнопки
		 buzzer(800,1000); //beep @ 800 Hz for 1 second		
		 		 
		 uint8_t houra;
		 houra = rtc.hour;
		 houra++;                               // Увеличиваем часы на 1
		 if(houra > 23) houra = 0;

		 uint8_t buf[3];
		 buf[0] = (DS1307_ADR<<1)|0;            // адресный пакет
		 buf[1] = 0x02;                         // адрес регистра
		 buf[2] = ((houra/10) << 4)|(houra%10); // значение часов - Преобразуем из двоичной системы в BCD и записываем в DS1307		 
		 TWI_SendData(buf, 3);                  // отправляем его
	 }
	 
	 
	 if((PINE & (1 << PE5))==0)                 // Если нажата кнопка
	 {
		 while((PINE & (1 << PE5))==0){}        // Ждем отпускания кнопки
		 buzzer(800,1000); //beep @ 800 Hz for 1 second		
		 			 
		 uint8_t mina;
		 mina = rtc.min;		 
		 mina++;                                // Увеличиваем минуты на 1
		 if(mina > 59) mina = 0;

		 uint8_t buf[3];
		 buf[0] = (DS1307_ADR<<1)|0;            // адресный пакет
		 buf[1] = 0x01;                         // адрес регистра
		 buf[2] = ((mina/10) << 4)|(mina%10);   // значение минут - Преобразуем из двоичной системы в BCD и записываем в DS1307
		 TWI_SendData(buf, 3);                  // отправляем его
	 }	
		
// 	 if((PINE & (1 << PE6))==0)                 // Если нажата кнопка
// 	 {
// 		 PRINTF("I`m a Firestarter!!! /n");
// 	 }
// 		 
// 	 if((PINE & (1 << PE7))==0)                 // Если нажата кнопка
// 	 {		 
// 		 PRINTF("I`m a Fireowner!!! /n");
// 	 }	 
	 
  } //loop
  return 0;
}   //main




ISR(ADC_vect)
{
	uint8_t tmp;            // temp register for storage of misc data

	tmp = ADMUX;            // read the value of ADMUX register
	tmp &= 0x07;            // AND the first 4 bits (value of ADC pin being used)

//	ADCvalue = ADCH;        // read the sensor value
			  ADCvalue = ADCL;
			  ADCvalue |= (ADCH << 8);
			  
	if (tmp == 0)
	{

	ADCvalue1 = ADCvalue;        // read the sensor value
	ADCvalue1 += ADCvalue1;
	K++;
	if (K==4)
	{
	ADCvalue1=ADCvalue1>>2;
	K=0;
	ADCvalue1=0;	
	}
				
		// put ADCvalue into whatever register you use for ADC0 sensor
		ADMUX++;            // add 1 to ADMUX to go to the next sensor			
	}
	
	else if (tmp == 1)
	{
		// put ADCvalue into whatever register you use for ADC1 sensor
	ADCvalue2 = ADCvalue;        // read the sensor value
//		ADMUX++;            // add 1 to ADMUX to go to the next sensor	
    ADMUX &= 0xF8;		
	}
	
//	else if (tmp == 2)
	// put ADCvalue into whatever register you use for ADC2 sensor
//    ADCvalue3 = ADCvalue;        // read the sensor value
	
//	ADMUX &= 0xF8;      // clear the last 4 bits to reset the mux to ADC0
//    ADMUX|= (0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0); //Выбор вывода
}

ISR (TIMER0_COMP_vect)
{
	pause ++;      //счетчик для светодиода
	read ++;       //счетчик для опроса
	datalog ++;    //счетчик для записи лог-файла
}

ISR (TIMER2_COMP_vect)
{
	g_SysTIME++;	/* increase system time */
}

