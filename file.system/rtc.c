/*--------------------------------------------------------------------------*/
/*  DS1338 RTC controls                                                     */

#include <avr/io.h>
#include <string.h>
#include "rtc.h"
#include "../drivers/twim.h"
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

// #define	IIC_INIT()	DDRD &= 0xFC; PORTD &= 0xFC	/* Set SCL/SDA as hi-z    было: 11110011 стало: 11111100 */
// #define SCL_LOW()	DDRD |=	0x01			    /* SCL = LOW                    00000100        00000001 */
// #define SCL_HIGH()	DDRD &=	0xFE			    /* SCL = High-Z                 11111011        11111110 */
// #define	SCL_VAL		((PIND & 0x01) ? 1 : 0)	    /* SCL input level              00000100        00000001 */
// #define SDA_LOW()	DDRD |=	0x02			    /* SDA = LOW                    00001000        00000010 */
// #define SDA_HIGH()	DDRD &=	0xFD			    /* SDA = High-Z                 11110111        11111101 */
// #define	SDA_VAL		((PIND & 0x02) ? 1 : 0)	    /* SDA input level              00001000        00000010 */

#define M41T56_ADR    208  //Адрес часов      на шине I2C 0xD0 0b11010000, dec 208, HEX 0xD0
#define DS1307_ADR    104  //Адрес часов      на шине I2C 0x68 0b01101000, dec 104, HEX 0x68
#define LM75_ADR      72   //Адрес термометра на шине I2C 0x48 0b01001000, dec  72, HEX 0x48

// Macro to send strings stored in program memory space
#define PRINTF(format, ...) printf_P(PSTR(format), ## __VA_ARGS__)

/*-------------------------------------------------*/
/* I2C bus protocol                                */


// static
// void iic_delay (void)
// {
// 	int n;
// 
// 	for (n = 4; n; n--) PINB;
// }
// 
// 
// /* Generate start condition on the IIC bus */
// static
// void iic_start (void)
// {
// 	SDA_HIGH();
// 	iic_delay();
// 	SCL_HIGH();
// 	iic_delay();
// 	SDA_LOW();
// 	iic_delay();
// 	SCL_LOW();
// 	iic_delay();
// }
// 
// 
// /* Generate stop condition on the IIC bus */
// static
// void iic_stop (void)
// {
// 	SDA_LOW();
// 	iic_delay();
// 	SCL_HIGH();
// 	iic_delay();
// 	SDA_HIGH();
// 	iic_delay();
// }
// 
// 
// /* Send a byte to the IIC bus */
// static
// int iic_send (BYTE dat)
// {
// 	BYTE b = 0x80;
// 	int ack;
// 
// 
// 	do {
// 		if (dat & b)	 {	/* SDA = Z/L */
// 			SDA_HIGH();
// 		} else {
// 			SDA_LOW();
// 		}
// 		iic_delay();
// 		SCL_HIGH();
// 		iic_delay();
// 		SCL_LOW();
// 		iic_delay();
// 	} while (b >>= 1);
// 	SDA_HIGH();
// 	iic_delay();
// 	SCL_HIGH();
// 	ack = SDA_VAL ? 0 : 1;	/* Sample ACK */
// 	iic_delay();
// 	SCL_LOW();
// 	iic_delay();
// 	return ack;
// }
// 
// 
// /* Receive a byte from the IIC bus */
// static
// BYTE iic_rcvr (int ack)
// {
// 	UINT d = 1;
// 
// 
// 	do {
// 		d <<= 1;
// 		SCL_HIGH();
// 		if (SDA_VAL) d++;
// 		iic_delay();
// 		SCL_LOW();
// 		iic_delay();
// 	} while (d < 0x100);
// 	if (ack) {		/* SDA = ACK */
// 		SDA_LOW();
// 	} else {
// 		SDA_HIGH();
// 	}
// 	iic_delay();
// 	SCL_HIGH();
// 	iic_delay();
// 	SCL_LOW();
// 	SDA_HIGH();
// 	iic_delay();
// 
// 	return (BYTE)d;
// }
// 
// 
// 
// /*-------------------------------------------------*/
// /* I2C block read/write controls                   */
// 
// 
// int iic_read (
// 	BYTE dev,		/* Device address */
// 	UINT adr,		/* Read start address */
// 	UINT cnt,		/* Read byte count */
// 	void* buff		/* Read data buffer */
// )
// {
// 	BYTE *rbuff = buff;
// 	int n;
// 
// 
// 	if (!cnt) return 0;
// 
// 	n = 10;
// 	do {							/* Select device */
// 		iic_start();
// 	} while (!iic_send(dev) && --n);
// 	if (n) {
// 		if (iic_send((BYTE)adr)) {		/* Set start address */
// 			iic_start();				/* Reselect device in read mode */
// 			if (iic_send(dev | 1)) {
// 				do {					/* Receive data */
// 					cnt--;
// 					*rbuff++ = iic_rcvr(cnt ? 1 : 0);
// 				} while (cnt);
// 			}
// 		}
// 	}
// 
// 	iic_stop();						/* Deselect device */
// 
// 	return cnt ? 0 : 1;
// }
// 
// 
// 
// int iic_write (
// 	BYTE dev,			/* Device address */
// 	UINT adr,			/* Write start address */
// 	UINT cnt,			/* Write byte count */
// 	const void* buff	/* Data to be written */
// )
// {
// 	const BYTE *wbuff = buff;
// 	int n;
// 
// 
// 	if (!cnt) return 0;
// 
// 	n = 10;
// 	do {							/* Select device */
// 		iic_start();
// 	} while (!iic_send(dev) && --n);
// 	if (n) {
// 		if (iic_send((BYTE)adr)) {		/* Set start address */
// 			do {						/* Send data */
// 				if (!iic_send(*wbuff++)) break;
// 			} while (--cnt);
// 		}
// 	}
// 
// 	iic_stop();						/* Deselect device */
// 
// 	return cnt ? 0 : 1;
// }



/*-------------------------------------------------*/
/* RTC functions                                   */


int rtc_gettime (RTC *rtc)
{
	BYTE buf[8];


       /*устанавливаем указатель DS1307 на нулевой адрес*/
       buf[0] = (DS1307_ADR<<1)|0; //адресный пакет - адрес DS1307 + бит W
       buf[1] = 0;                 //адрес регистра
       TWI_SendData(buf, 2);
       
       /*считываем время с DS1307*/
       buf[0] = (DS1307_ADR<<1)|1; //адресный пакет - адрес DS1307 + бит R
       TWI_SendData(buf, 8);
       
       /*переписываем данные буфера драйвера в свой буфер*/
       TWI_GetData(buf, 8);
       

	rtc->sec = (buf[1] & 0x0F) + ((buf[1] >> 4) & 7) * 10;
	rtc->min = (buf[2] & 0x0F) + (buf[2] >> 4) * 10;
	rtc->hour = (buf[3] & 0x0F) + ((buf[3] >> 4) & 3) * 10;
	rtc->wday = (buf[4] & 0x07);
	rtc->mday = (buf[5] & 0x0F) + ((buf[5] >> 4) & 3) * 10;
	rtc->month = (buf[6] & 0x0F) + ((buf[6] >> 4) & 1) * 10;
	rtc->year = 2000 + (buf[7] & 0x0F) + (buf[7] >> 4) * 10;

	return 1;
}




// int rtc_settime (const RTC *rtc)
// {
// 
// 	BYTE buf[8];
// 
// 
// 	buf[0] = rtc->sec / 10 * 16 + rtc->sec % 10;
// 	buf[1] = rtc->min / 10 * 16 + rtc->min % 10;
// 	buf[2] = rtc->hour / 10 * 16 + rtc->hour % 10;
// 	buf[3] = rtc->wday & 7;
// 	buf[4] = rtc->mday / 10 * 16 + rtc->mday % 10;
// 	buf[5] = rtc->month / 10 * 16 + rtc->month % 10;
// 	buf[6] = (rtc->year - 2000) / 10 * 16 + (rtc->year - 2000) % 10;
// 	return iic_write(DS1307_ADR, 0, 7, buf);
// }




// int rtc_init (void)
// {
// 	BYTE buf[8];	/* RTC R/W buffer */
// 	UINT adr;
// 
// 
// 	IIC_INIT();		/* Initialize IIC function */
// 
// 	/* Read RTC registers */
// 	if (!iic_read(DS1307_ADR, 0, 8, buf)) return 0;	/* IIC error */
// 
// 	if (buf[7] & 0x20) {	/* When data has been volatiled, set default time */
// 		/* Clear nv-ram. Reg[8..63] */
// 		memset(buf, 0, 8);
// 		for (adr = 8; adr < 64; adr += 8)
// 			iic_write(DS1307_ADR, adr, 8, buf);
// 		/* Reset time to Jan 1, '08. Reg[0..7] */
// 		buf[4] = 1; buf[5] = 1; buf[6] = 8;
// 		iic_write(DS1307_ADR, 0, 8, buf);
// 	}
// 	return 1;
// }

 
 
void first_init_DS1307 (void)
{ 
	   	uint8_t buf[8];
//                                        // Запускаем ход часов
//        /*устанавливаем указатель DS1307 на нулевой адрес*/
//       buf[0] = (DS1307_ADR<<1)|0; //адресный пакет - адрес DS1307 + бит W
//       buf[1] = 0;                 //адрес регистра
//       TWI_SendData(buf, 2);
//    
//       /*считываем время с DS1307*/
//       buf[0] = (DS1307_ADR<<1)|1; //адресный пакет - адрес DS1307 + бит R
//       TWI_SendData(buf, 2);
//       
//       /*переписываем данные буфера драйвера в свой буфер*/
//       TWI_GetData(buf, 2);
// 	       
//       sec  = buf[1];
//       sec &= ~(1 << 7); // обнуляем 7 бит
// 
// 	  
//       /*подготавливаем сообщение*/
//       buf[0] = (DS1307_ADR<<1)|0;  //адресный пакет
//       buf[1] = 0;                  //адрес регистра
//       buf[2] = sec;                //значение секунд 
//  
//       /*отправляем его*/
//       TWI_SendData(buf, 3);       //отправляем его 
   
   
 //Пример- Запись даты в регистры DS1307
 /*подготавливаем сообщение*/
buf[0] = (DS1307_ADR<<1)|0;  //адресный пакет
buf[1] = 0x03;               //адрес регистра дней недели
buf[2] = (0<<4)|6;           //день недели - 1 - SUN 
buf[3] = (1<<4)|3;           //день  - 21
buf[4] = (0<<4)|2;           //месяц - 09
buf[5] = (1<<4)|5;           //год   - 14


 TWI_SendData(buf, 6);       //отправляем его 
}