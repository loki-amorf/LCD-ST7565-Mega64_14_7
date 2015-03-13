//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru
//
//  Target(s)...: Mega
//
//  Compiler....:
//
//  Description.: Драйвер SPI
//
//  Data........: 2.10.12
//
//***************************************************************************

#include <avr/io.h>
#include <stdint.h>
#include "spi.h"


/*инициализация SPI*/
void SPI_Init(void)
{
   /*настройка портов ввода-вывода
   все выводы, кроме MISO выходы*/
   SPI_DDRX |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS)|(0<<SPI_MISO);
   SPI_DDRX &= ~(1<<SPI_MISO);
   SPI_PORTX |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS)|(1<<SPI_MISO);
   
  	/* Init chip select port  */
   SPI_SLAVE_DDR |= (1 << SPI_SLAVE_CS); /* port out */
   SPI_SLAVE_PORT |= (1 << SPI_SLAVE_CS); /* chip deselect */   
   
 
   /*разрешение spi,старший бит вперед,мастер, режим 0*/
   SPCR |=  (1<<SPE)|(1<<MSTR);
   SPCR &= ~(1<<DORD)|(1<<CPOL)|(1<<CPHA)|(1<<SPR0)|(1<<SPR1);
//   SPSR &= ~(1<<SPI2X); //обычная скорость
   SPSR |= (1<<SPI2X);    // SPI_SPEED_RATE_2X 	1 /* удвоенная скорость */
}


/*****************************************************************************
SPI port settings
******************************************************************************/

/* speed */
//#define SPI_SPEED_DIV4			((0 << SPR1)|(0 << SPR0))
//#define SPI_SPEED_DIV16			((0 << SPR1)|(1 << SPR0))
//#define SPI_SPEED_DIV64			((1 << SPR1)|(0 << SPR0))
//#define SPI_SPEED_DIV128	    	((1 << SPR1)|(1 << SPR0))

/* transfer modes */
//#define SPI_MODE_0				((0 << CPOL)|(0 << CPHA))
//#define SPI_MODE_1				((0 << CPOL)|(1 << CPHA))
//#define SPI_MODE_2				((1 << CPOL)|(0 << CPHA))
//#define SPI_MODE_3				((1 << CPOL)|(1 << CPHA))

/*****************************************************************************
SPI Init
******************************************************************************/
//void SPI_Init(void)
//{
	/*  MOSI, SCK и SS - out, MISO - in, include for MISO pull-up */
//	SPI_DDR |= (1 << SPI_MOSI)|(1 << SPI_SCK)|(1 << SPI_SS);
//	SPI_DDR &= ~(1 << SPI_MISO);
//	SPI_PORT |= (1 << SPI_MISO);

	/* enable SPI, MASTER mode, set SPI speed */
//	SPCR = (1 << SPE)|(1 << MSTR)| SPI_SPEED_RATE | SPI_MODE;

	/* set speed rate */
//	SPSR = (SPI_SPEED_RATE_2X << SPI2X);
//}



/*отослать байт данных по SPI*/
void SPI_WriteByte(uint8_t data)
{
   SPI_PORTX &= ~(1<<SPI_SS); 
   SPDR = data;
   while(!(SPSR & (1<<SPIF)));
   SPI_PORTX |= (1<<SPI_SS); 
}


/*отослать и получить байт данных по SPI*/
uint8_t SPI_ReadByte(uint8_t data)
{
   uint8_t report;
   
   SPI_PORTX &= ~(1<<SPI_SS);
   SPDR = data;
   while(!(SPSR & (1<<SPIF)));
   report = SPDR;
   SPI_PORTX |= (1<<SPI_SS); 
  
   return report; 
}


/*****************************************************************************
Exchenge byte's with SPI Protoss
******************************************************************************/
unsigned char SPI_Exch(unsigned char data)
{
	SPDR = data;						/* send data */
	while(0 == (SPSR & (1 <<SPIF))){};	/* wait for transfer */
	return SPDR;						/* return received byte */
}



/*отправить несколько байт данных по SPI*/
void SPI_WriteArray(uint8_t num, uint8_t *data)
{
   SPI_PORTX &= ~(1<<SPI_SS); 
   while(num--){
      SPDR = *data++;
      while(!(SPSR & (1<<SPIF)));
   }
   SPI_PORTX |= (1<<SPI_SS); 
}

/*****************************************************************************
Write block to SPI Protoss
******************************************************************************/
void SPI_WriteBlock(unsigned int size, unsigned char *data)
{
	while(size--)
	{
		SPDR = *data++;						/* send data */
		while(!(SPSR & (1 << SPIF))){};		/* wait for transfer */
	}
}

/*****************************************************************************
Read block with SPI Protoss
******************************************************************************/
void SPI_ReadBlock(unsigned int size, unsigned char *data)
{
	while(size--)
	{
		SPDR = 0;							/* send don't care data */
		while(0 == (SPSR & (1 <<SPIF))){};	/* wait for transfer */
		*data++ = SPDR;						/* return received byte */
	}
}

/*отправить и получить несколько байт данных по SPI*/
void SPI_ReadArray(uint8_t num, uint8_t *data)
{
   SPI_PORTX &= ~(1<<SPI_SS); 
   while(num--){
      SPDR = *data;
      while(!(SPSR & (1<<SPIF)));
      *data++ = SPDR; 
   }
   SPI_PORTX |= (1<<SPI_SS); 
}









