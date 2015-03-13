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
#ifndef SPI_H

#define SPI_H
#include <stdint.h>
//#include "compilers.h"

#define SPI_PORTX PORTB
#define SPI_DDRX DDRB

#define SPI_MISO 3
#define SPI_MOSI 2
#define SPI_SCK  1
#define SPI_SS   0




/* chip select port for AT45DB321*/
#define SPI_SLAVE_PORT		PORTB
#define SPI_SLAVE_PIN		PINB
#define SPI_SLAVE_DDR		DDRB
#define SPI_SLAVE_CS		5	/* bit */


/*____________функции____________________*/

/*инициализация SPI модуля*/
void SPI_Init(void);

/*отправить байт данных по SPI*/
void SPI_WriteByte(uint8_t data);

/*отправить и получить байт данных по SPI*/
uint8_t SPI_ReadByte(uint8_t data);

/*отправить несколько байт данных по SPI*/
void SPI_WriteArray(uint8_t num, uint8_t *data);

/*отправить и получить несколько байт данных по SPI*/
void SPI_ReadArray(uint8_t num, uint8_t *data);




			/** Sends and receives a byte through the SPI interface, blocking until the transfer is complete.
			 *  \param[in] Byte  Byte to send through the SPI interface.
			 *  \return Response byte from the attached SPI device.
			 */
//			static inline uint8_t SPI_TransferByte(const uint8_t Byte) ATTR_ALWAYS_INLINE;
			static inline uint8_t SPI_TransferByte(const uint8_t Byte)
			{
				SPDR = Byte;
				while (!(SPSR & (1 << SPIF)));
				return SPDR;
			}

			/** Sends a byte through the SPI interface, blocking until the transfer is complete. The response
			 *  byte sent to from the attached SPI device is ignored.
			 *  \param[in] Byte  Byte to send through the SPI interface.
			 */
//			static inline void SPI_SendByte(const uint8_t Byte) ATTR_ALWAYS_INLINE;
			static inline void SPI_SendByte(const uint8_t Byte)
			{
				SPDR = Byte;
				while (!(SPSR & (1 << SPIF)));
			}

			/** Sends a dummy byte through the SPI interface, blocking until the transfer is complete. The response
			 *  byte from the attached SPI device is returned.
			 *  \return The response byte from the attached SPI device.
			 */
//			static inline uint8_t SPI_ReceiveByte(void) ATTR_ALWAYS_INLINE ATTR_WARN_UNUSED_RESULT;
			static inline uint8_t SPI_ReceiveByte(void)
			{
				SPDR = 0x00;
				while (!(SPSR & (1 << SPIF)));
				return SPDR;
			}





#endif //SPI_H