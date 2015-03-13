/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "diskio.h"

/*****************************************************************************
debug messages
******************************************************************************/

// Macro to send strings stored in program memory space
#define PRINTF(format, ...) printf_P(PSTR(format), ## __VA_ARGS__)



/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive number (0..) */
)
{
	return  STA_OK;
//	return FR_OK;
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive number (0..) */
)
{
	return  STA_OK;
//	return FR_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive number (0..) */
	BYTE *buf,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	DataflashManager_ReadBlocks_RAM(sector, count, buf);
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive number (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	DataflashManager_WriteBlocks_RAM(sector, count, buff);
	return RES_OK;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

// DRESULT disk_ioctl (
// 	BYTE drv,		/* Physical drive number (0..) */
// 	BYTE ctrl,		/* Control code */
// 	void *buff		/* Buffer to send/receive control data */
// )
// {
// 	if (ctrl == CTRL_SYNC)
// 	  return RES_OK;
// 	else
// 	  return RES_PARERR;
// }

DRESULT disk_ioctl(BYTE pdrv, BYTE Command, void* buf)
{
	switch(Command)
	{
		case CTRL_SYNC:	/* Make sure that the disk drive has finished pending write process */
		{
	         return RES_OK;
		}
		break;

		case GET_SECTOR_COUNT:	/* Returns total sectors on the drive */
		{
			*(DWORD *)buf = AT45_PAGES;
		}
		break;

		case GET_BLOCK_SIZE:	/* Returns erase block size of the memory array */
		{
			*(DWORD *)buf = DISK_SEC_SIZE;
		}
		break;

		default:
		return RES_PARERR;
	}

	return RES_OK;
}

