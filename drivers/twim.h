//***************************************************************************
//
//  Author(s)...: ����� ������  http://ChipEnable.Ru   
//
//  Target(s)...: mega16
//
//  Compiler....: GCC
//
//  Description.: ������� �������� TWI ����������. 
//                ��� ������� �� Atmel`������ ����� - AVR315.
//
//  Data........: 13.11.13
//
//***************************************************************************
#ifndef TWIM_H
#define TWIM_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>


 //��������� ��� ������/������ ����������� ����������
// struct save
// {
// 	float summ_1, summ2, summ_3,
// 	reserve_1, reserve_2, reserve_3,
// 	reserve_max_1, reserve_max_2,
// 	reserve_max_3;
// 	unsigned long int statis_1,
// 	statis_2, statis_3, pot_1,
// 	pot_2, pot_3;
// 	unsigned char flag_1, flag_2,
// 	flag_3, numb, min_off, midle_off,
// 	max_off;
// 	};
 //����������� ���������� var ���� struct save	
/*struct save var_1;*/

 //������������� ��������� ���������� ������ ��������� var
/*void *p_m_1;*/

 //��������� ���������� ����� ��������� var
/*unsigned char *p_b_1;*/

 //�������� ��������� ��������� ����� ��������� var
//p_m_1 = &var_1;

// unsigned char *p_b_2;		// ��������� �� ������ EEPROM
// 
// unsigned char *p_b_3;		// ��������� �� ������ EEPROM
// 
// unsigned char *p_b_4;		// ��������� �� ������ EEPROM
/****************************************************************************
  ��������� ������
****************************************************************************/

/*���� �� ���������� �������� �������, ���������� �� �����*/
#ifndef F_CPU
   #define F_CPU  18432000UL
#endif

/*������ ������ TWI ������*/
#define TWI_BUFFER_SIZE  64      

/****************************************************************************
  ��������� ���� TWI ������ 
****************************************************************************/

/*����� ��������� ���� */                    
#define TWI_START                  0x08  // ��������� START ������������ 
#define TWI_REP_START              0x10  // ��������� ��������� START ������������ 
#define TWI_ARB_LOST               0x38  // ��� ������� ��������� 

/*��������� ���� �������� �����������*/                
#define TWI_MTX_ADR_ACK            0x18  // ��� ������� ����� SLA+W � �������� �������������
#define TWI_MTX_ADR_NACK           0x20  // ��� ������� ���� SLA+W � �� �������� �������������
#define TWI_MTX_DATA_ACK           0x28  // ��� ������� ���� ������ � �������� �������������  
#define TWI_MTX_DATA_NACK          0x30  // ��� ������� ���� ������ � �� �������� �������������

/*��������� ���� �������� ���������*/ 
#define TWI_MRX_ADR_ACK            0x40  // ��� ������� ����� SLA+R � �������� ������������ 
#define TWI_MRX_ADR_NACK           0x48  // ��� ������� ����� SLA+R � �� �������� ������������� 
#define TWI_MRX_DATA_ACK           0x50  // ���� ������ ������ � �������� �������������  
#define TWI_MRX_DATA_NACK          0x58  // ��� ������ ���� ������ ��� �������������  

/*������ ��������� ����*/
#define TWI_NO_STATE               0xF8  // �������������� ���������; TWINT = �0�
#define TWI_BUS_ERROR              0x00  // ������ �� ���� ��-�� ����������� ��������� ����� ��� ����

/*���������������� ����*/
#define TWI_SUCCESS                0xff

/****************************************************************************
  ����������� ��������
****************************************************************************/

#define TWI_READ_BIT     0       // ������� R/W ���� � �������� ������
#define TWI_ADR_BITS     1       // ������� ������ � �������� ������
#define TRUE             1
#define FALSE            0

/****************************************************************************
  ���������������� �������
****************************************************************************/

/*������������� � ��������� ������� SCL �������*/
uint8_t TWI_MasterInit(uint16_t fr);

/*�������� ������*/
void TWI_SendData(uint8_t *msg, uint8_t msgSize);

/*�������� �������� ������*/
uint8_t TWI_GetData(uint8_t *msg, uint8_t msgSize);

/*����� ������ TWI ������*/
uint8_t TWI_GetState(void);

void EEWriteByte(uint16_t address,uint8_t data);

void EEWriteArray (uint8_t num, uint16_t address, uint8_t *data);

uint8_t EEReadByte(uint16_t address);

uint8_t EEReadArray(uint8_t num, uint16_t address,  uint8_t *data);

uint8_t LM75_Read (void);

#endif //TWIM_H