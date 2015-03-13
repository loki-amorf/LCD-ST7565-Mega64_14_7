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
#include "twim.h"
#include <math.h>

#define TWSR_MASK     0xfc  

extern uint8_t EEPROM_AT24_buff_read[32];
extern uint8_t EEPROM_AT24_buff_write[32];
//extern float LM75_buff;
extern uint8_t LM75_buff_1;
extern uint8_t LM75_buff_2;


extern uint8_t *p_b_write;		// ��������� �� ������ EEPROM

extern uint8_t *p_b_read;		// ��������� �� ������ EEPROM


#define AT24C256_ADR       80   //����� EEPROM �� ���� I2C 0x50 0b1010000 dec  80
#define LM75_ADR           72   //����� ���������� �� ���� I2C 0x48 0b1001000 dec  72

volatile static uint8_t twiBuf[TWI_BUFFER_SIZE];
volatile static uint8_t twiState = TWI_NO_STATE;      
volatile static uint8_t twiMsgSize;       

/*������������ ��� ��������� �������� ������ twi ������*/
uint8_t pre[4] = {2, 8, 32, 128};

/****************************************************************************
 ������������� � ��������� ������� SCL �������
****************************************************************************/
uint8_t TWI_MasterInit(uint16_t fr)
{
  uint8_t i;
  uint16_t twbrValue;
  
  for(i = 0; i<4; i++){
    twbrValue = ((((F_CPU)/1000UL)/fr)-16)/pre[i];
    if ((twbrValue > 0)&& (twbrValue < 256)){
       TWBR = (uint8_t)twbrValue;
       TWSR = i;
       TWDR = 0xFF;
       TWCR = (1<<TWEN);
       return TWI_SUCCESS;
    }
  }
  return 0;  
}    

/****************************************************************************
 �������� - �� ����� �� TWI ������. ������������ ������ ������
****************************************************************************/
static uint8_t TWI_TransceiverBusy(void)
{
  return (TWCR & (1<<TWIE));                 
}

/****************************************************************************
 ����� ������ TWI ������
****************************************************************************/
uint8_t TWI_GetState(void)
{
  while (TWI_TransceiverBusy());             
  return twiState;                        
}

/****************************************************************************
 �������� ��������� msg �� msgSize ������ �� TWI ����
****************************************************************************/
void TWI_SendData(uint8_t *msg, uint8_t msgSize)
{
  uint8_t i;

  while(TWI_TransceiverBusy());   //����, ����� TWI ������ �����������             

  twiMsgSize = msgSize;           //��������� ���. ���� ��� ��������             
  twiBuf[0]  = msg[0];            //� ������ ���� ��������� 
  
  if (!(msg[0] & (TRUE<<TWI_READ_BIT))){   //���� ������ ���� ���� SLA+W
    for (i = 1; i < msgSize; i++){         //�� ��������� ��������� ����� ���������
      twiBuf[i] = msg[i];
    }
  }
                       
  twiState = TWI_NO_STATE ;
  TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); //��������� ���������� � ��������� ��������� �����                            
}

/****************************************************************************
 ���������� ���������� ������ � ����� msg � ���������� msgSize ����. 
****************************************************************************/
uint8_t TWI_GetData(uint8_t *msg, uint8_t msgSize)
{
  uint8_t i;

  while(TWI_TransceiverBusy());    //����, ����� TWI ������ ����������� 

  if(twiState == TWI_SUCCESS){     //���� ��������� ������� �������,                         
    for(i = 0; i < msgSize; i++){  //�� ������������ ��� �� ����������� ������ � ����������
      msg[i] = twiBuf[i];
    }
  }
  
  return twiState;                                   
}

/****************************************************************************
 ���������� ���������� TWI ������
****************************************************************************/
ISR(TWI_vect)
{
  static uint8_t ptr;
  uint8_t stat = TWSR & TWSR_MASK;
  
  switch (stat){
    
    case TWI_START:                   // ��������� START ������������ 
    case TWI_REP_START:               // ��������� ��������� START ������������        
       ptr = 0;      

    case TWI_MTX_ADR_ACK:             // ��� ������� ����� SLA+W � �������� �������������
    case TWI_MTX_DATA_ACK:            // ��� ������� ���� ������ � �������� �������������  
       if (ptr < twiMsgSize){
          TWDR = twiBuf[ptr];                    //��������� � ������� ������ ��������� ����
          TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT); //���������� ���� TWINT    
          ptr++;
       }
       else{
          twiState = TWI_SUCCESS;  
          TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO)|(0<<TWIE); //��������� ��������� ����, ���������� ����, ��������� ����������
       }
       break;
     
    case TWI_MRX_DATA_ACK:          //���� ������ ������ � �������� �������������  
       twiBuf[ptr] = TWDR;
       ptr++;
    
    case TWI_MRX_ADR_ACK:           //��� ������� ����� SLA+R � �������� ������������  
      if (ptr < (twiMsgSize-1)){
        TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);  //���� ��� �� ������������� �������� ����, ��������� �������������                             
      }
      else {
        TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT);            //���� ������� ������������� ����, ������������� �� ���������
      }    
      break; 
      
    case TWI_MRX_DATA_NACK:       //��� ������ ���� ������ ��� �������������      
      twiBuf[ptr] = TWDR;
      twiState = TWI_SUCCESS;  
      TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO); //��������� ��������� ����
      break; 
     
    case TWI_ARB_LOST:          //��� ������� ��������� 
      TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); // ���������� ���� TWINT, ��������� ��������� �����
      break;
      
    case TWI_MTX_ADR_NACK:      // ��� ������� ���� SLA+W � �� �������� �������������
    case TWI_MRX_ADR_NACK:      // ��� ������� ����� SLA+R � �� �������� �������������    
    case TWI_MTX_DATA_NACK:     // ��� ������� ���� ������ � �� �������� �������������
    case TWI_BUS_ERROR:         // ������ �� ���� ��-�� ����������� ��������� ����� ��� ����
    default:     
      twiState = stat;                                                                                    
      TWCR = (1<<TWEN)|(0<<TWIE)|(0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); //��������� ����������                              
  }
}



void EEWriteByte(uint16_t address,uint8_t data)
{
    uint8_t buf[8];


	buf[0] = (AT24C256_ADR<<1)|0;    //�������� ����� - ����� AT24C256 + ��� W
 	buf[1] = (address>>8);           //Now write ADDRH
 	buf[2] =  address & 0x00FF;      //Now write ADDRL	
 	buf[3] =  data;
	TWI_SendData(buf, 4);
	
//	return 1;
}

void EEWriteArray (uint8_t num, uint16_t address,  uint8_t *data)
{
	uint8_t k = num+3;
	uint8_t buf[k];

	buf[0] = (AT24C256_ADR<<1)|0;    //�������� ����� - ����� AT24C256 + ��� W
	buf[1] = (address>>8);           //Now write ADDRH
	buf[2] =  address & 0x00FF;      //Now write ADDRL

     for(uint8_t i=0; i < num; i++)
 	    {
//	    buf[i+3] =  *p_b_write ++;	
 	    buf[i+3] =  data [i];	
 	    }				
	   TWI_SendData(buf, k);
}



uint8_t EEReadByte(uint16_t address)
{
   	uint8_t buf[8];

  //Initiate a Dummy Write Sequence to start Random Read
	buf[0] = (AT24C256_ADR<<1)|0;    //�������� ����� - ����� AT24C256 + ��� W
  	buf[1] = (address>>8);           //Now write ADDRH
  	buf[2] =  address & 0x00FF;      //Now write ADDRL
//	buf[1] = 0x00;                   //Now write ADDRH
//	buf[2] = 0x00;                   //Now write ADDRL	
	TWI_SendData(buf, 3);
  //*************************DUMMY WRITE SEQUENCE END **********************
	
	/*��������� */
	buf[0] = (AT24C256_ADR<<1)|1;    //�������� ����� - ����� AT24C256 + ��� R
	buf[1] = 0;
	TWI_SendData(buf, 2);
	
	/*������������ ������ ������ �������� � ���� �����*/
	TWI_GetData(buf, 2);
	
   EEPROM_AT24_buff_read[address] = buf[1];
	
	return 1;
}




uint8_t EEReadArray(uint8_t num, uint16_t address,  uint8_t *data)
{
	uint8_t k = num+1;	
   	uint8_t buf[k];

   

    //Initiate a Dummy Write Sequence to start Random Read
	buf[0] = (AT24C256_ADR<<1)|0;    //�������� ����� - ����� AT24C256 + ��� W
  	buf[1] = (address>>8);           //Now write ADDRH
  	buf[2] =  address & 0x00FF;      //Now write ADDRL
	TWI_SendData(buf, 3);
    //*************************DUMMY WRITE SEQUENCE END **********************
	
	/*��������� */
	buf[0] = (AT24C256_ADR<<1)|1;    //�������� ����� - ����� AT24C256 + ��� R
	buf[1] = 0;
	TWI_SendData(buf, k);
	
	/*������������ ������ ������ �������� � ���� �����*/
	TWI_GetData(buf, k);
	
      for(uint8_t i=0; i < num; i++)
      {
 //     *p_b_3 ++ = buf[i+1];
      p_b_read [i] = buf[i+1];
 //       data [i]  = buf[i+1];
//      *data ++  = buf[i+1];
//     *p_b_4 ++ = buf[i+1];
	  }
	return 1;
}


	

uint8_t LM75_Read (void)
{
    uint8_t MSByte, LSByte, buf[3];
	float LM75_buff;

      buf[0] = (LM75_ADR<<1)|1; //�������� ����� - ����� LM75A + ��� R
      TWI_SendData(buf, 3);
      
      /*������������ ������ ������ �������� � ���� �����*/
      TWI_GetData(buf, 3);
	       
      MSByte  = buf[1];
      LSByte  = buf[2];
	  
    LM75_buff = ((MSByte <<8 | LSByte)>>5)*0.125; //���������� ������� ���� � �������, 
	                                              //�������������� ������� �������� ������ �� 5 ���,
											      //������� �������� ����� �� 8 ���.
//    LM75_buff = (ceil((LM75_buff * 10)))/10;      //��������� �������� ����������� �� ������ ����� ����� �������
	LM75_buff = ceil((LM75_buff * 10));           //��������� �������� ����������� �� ������ ����� ����� �������
	LM75_buff_1 = LM75_buff / 10;
	LM75_buff_2 = (uint8_t)LM75_buff % 10;
}