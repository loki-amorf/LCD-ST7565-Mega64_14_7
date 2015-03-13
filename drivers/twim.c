//***************************************************************************
//
//  Author(s)...: Павел Бобков  http://ChipEnable.Ru   
//
//  Target(s)...: mega16
//
//  Compiler....: GCC
//
//  Description.: Драйвер ведущего TWI устройства. 
//                Код основан на Atmel`овских доках - AVR315.
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


extern uint8_t *p_b_write;		// указатель на массив EEPROM

extern uint8_t *p_b_read;		// указатель на массив EEPROM


#define AT24C256_ADR       80   //Адрес EEPROM на шине I2C 0x50 0b1010000 dec  80
#define LM75_ADR           72   //Адрес термометра на шине I2C 0x48 0b1001000 dec  72

volatile static uint8_t twiBuf[TWI_BUFFER_SIZE];
volatile static uint8_t twiState = TWI_NO_STATE;      
volatile static uint8_t twiMsgSize;       

/*предделители для установки скорости обмена twi модуля*/
uint8_t pre[4] = {2, 8, 32, 128};

/****************************************************************************
 Инициализация и установка частоты SCL сигнала
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
 Проверка - не занят ли TWI модуль. Используется внутри модуля
****************************************************************************/
static uint8_t TWI_TransceiverBusy(void)
{
  return (TWCR & (1<<TWIE));                 
}

/****************************************************************************
 Взять статус TWI модуля
****************************************************************************/
uint8_t TWI_GetState(void)
{
  while (TWI_TransceiverBusy());             
  return twiState;                        
}

/****************************************************************************
 Передать сообщение msg из msgSize байтов на TWI шину
****************************************************************************/
void TWI_SendData(uint8_t *msg, uint8_t msgSize)
{
  uint8_t i;

  while(TWI_TransceiverBusy());   //ждем, когда TWI модуль освободится             

  twiMsgSize = msgSize;           //сохряняем кол. байт для передачи             
  twiBuf[0]  = msg[0];            //и первый байт сообщения 
  
  if (!(msg[0] & (TRUE<<TWI_READ_BIT))){   //если первый байт типа SLA+W
    for (i = 1; i < msgSize; i++){         //то сохраняем остальную часть сообщения
      twiBuf[i] = msg[i];
    }
  }
                       
  twiState = TWI_NO_STATE ;
  TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); //разрешаем прерывание и формируем состояние старт                            
}

/****************************************************************************
 Переписать полученные данные в буфер msg в количестве msgSize байт. 
****************************************************************************/
uint8_t TWI_GetData(uint8_t *msg, uint8_t msgSize)
{
  uint8_t i;

  while(TWI_TransceiverBusy());    //ждем, когда TWI модуль освободится 

  if(twiState == TWI_SUCCESS){     //если сообщение успешно принято,                         
    for(i = 0; i < msgSize; i++){  //то переписываем его из внутреннего буфера в переданный
      msg[i] = twiBuf[i];
    }
  }
  
  return twiState;                                   
}

/****************************************************************************
 Обработчик прерывания TWI модуля
****************************************************************************/
ISR(TWI_vect)
{
  static uint8_t ptr;
  uint8_t stat = TWSR & TWSR_MASK;
  
  switch (stat){
    
    case TWI_START:                   // состояние START сформировано 
    case TWI_REP_START:               // состояние повторный START сформировано        
       ptr = 0;      

    case TWI_MTX_ADR_ACK:             // был передан пакет SLA+W и получено подтверждение
    case TWI_MTX_DATA_ACK:            // был передан байт данных и получено подтверждение  
       if (ptr < twiMsgSize){
          TWDR = twiBuf[ptr];                    //загружаем в регистр данных следующий байт
          TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT); //сбрасываем флаг TWINT    
          ptr++;
       }
       else{
          twiState = TWI_SUCCESS;  
          TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO)|(0<<TWIE); //формируем состояние СТОП, сбрасываем флаг, запрещаем прерывания
       }
       break;
     
    case TWI_MRX_DATA_ACK:          //байт данных принят и передано подтверждение  
       twiBuf[ptr] = TWDR;
       ptr++;
    
    case TWI_MRX_ADR_ACK:           //был передан пакет SLA+R и получено подтвеждение  
      if (ptr < (twiMsgSize-1)){
        TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);  //если это не предпоследний принятый байт, формируем подтверждение                             
      }
      else {
        TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT);            //если приняли предпоследний байт, подтверждение не формируем
      }    
      break; 
      
    case TWI_MRX_DATA_NACK:       //был принят байт данных без подтверждения      
      twiBuf[ptr] = TWDR;
      twiState = TWI_SUCCESS;  
      TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO); //формируем состояние стоп
      break; 
     
    case TWI_ARB_LOST:          //был потерян приоритет 
      TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); // сбрасываем флаг TWINT, формируем повторный СТАРТ
      break;
      
    case TWI_MTX_ADR_NACK:      // был передан пает SLA+W и не получено подтверждение
    case TWI_MRX_ADR_NACK:      // был передан пакет SLA+R и не получено подтверждение    
    case TWI_MTX_DATA_NACK:     // был передан байт данных и не получено подтверждение
    case TWI_BUS_ERROR:         // ошибка на шине из-за некоректных состояний СТАРТ или СТОП
    default:     
      twiState = stat;                                                                                    
      TWCR = (1<<TWEN)|(0<<TWIE)|(0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); //запретить прерывание                              
  }
}



void EEWriteByte(uint16_t address,uint8_t data)
{
    uint8_t buf[8];


	buf[0] = (AT24C256_ADR<<1)|0;    //адресный пакет - адрес AT24C256 + бит W
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

	buf[0] = (AT24C256_ADR<<1)|0;    //адресный пакет - адрес AT24C256 + бит W
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
	buf[0] = (AT24C256_ADR<<1)|0;    //адресный пакет - адрес AT24C256 + бит W
  	buf[1] = (address>>8);           //Now write ADDRH
  	buf[2] =  address & 0x00FF;      //Now write ADDRL
//	buf[1] = 0x00;                   //Now write ADDRH
//	buf[2] = 0x00;                   //Now write ADDRL	
	TWI_SendData(buf, 3);
  //*************************DUMMY WRITE SEQUENCE END **********************
	
	/*считываем */
	buf[0] = (AT24C256_ADR<<1)|1;    //адресный пакет - адрес AT24C256 + бит R
	buf[1] = 0;
	TWI_SendData(buf, 2);
	
	/*переписываем данные буфера драйвера в свой буфер*/
	TWI_GetData(buf, 2);
	
   EEPROM_AT24_buff_read[address] = buf[1];
	
	return 1;
}




uint8_t EEReadArray(uint8_t num, uint16_t address,  uint8_t *data)
{
	uint8_t k = num+1;	
   	uint8_t buf[k];

   

    //Initiate a Dummy Write Sequence to start Random Read
	buf[0] = (AT24C256_ADR<<1)|0;    //адресный пакет - адрес AT24C256 + бит W
  	buf[1] = (address>>8);           //Now write ADDRH
  	buf[2] =  address & 0x00FF;      //Now write ADDRL
	TWI_SendData(buf, 3);
    //*************************DUMMY WRITE SEQUENCE END **********************
	
	/*считываем */
	buf[0] = (AT24C256_ADR<<1)|1;    //адресный пакет - адрес AT24C256 + бит R
	buf[1] = 0;
	TWI_SendData(buf, k);
	
	/*переписываем данные буфера драйвера в свой буфер*/
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

      buf[0] = (LM75_ADR<<1)|1; //адресный пакет - адрес LM75A + бит R
      TWI_SendData(buf, 3);
      
      /*переписываем данные буфера драйвера в свой буфер*/
      TWI_GetData(buf, 3);
	       
      MSByte  = buf[1];
      LSByte  = buf[2];
	  
    LM75_buff = ((MSByte <<8 | LSByte)>>5)*0.125; //Складываем старший байт с младшим, 
	                                              //предварительно младший сдвигаем вправо на 5 бит,
											      //старший сдвигаем влево на 8 бит.
//    LM75_buff = (ceil((LM75_buff * 10)))/10;      //Округляем значение температуры до одного знака после запятой
	LM75_buff = ceil((LM75_buff * 10));           //Округляем значение температуры до одного знака после запятой
	LM75_buff_1 = LM75_buff / 10;
	LM75_buff_2 = (uint8_t)LM75_buff % 10;
}