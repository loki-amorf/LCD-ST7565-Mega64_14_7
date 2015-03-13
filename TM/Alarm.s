#include <avr/io.h> // библиотека ввода-вывода

#define F_XTAL 18432000

#define	delay	r28

#define	temp	r20

// параметр numb функции void alarm(unsigned char numb) передается компилятором через r24
#define	temp_1	r24		// число трелей звукового сигнала

#define	BUZZER	4
//*****************************************************************************************
.global alarm

alarm:
	push	temp
	lds		temp, SREG
	push	temp
	push	delay
cycle_alarm:
	wdr
	ldi		temp, 5
cycle_alarm_1:
	cbi		_SFR_IO_ADDR(PORTB), BUZZER
	ldi		delay, F_XTAL/230400
	rcall	pause
	sbi		_SFR_IO_ADDR(PORTB), BUZZER
	ldi		delay, F_XTAL/230400
	rcall	pause
	dec 	temp
	brne 	cycle_alarm_1
	ldi		delay, F_XTAL/73728
	rcall	pause
	ldi		delay, F_XTAL/73728
	rcall	pause
	dec 	temp_1
	brne 	cycle_alarm
	pop		delay
	pop		temp
	sts		SREG, temp
	pop		temp
	ret
;******************************************************************************************
;задержка времени малая

pause:

	push	r29
d_12:
	clr		r29
d_11:
	wdr
	dec		r29
	brne	d_11
	dec		delay
	brne	d_12
	pop		r29
	ret
;******************************************************************************************
