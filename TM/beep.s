#include <avr/io.h> // библиотека ввода-вывода

#define F_XTAL 18432000

#define	delay	r17
#define	temp	r16


#define	BUZZER	4
//******************************************************************************************
.global beep

beep:
	push 	temp
	lds		temp, SREG
	push 	temp
	push 	delay
	cbi		_SFR_IO_ADDR(PORTB), BUZZER
	ldi		delay, F_XTAL/18432000
	rcall	pause
	sbi		_SFR_IO_ADDR(PORTB), BUZZER
	pop		delay
	pop 	temp
	sts		SREG, temp
	pop 	temp
	ret
;******************************************************************************************
;задержка времени малая

pause:

	push 	r29
d_12:
	clr 	r29
d_11:
	wdr
	dec 	r29
	brne 	d_11
	dec 	delay
	brne 	d_12
	pop		r29
	ret
;******************************************************************************************
