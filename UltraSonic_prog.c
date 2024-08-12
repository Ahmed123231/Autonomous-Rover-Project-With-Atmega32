/*
 * UltraSonic_prog.c
 *
 * Created: 8/5/2024 9:55:35 PM
 *  Author: ahmed
 */ 


#include "BIT_MATH.h"
#include "STD_TYPE.h"
#include "DIO_interface.h"
#include "TIMER0_Interface.h"
#include "TIMER0_reg.h"
#include "UltraSonic_interface.h"
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/io.h>




void UltraSonic_Sendpulse(void){
	
	DIO_voidSetPinVal(TRIG_PORT,TRIG_PIN,HIGH);
	_delay_us(10);
	DIO_voidSetPinVal(TRIG_PORT,TRIG_PIN,LOW);
	
	
}
u32 UltraSonic_Read_Echo(void){
	
	unsigned long count = 0;
	unsigned long duration = 0;
	unsigned char overflow_count = 0;

	// Reset Timer0 and set prescaler
	TCNT0 = 0;
	TCCR0 = (1 << CS01) | (1 << CS00); // Prescaler 64

	// Wait for the echo pin to go HIGH
	
	while (DIO_u8GetPinVal(ECHO_PORT, ECHO_PIN) == 0);

	// Measure the duration of the pulse
	while (DIO_u8GetPinVal(ECHO_PORT, ECHO_PIN) == 1)
	{
		// Check for Timer0 overflow
		if (TIFR & (1 << TOV0))
		{
			TIFR = (1 << TOV0); // Clear overflow flag
			overflow_count++;
			TCNT0 = 0; // Reset Timer0 count
		}
	}

	duration = (overflow_count * (255+ 1)) + TCNT0; // Total duration in timer counts

	// Stop Timer0
	TCCR0 = 0; // Turn off Timer0

	return duration;
	
	
	
}

u8 UltraSonic_Calc_Distance(u32(*FuncPtr)(void)){
	
	
	u32 duration =FuncPtr();
	u8 distance=0;
	
	distance = ((duration * SOUND_SPEED) / (F_CPU / PRESCALER)) / 2;
	distance -= OFFSET_CM; // Correct for the offset
	//distance /=10;
	
	return distance ;
	
	
	
}

