/*
 * TIMER0_reg.h
 *
 * Created: 6/4/2024 11:15:20 AM
 *  Author: ahmed
 */ 


#ifndef TIMER0_REG_H_
#define TIMER0_REG_H_

#define TCCR0_REG   *((volatile u8*)0x53)
#define TCNT0_REG	*((volatile u8*)0x52)
#define TIMSK_REG	*((volatile u8*)0x59)
#define TIFR_REG	*((volatile u8*)0x58)
#define OCR0_REG    *((volatile u8*)0x5C)




#endif /* TIMER0_REG_H_ */