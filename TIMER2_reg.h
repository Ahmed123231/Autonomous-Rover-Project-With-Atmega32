/*
 * TIMER2_reg.h
 *
 * Created: 6/15/2024 3:49:11 PM
 *  Author: ahmed
 */ 


#ifndef TIMER2_REG_H_
#define TIMER2_REG_H_


#define TCCR2_REG          *((volatile u8*)0x45)
#define TCNT2_REG          *((volatile u8*)0x44)
#define OCR2_REG           *((volatile u8*)0x43)
#define ASSR_REG           *((volatile u8*)0x42)
#define TIMSK_REG          *((volatile u8*)0x59)
#define TIFR_REG           *((volatile u8*)0x58)



#endif /* TIMER2_REG_H_ */