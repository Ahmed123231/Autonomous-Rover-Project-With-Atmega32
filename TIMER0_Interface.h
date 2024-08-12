/*
 * TIMER0_Interface.h
 *
 * Created: 6/4/2024 11:14:49 AM
 *  Author: ahmed
 */ 


#ifndef TIMER0_INTERFACE_H_
#define TIMER0_INTERFACE_H_


/****** Prescaler Config ************/

#define NO_CLOCK             0
#define NO_PRESCALER		 1
#define Division_8			 2
#define Division_64			 3
#define	Division_256		 4
#define Division_1023		 5

/*********** TIMER0 Mode Config *********/
#define OVF_MODE             0
#define CTC_MODE             1
#define CTC_VAL              250

#define TIMER0_PRESCALER     Division_64

/*********** Function to Enable timer with Interrupt *****/

void TIMER0_voidInitTimerWithINTEnable(u8 copy_u8Mode);


/********* Timer0 Call Back Function **********/

void TIMER0_voidSetCallBack(u8 copy_u8Mode ,void(*ptr)(void));

/********* Timer0 Fast PWM Function **********/

void TIMER0_voidFastPWM(u8 copy_u8dutycycle);

/********* Timer0 Phase Correct PWM Function **********/

void TIMER0_voidPCPWM(u8 copy_u8dutycycle);

#endif /* TIMER0_INTERFACE_H_ */