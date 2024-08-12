/*
 * TIMER2_interface.h
 *
 * Created: 6/15/2024 3:49:36 PM
 *  Author: ahmed
 */ 


#ifndef TIMER2_INTERFACE_H_
#define TIMER2_INTERFACE_H_

/****** Prescaler Config ************/

#define NO_CLOCK             0
#define NO_PRESCALER		 1
#define Division_8			 2
#define Division_64			 3
#define	Division_256		 4
#define Division_1023		 5

/*********** TIMER2 Mode Config *********/

#define OVF_MODE           0  
#define CTC_MODE           1
#define ASYNC_MODE         2
#define CTC_VAL            250  

/*********** TIMER2 Prescaler Config *********/

#define TIMER2_PRESCALER     Division_64

/*********** Function to Enable timer with Interrupt *****/

void TIMER2_voidInitTimerWithINTEnable(u8 copy_u8Mode);

/********* Timer2 Call Back Function **********/

void TIMER2_voidSetCallBack(u8 copy_u8Mode ,void(*ptr)(void));

/********* Timer2 Fast PWM Function **********/

void TIMER2_voidFastPWM(u8 copy_u8dutycycle);

/********* Timer2 Phase Correct PWM Function **********/

void TIMER2_voidPCPWM(u8 copy_u8dutycycle);

#endif /* TIMER2_INTERFACE_H_ */