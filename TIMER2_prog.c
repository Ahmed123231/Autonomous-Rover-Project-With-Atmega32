/*
 * TIMER2_prog.c
 *
 * Created: 6/15/2024 3:50:02 PM
 *  Author: ahmed
 */ 

#include "BIT_MATH.h"
#include "STD_TYPE.h"
#include "TIMER2_reg.h"
#include "TIMER2_Interface.h"
#include "INT_Vector.h"

/************* Intializing Point To a Function Over Flow Mode **************/

static void(*OVF_ptr)(void)=0;

/************* Intializing Point To a Function CTC Mode **************/

static void(*CTC_ptr)(void)=0;


/*********** Function to Enable timer with Interrupt *****/

void TIMER2_voidInitTimerWithINTEnable(u8 copy_u8Mode){
	
	/****** 1-Prescaler Selection *********************/
	TCCR2_REG &= 0b11111000;
	TCCR2_REG |= TIMER2_PRESCALER ;
	if(copy_u8Mode == OVF_MODE){
		/****** 2- Timer Mode Selection : Overflow Mode (Normal Mode) ***/
		CLR_BIT(TCCR2_REG,3);
		CLR_BIT(TCCR2_REG,6);
		/****** 3- Enable Timer0 OverFlow Interrupt ******/
		SET_BIT(TIMSK_REG,6);
	}
	else if (copy_u8Mode == CTC_MODE){
		/****** 2- Timer Mode Selection : CTC Mode***/
		SET_BIT(TCCR2_REG,3);
		CLR_BIT(TCCR2_REG,6);
		/**** SET OCR0 Value *****/
		OCR2_REG=CTC_VAL;
		/****** 3- Enable Timer0 CTC Interrupt ******/
		SET_BIT(TIMSK_REG,7);
		
	}
	
	
}

/********* Timer2 Call Back Function **********/

void TIMER2_voidSetCallBack(u8 copy_u8Mode ,void(*ptr)(void)){
	
	if(copy_u8Mode == OVF_MODE){
		OVF_ptr=ptr;
	}
	else if (copy_u8Mode == CTC_MODE){
		CTC_ptr=ptr;
	}
	
	
}

/********* Timer2 Fast PWM Function **********/

void TIMER2_voidFastPWM(u8 copy_u8dutycycle){
	/****** 2-Prescaler Selection *********************/
	TCCR2_REG &= 0b11111000;
	TCCR2_REG |= TIMER2_PRESCALER ;
	
	SET_BIT(TCCR2_REG,3);
	SET_BIT(TCCR2_REG,6);
	SET_BIT(TCCR2_REG,5);
	CLR_BIT(TCCR2_REG,4);
	
	OCR2_REG=copy_u8dutycycle*2.55;
}




/********* Timer2 Phase Correct PWM Function **********/

void TIMER2_voidPCPWM(u8 copy_u8dutycycle){
	
	/****** 2-Prescaler Selection *********************/
	
	TCCR2_REG &= 0b11111000;
	TCCR2_REG |= TIMER2_PRESCALER ;
	
	SET_BIT(TCCR2_REG,6);
	CLR_BIT(TCCR2_REG,3);
	SET_BIT(TCCR2_REG,5);
	CLR_BIT(TCCR2_REG,4);
	
	OCR2_REG=copy_u8dutycycle*2.56;
	
}




ISR(_TIMER2_OVF){
	
	OVF_ptr();
}

ISR(_TIMER2_COMP){
	
	CTC_ptr();
	
}