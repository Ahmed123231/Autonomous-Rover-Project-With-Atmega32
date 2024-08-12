/*
 * TIMER0_prog.c
 *
 * Created: 6/4/2024 11:15:38 AM
 *  Author: ahmed
 */ 

#include "BIT_MATH.h"
#include "STD_TYPE.h"
#include "TIMER0_reg.h"
#include "TIMER0_Interface.h"
#include "INT_Vector.h"




/************* Intializing Point To a Function Over Flow Mode **************/

static void(*OVF_ptr)(void)=0;

/************* Intializing Point To a Function CTC Mode **************/

static void(*CTC_ptr)(void)=0;

/*********** Function to Enable timer with Interrupt *****/

void TIMER0_voidInitTimerWithINTEnable(u8 copy_u8Mode){
		
		/****** 1-Prescaler Selection *********************/
		TCCR0_REG &= 0b11111000;
		TCCR0_REG |= TIMER0_PRESCALER ;
		
		
		if(copy_u8Mode == OVF_MODE){
		/****** 2- Timer Mode Selection : Overflow Mode (Normal Mode) ***/
		CLR_BIT(TCCR0_REG,3);
		CLR_BIT(TCCR0_REG,6);
		/****** 3- Enable Timer0 OverFlow Interrupt ******/
		//SET_BIT(TIMSK_REG,0);
	}
	else if (copy_u8Mode == CTC_MODE){
		/****** 2- Timer Mode Selection : CTC Mode***/
		SET_BIT(TCCR0_REG,3);
		CLR_BIT(TCCR0_REG,6);
		/**** SET OCR0 Value *****/
		OCR0_REG=CTC_VAL;
		/****** 3- Enable Timer0 CTC Interrupt ******/
		SET_BIT(TIMSK_REG,1);
		
	}
	
	
}


/********* Timer0 Call Back Function **********/


void TIMER0_voidSetCallBack(u8 copy_u8Mode ,void(*ptr)(void)){
	
	if(copy_u8Mode == OVF_MODE){
	    OVF_ptr=ptr;
	}
	else if (copy_u8Mode == CTC_MODE){
		CTC_ptr=ptr;
	}
	
}

/********* Timer0 Fast PWM Function **********/

void TIMER0_voidFastPWM(u8 copy_u8dutycycle){
	
	
	/****** 2-Prescaler Selection *********************/
	TCCR0_REG &= 0b11111000;
	TCCR0_REG |= TIMER0_PRESCALER ;
	
	SET_BIT(TCCR0_REG,3);
	SET_BIT(TCCR0_REG,6);
	SET_BIT(TCCR0_REG,5);
	CLR_BIT(TCCR0_REG,4);
	
	OCR0_REG=copy_u8dutycycle*2.56;
	
}

/********* Timer0 Phase Correct PWM Function **********/

void TIMER0_voidPCPWM(u8 copy_u8dutycycle){
	
	/****** 2-Prescaler Selection *********************/
	
	TCCR0_REG &= 0b11111000;
	TCCR0_REG |= TIMER0_PRESCALER ;
	
	SET_BIT(TCCR0_REG,6);
	CLR_BIT(TCCR0_REG,3);
	SET_BIT(TCCR0_REG,5);
	CLR_BIT(TCCR0_REG,4);
	
	OCR0_REG=copy_u8dutycycle*2.56;
	
}





ISR(_TIMER0_OVF){
	
	OVF_ptr();
}

ISR(_TIMER0_COMP){
	
	CTC_ptr();
	
}