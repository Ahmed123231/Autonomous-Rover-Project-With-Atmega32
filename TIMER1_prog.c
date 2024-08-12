/*
 * TIMER1_prog.c
 *
 * Created: 6/21/2024 11:18:54 AM
 *  Author: ahmed
 */
#include "BIT_MATH.h"
#include "STD_TYPE.h"
#include "TIMER1_reg.h"
#include "TIMER1_interface.h"
#include "INT_Vector.h"
 
/************* Intializing Point To a Function Over Flow Mode **************/

static void(*OVF_ptr)(void)=0;

/************* Intializing Point To a Function CTC COMPARE A Mode **************/

static void(*CTC_COMA_ptr)(void)=0;
 
/************* Intializing Point To a Function CTC COMPARE B Mode **************/

static void(*CTC_COMB_ptr)(void)=0;
 
 
 
 /*********** Function to Enable timer with Interrupt *****/

 void TIMER1_voidInitTimerWithINTEnable(u8 copy_u8Mode){
	 
	 
	 if (copy_u8Mode == OVF_MODE)
	 {
		 /****** 2- Timer Mode Selection : Overflow Mode (Normal Mode) ***/
		 CLR_BIT(TCCR1A_REG,0);
		 CLR_BIT(TCCR1A_REG,1);
		 CLR_BIT(TCCR1B_REG,3);
		 CLR_BIT(TCCR1B_REG,4);
		 /****** 3- Enable Timer1 OverFlow Interrupt ******/
		 //SET_BIT(TIMSK_REG,2);
	 }
	 else if (copy_u8Mode == CTC_MODE)
	 {
		 /*********** Wave Form Generation MODE BIT **********/
		 #if CTC_TOPVAL == CTC_VALOCR
		 /*********Compare OUTPUT MODE**********/
		 if(COM_MODE == COMA1){
			 
			 CLR_BIT(TCCR1A_REG,6);
			 SET_BIT(TCCR1A_REG,7);
			 SET_BIT(TIMSK_REG,4);
		 }
		 else if (COM_MODE == COMB1)
		 {
			 CLR_BIT(TCCR1A_REG,4);
			 SET_BIT(TCCR1A_REG,5);
			 SET_BIT(TIMSK_REG,3);
		 }
		 /********* CTC TOP VAL OCR1A **********/
		 CLR_BIT(TCCR1A_REG,0);
		 CLR_BIT(TCCR1A_REG,1);
		 SET_BIT(TCCR1B_REG,3);
		 CLR_BIT(TCCR1B_REG,4);
		 
		 OCR1A_REG=OCR1_VAL;
		 
		 #elif CTC_TOPVAL == CTC_VALOCR
		 
		 /*********Compare OUTPUT MODE**********/
		 if(COM_MODE == COMA1){
			 
			 CLR_BIT(TCCR1A_REG,6);
			 SET_BIT(TCCR1A_REG,7);
			 SET_BIT(TIMSK_REG,4);
		 }
		 else if (COM_MODE == COMB1)
		 {
			 CLR_BIT(TCCR1A_REG,4);
			 SET_BIT(TCCR1A_REG,5);
			 SET_BIT(TIMSK_REG,3);
		 }
		 /********* CTC TOP VAL ICR1 **********/
		 CLR_BIT(TCCR1A_REG,0);
		 CLR_BIT(TCCR1A_REG,1);
		 SET_BIT(TCCR1B_REG,3);
		 SET_BIT(TCCR1B_REG,4);
		 
		 ICR1_REG=ICR1_VAL;
		 
		 #endif
	 }
	 
	/*********** 1-Select Prescaler ***************/
		TCCR1B_REG &=0b11111000;
		TCCR1B_REG |= TIMER1_PRESCALER; 
	 
	 
 }

/********* Timer1 Call Back Function **********/

void TIMER1_voidSetCallBack(u8 copy_u8Mode ,void(*ptr)(void)){
	
		if(copy_u8Mode == OVF_MODE)
		{
			OVF_ptr=ptr;
		}
		else if (copy_u8Mode == COMA_MODE)
		{
			CTC_COMA_ptr=ptr;
		}
		else if (copy_u8Mode == COMB_MODE)
		{
			CTC_COMB_ptr=ptr;
		}
	
	
	
	
}


 /********* Timer1 Fixed resolution Phase Correct PWM Function **********/

void TIMER1_void_Fixed_PCPWM(u8 copy_u8dutycycle , u8 copy_u8mode){
		
		/*********** 1-Select Prescaler ***************/
		TCCR1B_REG &=0b11111000;
		TCCR1B_REG |= TIMER1_PRESCALER ;
		
		
		if(copy_u8mode == PWMPC_8BIT)
		{
		SET_BIT(TCCR1A_REG,0);
		CLR_BIT(TCCR1A_REG,1);
		CLR_BIT(TCCR1B_REG,3);
		CLR_BIT(TCCR1B_REG,4);
		}
		else if (copy_u8mode == PWMPC_9BIT)
		{
		CLR_BIT(TCCR1A_REG,0);
		SET_BIT(TCCR1A_REG,1);
		CLR_BIT(TCCR1B_REG,3);
		CLR_BIT(TCCR1B_REG,4);
		}
		else if (copy_u8mode == PWMPC_10BIT)
		{
		SET_BIT(TCCR1A_REG,0);
		SET_BIT(TCCR1A_REG,1);
		CLR_BIT(TCCR1B_REG,3);
		CLR_BIT(TCCR1B_REG,4);
		}
		
	
	
	
}
 
		
 
 
 
 
 
 
 
void TIMER1_voidFastPWMICR1(void){
	
	/********* Select Mode Fast PWM ICR1 **********/
	CLR_BIT(TCCR1A_REG,0);
	SET_BIT(TCCR1A_REG,1);
	SET_BIT(TCCR1B_REG,3);
	SET_BIT(TCCR1B_REG,4);
	/********* Fast PWM :NON Intverting Mode ********/
	SET_BIT(TCCR1A_REG,7);
	CLR_BIT(TCCR1A_REG,6);
	SET_BIT(TCCR1A_REG,5);
	CLR_BIT(TCCR1A_REG,4);
	/*********** Select Prescaler ***************/
	 
	 CLR_BIT(TCCR1B_REG,0) ;
	 SET_BIT(TCCR1B_REG,1) ;
	 CLR_BIT(TCCR1B_REG,2) ;
	/***** SET TOP VALUE 40000 --> F=50Hz / P=20ms ***************/
	ICR1_REG=40000;
	OCR1B_REG=255;
	
	
}

void TIMER1_voidSetOCRVal(u16 copy_u16val){
	
	OCR1A_REG=copy_u16val;
	
}

void TIMER1_voidFastPWMOCRB(u8 DutyCycle){
	
	
	OCR1B_REG =((DutyCycle * 255 / 100));
	
	
	
	
}