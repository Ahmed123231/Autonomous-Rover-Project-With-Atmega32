/*
 * EXTI_prog.c
 *
 * Created: 5/24/2024 9:06:03 PM
 *  Author: ahmed
 */ 

#include "BIT_MATH.h"
#include "STD_TYPE.h"
#include "EXTI_reg.h"
#include "EXTI_interface.h"
#include "INT_Vector.h"


/************** Intializing 3 Pointers to INT functions **********/

void(*INT_0_PTR)(void)=0;
void(*INT_1_PTR)(void)=0;
void(*INT_2_PTR)(void)=0;




/********* Function to initialize external interrupt ****/
void EXTI_voidInit(u8 copy_u8interruptNum ,u8 copy_u8SenseControl){
	
	switch(copy_u8interruptNum){
		
		case INT_0 :
				if(copy_u8SenseControl == LOW_LEVEL)
				{
					CLR_BIT(MCUCR_REG,0);
					CLR_BIT(MCUCR_REG,1);
				}
				else if (copy_u8SenseControl == ANY_LOGICAL_CHANGE)
				{
					SET_BIT(MCUCR_REG,0);
					CLR_BIT(MCUCR_REG,1);
					
				}
				else if (copy_u8SenseControl == FALLING_EDGE)
				{
					CLR_BIT(MCUCR_REG,0);
					SET_BIT(MCUCR_REG,1);
		
				}
				else if (copy_u8SenseControl == RISING_EDGE)
				{
					SET_BIT(MCUCR_REG,0);
					SET_BIT(MCUCR_REG,1);
					
				}
				else{
					
					// Do Nothing 
				}
				
		  case INT_1 :
			  	if(copy_u8SenseControl == LOW_LEVEL)
			  	{
				  	CLR_BIT(MCUCR_REG,2);
				  	CLR_BIT(MCUCR_REG,3);
			  	}
			  	else if (copy_u8SenseControl == ANY_LOGICAL_CHANGE)
			  	{
				  	SET_BIT(MCUCR_REG,2);
				  	CLR_BIT(MCUCR_REG,3);
				  	
			  	}
			  	else if (copy_u8SenseControl == FALLING_EDGE)
			  	{
				  	CLR_BIT(MCUCR_REG,0);
				  	SET_BIT(MCUCR_REG,1);
				  	
			  	}
			  	else if (copy_u8SenseControl == RISING_EDGE)
			  	{
				  	SET_BIT(MCUCR_REG,2);
				  	SET_BIT(MCUCR_REG,3);
				  	
			  	}
			  	else{
				  	
				  	// Do Nothing
			  	}	
			case INT_2 :
			
			if (copy_u8SenseControl == FALLING_EDGE)
			{
				CLR_BIT(MCUCSR_REG,6);
				
				
			}
			else if (copy_u8SenseControl == RISING_EDGE)
			{
				
				SET_BIT(MCUCSR_REG,6);
				
			}
			else{
				
				// Do Nothing
			}
					
	}
	
}

/******* Function to Enable INT 0 Pin ********/
void EXTI_voidINT0Enable(void){
	
	SET_BIT(GICR_REG,6);
	
}
/******* Function to Disable INT 0 Pin ********/
void EXTI_voidINT0Disable(void){
	
	CLR_BIT(GICR_REG,6);
	
}

/******* Function to Enable INT 1 Pin ********/
void EXTI_voidINT1Enable(void){
	
	SET_BIT(GICR_REG,7);
}
/******* Function to Disable INT 1 Pin ********/
void EXTI_voidINT1Disable(void){
	CLR_BIT(GICR_REG,7);
	
}
/******* Function to Enable INT 2 Pin ********/
void EXTI_voidINT2Enable(void){
	
	SET_BIT(GICR_REG,5);
	
}
/******* Function to Disable INT 2 Pin ********/
void EXTI_voidINT2Disable(void){
	
	CLR_BIT(GICR_REG,5);
	
}

void EXTI_SetCallBack(u8 copy_InterruptNum ,void(*ptr)(void)){
	
	if(copy_InterruptNum == INT_0)
	{
		INT_0_PTR=ptr;
	}
	if(copy_InterruptNum == INT_1)
	{
		INT_1_PTR=ptr;
	}
	if(copy_InterruptNum == INT_2)
	{
		INT_2_PTR=ptr;
	}
	
	
}


ISR(_INT_0){
	
	
		INT_0_PTR();
	
}
ISR(_INT_1){
	
	
	INT_1_PTR();
	
}
ISR(_INT_2){
	
	
	INT_2_PTR();
	
}