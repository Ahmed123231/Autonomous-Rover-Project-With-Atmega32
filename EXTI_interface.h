/*
 * EXTI_interface.h
 *
 * Created: 5/24/2024 11:52:18 PM
 *  Author: ahmed
 */ 


#ifndef EXTI_INTERFACE_H_
#define EXTI_INTERFACE_H_

/******** Interrupt Pins **********/
#define INT_0						0
#define INT_1						1
#define INT_2						2


/******** Sense Control **************/

#define RISING_EDGE                 0
#define FALLING_EDGE                1
#define ANY_LOGICAL_CHANGE          2
#define LOW_LEVEL                   3

/********* Function to initialize external interrupt ****/
void EXTI_voidInit(u8 copy_u8interruptNum ,u8 copy_u8SenseControl);

/******* Function to Enable INT 0 Pin ********/
void EXTI_voidINT0Enable(void);
/******* Function to Disable INT 0 Pin ********/
void EXTI_voidINT0Disable(void);

/******* Function to Enable INT 1 Pin ********/
void EXTI_voidINT1Enable(void);
/******* Function to Disable INT 1 Pin ********/
void EXTI_voidINT1Disable(void);
/******* Function to Enable INT 2 Pin ********/
void EXTI_voidINT2Enable(void);
/******* Function to Disable INT 2 Pin ********/
void EXTI_voidINT2Disable(void);

/********* Function to Set call back a function ************/

void EXTI_SetCallBack(u8 copy_InterruptNum , void(*ptr)(void));




#endif /* EXTI_INTERFACE_H_ */