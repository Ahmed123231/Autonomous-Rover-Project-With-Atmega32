/*
 * TIMER1_interface.h
 *
 * Created: 6/21/2024 11:18:26 AM
 *  Author: ahmed
 */ 


#ifndef TIMER1_INTERFACE_H_
#define TIMER1_INTERFACE_H_





/****** Prescaler Config ************/
#define NO_CLOCK             0
#define NO_PRESCALER         1
#define Division_8           2
#define Division_64          3
#define Division_256         4
#define Division_1023        5

/*********** TIMER1 Mode Config *********/
#define OVF_MODE             0
#define CTC_MODE             1
#define COMA_MODE            2
#define COMB_MODE            3

/*********** PWM Config *********/
#define ICR1_VAL             40000
#define OCR1_VAL             1000
#define PWMPC_8BIT           0
#define PWMPC_9BIT           1
#define PWMPC_10BIT          2

/*********** CTC Config *********/
#define CTC_VALOCR           0
#define CTC_VALICR           1
#define CTC_TOPVAL           CTC_VALOCR
#define COMA1                0
#define COMB1                1
#define COM_MODE             COMA1

/*********** Prescaler Selection *********/
#define TIMER1_PRESCALER     Division_8

/*********** Function Prototypes *********/

// Function to initialize Timer1 with interrupt enable
void TIMER1_voidInitTimerWithINTEnable(u8 copy_u8Mode);

// Function to configure Phase Correct PWM
void TIMER1_void_Fixed_PCPWM(u8 copy_u8dutycycle, u8 copy_u8mode);

// Function to set the callback function
void TIMER1_voidSetCallBack(u8 copy_u8Mode, void (*ptr)(void));

// Function to configure Fast PWM with ICR1
void TIMER1_voidFastPWMICR1(void);

// Function to set the OCR1A value
void TIMER1_voidSetOCRVal(u16 copy_u16val);

// Function to configure Fast PWM with OCR1B
void TIMER1_voidFastPWMOCRB(u8 DutyCycle);


#endif /* TIMER1_INTERFACE_H_ */