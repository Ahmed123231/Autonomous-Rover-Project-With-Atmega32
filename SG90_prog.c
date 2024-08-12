/*
 * SG90_prog.c
 *
 * Created: 7/30/2024 3:28:38 AM
 *  Author: ahmed
 */ 

#include "BIT_MATH.h"
#include "STD_TYPE.h"
#include "TIMER1_interface.h"
#include "TIMER1_reg.h"
#include "DIO_interface.h"
#include "SG90_interface.h"




void Servo_Init(void){
	
	
	// Initialize Timer1 for Fast PWM mode
	
	DIO_voidSetPinDir(PWM_PORT,PWM_PIN,OUTPUT);
	// Set initial angle to 0 degrees
	 TIMER1_voidFastPWMICR1();
	
	
}
void Servo_SetAngle(u8 angle){
	
	u16 ocr_val =angle*23+1000 ;
	
	
	TIMER1_voidSetOCRVal(ocr_val);
	
	
}