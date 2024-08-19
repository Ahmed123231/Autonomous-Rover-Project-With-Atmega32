/*
 * Rover_Test1.c
 *
 * Created: 7/31/2024 4:59:40 AM
 * Author : ahmed
 */ 

#include "BIT_MATH.h"
#include "STD_TYPE.h"
#include "DIO_interface.h"
#include "SG90_interface.h"
#include "TIMER0_Interface.h"
#include "TIMER0_reg.h"
#include "TIMER1_interface.h"
#include "TIMER2_interface.h"
#include "UltraSonic_interface.h"
#include "TIMER1_interface.h"
#include "TIMER1_reg.h"
#include "LCD_interface.h"
#include "LCD_config.h"
#include "Rover_interface.h"
#include "UART_interface.h"

#define F_CPU 16000000UL
#include <util/delay.h>

Rover_Status Rover_state;
int main(void)
{
			
			
			Rover_voidInit();
			
			while (1)
			{
				
				Rover_RunAlgorithm();
				
			}
			
}

