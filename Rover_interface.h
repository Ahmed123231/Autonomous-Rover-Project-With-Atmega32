/*
 * Rover_interface.h
 *
 * Created: 8/7/2024 3:01:36 AM
 *  Author: ahmed
 */ 


#ifndef ROVER_INTERFACE_H_
#define ROVER_INTERFACE_H_

#include "STD_TYPE.h"

#define CTRL1_PORT	DIO_PORTD
#define CTRL1_PIN1	DIO_PIN0
#define CTRL1_PIN2	DIO_PIN1
#define CTRL2_PORT	DIO_PORTD
#define CTRL2_PIN1	DIO_PIN2
#define CTRL2_PIN2	DIO_PIN3


#define IRB_PORT	DIO_PORTC
#define IRB_PIN		DIO_PIN0

#define IRR_PORT	DIO_PORTC
#define IRR_PIN		DIO_PIN1

#define IRL_PORT	DIO_PORTC
#define IRL_PIN		DIO_PIN2

#define IRF_PORT	DIO_PORTC
#define IRF_PIN		DIO_PIN3


typedef struct{
		u8 Distance;   // Variable Storing Distance Value 
		u8 F_Obj;	   // Storing Forward Object State	(1 -> Object Detected | 0-> No Object)
		u8 L_Obj;	   // Storing Left Object State		(1 -> Object Detected | 0-> No Object)
		u8 R_Obj;	   // Storing Right Object State	(1 -> Object Detected | 0-> No Object)
		u8 B_Obj;	   // Storing Back Object State		(1 -> Object Detected | 0-> No Object)
		u8 Direction;  // Storing Direction State		(1->Forward | 2->Backward | 3->Right | 4->Left | 5->Stop)		
	}Rover_Status;


/******************** Initialize Radar and LCD ********************/
/* Initializes the radar and LCD modules for operation. */
void Radar_LCD_Init(void);

/************** Sweep Radar and Display Angles on LCD **************/
/* Performs a radar sweep and updates the angles on the LCD.
   Takes pointers to store the angles swept. */
void Radar_LCD_Sweep(u8 *angle_1 , u8 *angle_2);

/***************** Initialize Rover Motor Control *****************/
/* Initializes the motor control system for the rover. */
void Rover_voidMotorsInit(void);

/****************** Move Rover Forward (Alternative) ******************/
/* Moves the rover forward at the specified speed (alternative function). */
void Rover_voidMOVFWD(u8 speed);

/***************** Move Rover Backward at Given Speed *****************/
/* Moves the rover backward at the specified speed. */
void Rover_voidMOVBCWD(u8 speed);

/*************** Turn Rover Right at Given Speed ****************/
/* Turns the rover to the right at the specified speed. */
void Rover_voidMOVRW(u8 Rspeed);

/*************** Turn Rover Left at Given Speed ****************/
/* Turns the rover to the left at the specified speed. */
void Rover_voidMOVLF(u8 Lspeed);

/***************** Stop Rover Movement *****************/
/* Stops all movement of the rover. */
void Rover_voidStop(void);

/*************** Display Rover Status on LCD ****************/
/* Prints the current status of the rover on the LCD.
   Takes a pointer to the Rover_Status structure. */
void ROVER_LCD_PrintStatus(Rover_Status *rover_state);

/*************** Initialize IR Sensors ***************/
/* Initializes the IR sensors used for object detection. */
void IrSensor_voidInit(void);

/*************** Get Direction from IR Sensors ***************/
/* Reads the direction from the IR sensors to determine obstacles. */
void IrSensor_GetDir(void);

/*************** Initialize Buzzer Module ***************/
/* Initializes the buzzer for sound alerts. */
void Buzzer_voidInit(void);

/************ Toggle Buzzer Sound for Alerts ************/
/* Toggles the buzzer sound on and off. */
void Buzzer_voidToggle(void);


#endif /* ROVER_INTERFACE_H_ */