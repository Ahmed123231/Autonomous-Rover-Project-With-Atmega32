/*
 * SG90_interface.h
 *
 * Created: 7/30/2024 3:28:20 AM
 *  Author: ahmed
 */ 


#ifndef SG90_INTERFACE_H_
#define SG90_INTERFACE_H_


#define PWM_PORT       DIO_PORTD
#define PWM_PIN        DIO_PIN5

void Servo_Init(void);
void Servo_SetAngle(u8 angle);


#endif /* SG90_INTERFACE_H_ */