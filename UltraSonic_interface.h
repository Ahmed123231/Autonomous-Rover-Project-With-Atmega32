/*
 * UltraSonic_interface.h
 *
 * Created: 8/5/2024 9:55:18 PM
 *  Author: ahmed
 */ 


#ifndef ULTRASONIC_INTERFACE_H_
#define ULTRASONIC_INTERFACE_H_



#define TRIG_PORT			DIO_PORTC
#define TRIG_PIN			DIO_PIN4
#define ECHO_PORT			DIO_PORTC
#define ECHO_PIN			DIO_PIN5
#define SOUND_SPEED 34600UL // Speed of sound in cm/s at room temperature
#define PRESCALER 64
#define OFFSET_CM 17 // Offset to correct the measurement




void UltraSonic_Sendpulse(void);
u32 UltraSonic_Read_Echo(void);
u8 UltraSonic_Calc_Distance(u32(*FuncPtr)(void));



#endif /* ULTRASONIC_INTERFACE_H_ */