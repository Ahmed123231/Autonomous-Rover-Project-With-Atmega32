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
// 			Radar_LCD_Init();
// 			Rover_voidMotorsInit();
// 			IrSensor_voidInit();
// 			Buzzer_voidInit();
// 			RoverTransmit_Init();
// 			Buzzer_voidToggle();
			u8 angle_1=0;
			u8 angle_2=90;
			
			while (1)
			{
				
				
				Rover_RunAlgorithm();
				/*
				IrSensor_GetDir(); // Update IR sensor readings
				UltraSonic_Sendpulse(); // Send ultrasonic pulse
				Rover_state.Distance = UltraSonic_Calc_Distance(UltraSonic_Read_Echo); // Calculate distance
				ROVER_LCD_PrintStatus(&Rover_state);
				RoverTransmitStatus(); // Sending Rover Status with Bluetooth
				// Check if the front is clear (IR sensor or ultrasonic distance)
				if (Rover_state.F_Obj == 1 && Rover_state.Distance > 10)
				{
					// Move forward
					Rover_voidMOVFWD(50);
					ROVER_LCD_PrintStatus(&Rover_state);
					RoverTransmitStatus();
					// Continue moving forward while the path is clear
					while (Rover_state.F_Obj == 1 && Rover_state.Distance > 10)
					{
						IrSensor_GetDir();
						UltraSonic_Sendpulse();
						Rover_state.Distance = UltraSonic_Calc_Distance(UltraSonic_Read_Echo);
						ROVER_LCD_PrintStatus(&Rover_state);
						RoverTransmitStatus(); // Sending Rover Status with Bluetooth
						// If an object is detected in front, stop the rover
						if (Rover_state.F_Obj != 1 || Rover_state.Distance <= 10)
						{
							Rover_voidStop();
							Buzzer_voidToggle();
							break;
						}

						_delay_ms(100);
					}
				}
				else
				{
					// Object detected in front, take a step back
					Rover_voidMOVBCWD(55);
					_delay_ms(500); // Delay to move back for a short period
					Rover_voidStop();
					
					Buzzer_voidToggle();

					// Re-check sensors after taking a step back
					IrSensor_GetDir();
					UltraSonic_Sendpulse();
					Rover_state.Distance = UltraSonic_Calc_Distance(UltraSonic_Read_Echo);
					ROVER_LCD_PrintStatus(&Rover_state);
					RoverTransmitStatus(); // Sending Rover Status with Bluetooth
					// Decide whether to turn right or left based on sensor readings
					if (Rover_state.L_Obj != 1 && Rover_state.R_Obj == 1)
					{
						// Left is blocked, turn right
						Rover_voidMOVRW(85);
						_delay_ms(1500);
						
						Buzzer_voidToggle();
						ROVER_LCD_PrintStatus(&Rover_state);
						RoverTransmitStatus(); // Sending Rover Status with Bluetooth
					}
					else if (Rover_state.R_Obj != 1 && Rover_state.L_Obj == 1)
					{
						// Right is blocked, turn left
						Rover_voidMOVLF(85);
						_delay_ms(1500);
						Buzzer_voidToggle();
						ROVER_LCD_PrintStatus(&Rover_state);
						RoverTransmitStatus(); // Sending Rover Status with Bluetooth
					}
					else if (Rover_state.L_Obj == 1)
					{
						// Prefer to turn left if both left and right are clear
						Rover_voidMOVLF(85);
						_delay_ms(1500);
						Buzzer_voidToggle();
						ROVER_LCD_PrintStatus(&Rover_state);
						RoverTransmitStatus(); // Sending Rover Status with Bluetooth
					}
					else if (Rover_state.R_Obj == 1)
					{
						// Turn right if left is blocked and right is clear
						Rover_voidMOVRW(85);
						_delay_ms(1500);
						Buzzer_voidToggle();
						ROVER_LCD_PrintStatus(&Rover_state);
						RoverTransmitStatus(); // Sending Rover Status with Bluetooth
					}

					// After turning, move forward if the front is clear
					_delay_ms(500); // Allow time for the turn to complete
					Rover_voidStop();
					IrSensor_GetDir();
					UltraSonic_Sendpulse();
					Rover_state.Distance = UltraSonic_Calc_Distance(UltraSonic_Read_Echo);
					ROVER_LCD_PrintStatus(&Rover_state);
					RoverTransmitStatus(); // Sending Rover Status with Bluetooth
					_delay_ms(50);

					if (Rover_state.F_Obj == 1 && Rover_state.Distance > 10)
					{
						Rover_voidMOVFWD(55);
						ROVER_LCD_PrintStatus(&Rover_state);
						RoverTransmitStatus(); // Sending Rover Status with Bluetooth
						_delay_ms(50);
					}
					else
					{
						// If both forward and back are blocked, turn left or right based on availability
						if ((Rover_state.F_Obj != 1 || Rover_state.Distance < 10) && Rover_state.B_Obj != 1)
						{
							Rover_voidStop();
							Rover_voidMOVLF(85); // Turn left
							_delay_ms(1500);
							Radar_LCD_Sweep(&angle_1, &angle_2);
							Buzzer_voidToggle();
							ROVER_LCD_PrintStatus(&Rover_state);
							RoverTransmitStatus(); // Sending Rover Status with Bluetooth
							_delay_ms(50);

							// Check if left is clear while turning
							while (Rover_state.L_Obj == 1)
							{
								IrSensor_GetDir();
								ROVER_LCD_PrintStatus(&Rover_state);
								RoverTransmitStatus(); // Sending Rover Status with Bluetooth
								_delay_ms(50);
								// If left is blocked, stop turning left
								if (Rover_state.L_Obj == 1)
								{
									Rover_voidStop();
									Buzzer_voidToggle();
									Radar_LCD_Sweep(&angle_1, &angle_2);
									ROVER_LCD_PrintStatus(&Rover_state);
									RoverTransmitStatus(); // Sending Rover Status with Bluetooth
									_delay_ms(50);
									break;
								}

								_delay_ms(100);
							}

							// If left is blocked, turn right instead
							if (Rover_state.L_Obj == 1)
							{
								Rover_voidMOVRW(85); // Turn right if left is blocked
								_delay_ms(1500);
								Radar_LCD_Sweep(&angle_1, &angle_2);
								ROVER_LCD_PrintStatus(&Rover_state);
								RoverTransmitStatus(); // Sending Rover Status with Bluetooth
								_delay_ms(50);
								// Continue turning right until a path is clear
								while (Rover_state.R_Obj == 1)
								{
									IrSensor_GetDir();
									ROVER_LCD_PrintStatus(&Rover_state);
									RoverTransmitStatus(); // Sending Rover Status with Bluetooth

									// Stop turning right if the right path is clear
									if (Rover_state.R_Obj != 1)
									{
										Rover_voidStop();
										Buzzer_voidToggle();
										Radar_LCD_Sweep(&angle_1, &angle_2);
										ROVER_LCD_PrintStatus(&Rover_state);
										RoverTransmitStatus(); // Sending Rover Status with Bluetooth
										_delay_ms(50);
										break;
									}

									_delay_ms(100);
								}

								// Move in the clear direction after turning right
								if (Rover_state.F_Obj == 1 || Rover_state.Distance > 10)
								{
									Rover_voidMOVFWD(55); // Move forward
									Buzzer_voidToggle();
									ROVER_LCD_PrintStatus(&Rover_state);
									RoverTransmitStatus(); // Sending Rover Status with Bluetooth
									_delay_ms(50);
								}
								else if (Rover_state.B_Obj == 1)
								{
									Rover_voidMOVBCWD(55); // Move backward
									Buzzer_voidToggle();
									ROVER_LCD_PrintStatus(&Rover_state);
									RoverTransmitStatus(); // Sending Rover Status with Bluetooth
									_delay_ms(50);
								}
							}
						}
						else
						{
							// If only the front is blocked, move backward
							Rover_voidMOVBCWD(55);
							ROVER_LCD_PrintStatus(&Rover_state);
							RoverTransmitStatus(); // Sending Rover Status with Bluetooth
							_delay_ms(50);
							UltraSonic_Sendpulse();
							Rover_state.Distance = UltraSonic_Calc_Distance(UltraSonic_Read_Echo);
							ROVER_LCD_PrintStatus(&Rover_state);
							RoverTransmitStatus(); // Sending Rover Status with Bluetooth

							// Continue moving Backwards while the path is blocked
							while ((Rover_state.F_Obj != 1 || Rover_state.Distance < 10) || Rover_state.B_Obj == 1)
							{
								IrSensor_GetDir();
								UltraSonic_Sendpulse();
								Rover_state.Distance = UltraSonic_Calc_Distance(UltraSonic_Read_Echo);
								ROVER_LCD_PrintStatus(&Rover_state);
								RoverTransmitStatus(); // Sending Rover Status with Bluetooth

								// If both directions are clear  Prefered  Direction is forward
								if (Rover_state.F_Obj == 1 && Rover_state.B_Obj != 1 && Rover_state.Distance > 10)
								{
									Rover_voidStop();
									Radar_LCD_Sweep(&angle_1, &angle_2);
									Rover_voidMOVFWD(55);
									ROVER_LCD_PrintStatus(&Rover_state);
									RoverTransmitStatus(); // Sending Rover Status with Bluetooth
									_delay_ms(50);
									Buzzer_voidToggle();
									break;
								}

								_delay_ms(100); // Delay for sensor Readings update
							}
						}
					}
				}

				_delay_ms(100);
				*/
				
			}
			
}

