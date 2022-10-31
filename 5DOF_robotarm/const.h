#ifndef CONST_H
#define CONST_H

#include <avr/io.h>

//Deklaration von globalen Konstanten

//Pi
extern const double PI;

//L�ngen der Glieder [mm]
extern const double L_1;
extern const double L_2;
extern const double L_3;

//Servospezifische Konstanten
//0: Servo-Pin am Arduino (PORTB)
//1: P_0 = Pulsl�nge bei Winkel 0 (Neutralstellung)
//2: P_-90 = Pulsl�nge bei -90 Grad
//3: P_Min = Minimale Pulsl�nge f�r den Servo
//4: P_Max = Maximale Pulsl�nge f�r den Servo
extern const uint16_t SERVO_const[5][5];
//Aktualisierungsrate der Servoposition in ms
extern const uint8_t SERVO_ms_refresh;

//Joystickspezifische Konstanten
//Joystick Deadzone (f�r alle Joysticks/Achsen gleich)
//Gilt in +- x- und y-Richtung, ca. 5% der Auslenkung in jede Richtung (512x0,05=25,6)
//Verhindert automatisches Laufen der Servos, wenn die Mittelstellung der Joysticks nicht genau bei 512 ist
extern const uint8_t JOY_dz;
//Joystick Mittelstellung
//10 bit = (1024/2) - 1 = 511
extern const uint16_t JOY_mid;
//Testkonstante f�r die Servogeschwindigkeit
extern const uint8_t JOY_div;
//Abtastrate der Joysticks in ms
extern const uint8_t JOY_ms_refresh;

#endif