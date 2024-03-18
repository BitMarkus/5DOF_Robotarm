#include "const.h"

//Definition von globalen Konstanten

//Pi
const double PI = 3.14159265;

//Längen der Glieder [mm]
const double L_1 = 160.0;
const double L_2 = 90.0;
const double L_3 = 190.0;

//Servospezifische Konstanten
//0: Servo-Pin am Arduino (PORTB)
//1: P_0 = Pulslänge bei Winkel 0 (Neutralstellung)
//2: P_-90 = Pulslänge bei -90° (leichter zu mesen als +90°)
//3: P_Min = Minimale Pulslänge für den Servo
//4: P_Max = Maximale Pulslänge für den Servo
//Pulslängen in µs, für Timer 1 x 2 nehmen
const uint16_t SERVO_const[5][5] =
{
	//Servo G1
	{PINB0, 1573, 590, 590, 2496},
	//Servo G2
	{PINB1, 1554, 2446, 1030, 2446},
	//Servo G3
	{PINB2, 1546, 580, 580, 2496},
	//Servo G4
	{PINB3, 2300, 1146, 600, 2340},
	//Servo G5
	{PINB4, 2026, 976, 1615, 2066}
};
//Aktualisierungsrate der Servoposition in ms (24 ms)
const uint8_t SERVO_ms_refresh = 23;

//Joystickspezifische Konstanten
//Joystick Deadzone (für alle Joysticks/Achsen gleich)
//Gilt in +- x- und y-Richtung, ca. 5% der Auslenkung in jede Richtung (512 x 0,05 = 25,6)
//Verhindert automatisches Laufen der Servos, wenn die Mittelstellung der Joysticks nicht genau bei 512 ist
const uint8_t JOY_dz = 25;
//Joystick Mittelstellung
//10 bit = (1024/2) - 1 = 511
const uint16_t JOY_mid = 511;
//Testkonstante für die Servogeschwindigkeit
//Je höher, desto langsamer die Servos
const uint8_t JOY_div = 80;
//Abtastrate der Joysticks in ms (12 ms)
const uint8_t JOY_ms_refresh = 11;

