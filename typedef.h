#ifndef TYPEDEF_H
#define TYPEDEF_H

//Typedef f�r Vektoren im 3D-Raum
typedef struct vector
{
	double x;
	double y;
	double z;
} vec;

//Typedef f�r Punkte im 3D-Raum
typedef struct point
{
	double x;
	double y;
	double z;
} dot;

//Typedef f�r 4x4 Matrizen
typedef double matr[4][4];

//Typedef f�r Struct, welches die Gelenkwinkel enth�lt
//Angabe in Rad
//_21/_22 bzw. _31/_32 beziehen sich auf zwei M�glichkeiten
//der Gelenkstellungen, um den TCP zu erreichen
typedef struct angles
{
	double Phi_1;
	double Phi_21;
	double Phi_22;
	double Phi_31;
	double Phi_32;
	double Phi_4;
	double Phi_5;
} ang;

//Typedef f�r Struct, dass die Pulsweiten f�r die Servos enth�lt
//Pulsweiten in �s m�ssen als Eintrag in TCNT1 x 2 genommen werden
typedef struct pulsewidth
{
	uint16_t pul_1;
	uint16_t pul_2;
	uint16_t pul_3;
	uint16_t pul_4;
	uint16_t pul_5;
} pul;

//Typedef f�r Struct, dass die analogen Werte f�r die drei Joysticks enth�lt
typedef struct ADC_joystick
{
	uint16_t joy_1_x;
	uint16_t joy_1_y;
	uint16_t joy_2_x;
	uint16_t joy_2_y;
	uint16_t joy_3_x;
	uint16_t joy_3_y;
} adcj;

//Typedef f�r Struct, dass den Zustand der Info-LEDs enth�lt (0 oder 1)
typedef struct LED_info
{
	uint8_t led_g[5];
	uint8_t led_y[5];
	uint8_t led_r[5];
} led;

#endif