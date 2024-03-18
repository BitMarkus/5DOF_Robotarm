#ifndef FUNCTIONS_H
#define FUNCTIONS_H

//Defines f�r USART
#define F_CPU 16000000UL
#define BUAD 9600
#define UBRR ((F_CPU/16/BUAD) - 1)
//Defines f�r Schieberegister (Port und Pins)
//Zwei Schieberegister hintereinander geschaltet (16 Bit)
#define HC595_PORT		PORTD
#define HC595_DDR		DDRD
#define HC595_DS_POS	PD5		//Data pin (DS oder SER)
#define HC595_SH_CP_POS PD7		//Shift Clock Pin (SH_CP oder SRCLK)
#define HC595_ST_CP_POS PD6		//Store Clock Pin (ST_CP oder RCLK)
#define SHIFTR_BITS		16		//Anzahl der Bits in beiden Schieberegistern

//Includes
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "typedef.h"
#include "const.h"

//Funktionsdeklarationen

//Umrechnung von Rad in Grad
double rad_in_grad(double rad);
double grad_in_rad(double grad);
//Arduino map() Funktion f�r double-Variablen
double mapd(double val, double in_min, double in_max, double out_min, double out_max);

//USART
//USART initialisieren
void usart_init();
//Einzelnes Zeichen �ber USART empfangen (wird nicht ben�tigt)
uint8_t usart_rec_char(void);
//String �ber USART empfangen und in Puffer ablegen (wird nicht ben�tigt)
void usart_rec_strg(char *Buffer, uint8_t MaxLen);
//Einzelnes Zeichen �ber USART an PC senden
void usart_send_char(char ch);
//String �ber USART an PC senden
void usart_send_strg(char *strg);
//Parsen des gesendeten Strings vom PC (Delimiter: ,)
//Koordinaten m�ssen in der Form: x,y,z gesendet werden
//Umwandeln der Substrings in Double-Variablen und speichern im TCP-Struct
void pars_rx_strg(char *strg, dot *TCP);

//Timer 1 f�r PWM-Signal
void pwm_init();

//ADC initialisieren (Joysticks)
void adc_init();
//ADC auslesen
uint16_t adc_read(uint8_t ch);

//Timer 0 f�r die Abtastfrequenz der Joysticks und die Servogeschwindigkeit
void ms_init();

//Vektorrechnungen
//Betrag eines Vektors berechnen
double vec_betrag(vec *v);
//Skalarprodukt von zwei Vektoren berechnen
double vec_skalarprod(vec *v1, vec *v2);
//Zwei Vektoren subtrahieren -> Vektor
vec vec_subtract(vec *v1, vec *v2);
//Ortsvektoren zweier Punkte subtrahieren -> Vektor
vec dot_subtract(dot *p1, dot *p2);

//Matrizenrechnungen
//4x4 Matrizen multiplizieren
//�bergeben werden Matrix 1 und 2
//Das Ergebnis der Multiplikation wird in Matrix 3 geschrieben
void matr_x_matr(matr m_src1, matr m_src2, matr m_dst);
//4x4 Matrix mit Punkt multiplizieren
//p_src = Punkt f�r die Multiplikation
//p_dst = Ergebnis der Multiplikation
//Punkte werden der Einfachheit halber nicht als homogene Koordinaten dargestellt (=4. Variable h)
//h ist in aller Regel = 1, ausser die Matrix ist eine Projektionsmatrix (f�r perspektivische Darstellung)
//Dann m�ssen die x,y,z-Koordinaten durch h geteilt werden
void matr_x_dot(matr m, dot *p_src, dot *p_dst);

//Kinematik
//Funktion f�r die inverse Kinematik
//�bergeben wird der gew�nschte TCP
//Die berechneten Gelenkwinkel werden im globalen Struct Ang gespeichert
//K�nnen die Winkel nicht berechnet werden, gibt die Funktion 0 zur�ck, ansonsten 1
uint8_t inv_kinematic(dot *TCP, ang *Ang);
//Funktion f�r die Vorw�rtskinematik
//�bergeben wird das Struct Ang mit den Gelenkwinkeln,
//die Transformationsmatrix und der Punkt, der transformiert werden soll
//Ausserdem der Punkt, in dem die Transformation gespeichert wird
//Der letzte Parameter x gibt an, ob Gelenkstellung 1 oder 2 verwendet werden soll
//Kann der Punkt nicht berechnet werden, gibt die Funktion 0 zur�ck, ansonsten 1
uint8_t fwd_kinematic(ang *Ang, dot *p_src, dot *p_dst, uint8_t x);

//Berechnung von Pulsl�ngen
uint16_t ang_to_puls(uint8_t s_nr, double ang, led *Led);
//Pulsl�ngen in Struct Pul speichern
//Der letzte Parameter x gibt an, ob Gelenkstellung 1 oder 2 verwendet werden soll
void set_pulse(ang *Ang, pul *Pul, uint8_t x, led *Led);
//Wandelt die Joystickeingaben in eine Pulsweite um
//j_val = analoger Wert der Joystickachse (1-1024)
//s_nr = Servonummer (1-5)
//s_puls_alt = aktueller Servopuls (�s*2)
//s_dir = gew�nschte Servorichtung (1,0)
//led *Led = Referenz zum Struct f�r die Info-LEDs
uint16_t joy_to_pulse(uint16_t j_val, uint8_t s_nr, uint16_t s_puls_alt, uint8_t s_dir, led *Led);

//Gelenkwinkel auf die Neutralstellung setzen
void reset_ang(ang *Ang);
//TCP auf die Neutralstellung setzen
void reset_tcp(dot *TCP);

//Funktionen f�r die Schieberegister (2xHC595)
//HC595 initialisieren
void shiftr_init();
//Puls auf Shift Clock Pin (SH_CP oder SRCLK)
void shiftr_shift();
//Puls auf Store Clock Pin (ST_CP oder RCLK)
void shiftr_latch();
//Zust�nde der Info-LEDs aus dem Struct "Led" Bit f�r Bit in uint16_t Variable schreiben
//Auff�llen von Rechts
//Bit 16 (links): ungenutzt, immer 0
uint16_t shiftr_data(led *Led);
//Funktion gibt den Zustand der Info-LEDs �ber zwei Schieberegister (= 16 Byte) aus
//Da nur 15 LEDs vorhanden sind, ist das letzte Byte immer 0
void shiftr_out(uint16_t data);

#endif