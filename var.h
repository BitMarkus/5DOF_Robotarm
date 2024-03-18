#ifndef VAR_H
#define VAR_H

//Defines
#define USART_RX_MAX 100

//Includes
#include "typedef.h"

//Deklaration von globalen Variablen

//Tool Center Point = gew�nschte Effektorposition
dot TCP;
//Gelenkwinkel des Roboters
ang Ang;
//(modifizierte) Pulsweite f�r die Servos
pul Pul;
//ADC-Werte der drei Joysticks (x- und y-Achsen)
volatile adcj adcJ;
//berechneter TCP �ber Vorw�rtskinematik
dot TCP_calc;
//Nullpunkt des Weltkoordinatensystems
dot P_Zero;
//USART Empfangspuffer f�r die Koordinaten
char usart_rx_buffer[USART_RX_MAX + 1];
//Variable gibt an, ob der String mit den Koordinaten �ber USART komplett empfangen wurde
//1 = komplett, 0 = noch nicht komplett
volatile uint8_t usart_strg_compl;
//Z�hler f�r die Anzahl an �bergebenen Zeichen �ber USART
volatile uint8_t usart_strg_count;
//Z�hler f�r ms f�r die Abtastrate der Joysticks (Timer 0)
volatile uint8_t Joy_ms_count;
//Z�hler f�r ms f�r die Aktualisierungsrate der Servoposition (Timer 0)
volatile uint8_t Serv_ms_count;
//Struct f�r den Zustand der Info-LEDs
led Led;

#endif