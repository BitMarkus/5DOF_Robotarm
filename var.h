#ifndef VAR_H
#define VAR_H

//Defines
#define USART_RX_MAX 100

//Includes
#include "typedef.h"

//Deklaration von globalen Variablen

//Tool Center Point = gewünschte Effektorposition
dot TCP;
//Gelenkwinkel des Roboters
ang Ang;
//(modifizierte) Pulsweite für die Servos
pul Pul;
//ADC-Werte der drei Joysticks (x- und y-Achsen)
volatile adcj adcJ;
//berechneter TCP über Vorwärtskinematik
dot TCP_calc;
//Nullpunkt des Weltkoordinatensystems
dot P_Zero;
//USART Empfangspuffer für die Koordinaten
char usart_rx_buffer[USART_RX_MAX + 1];
//Variable gibt an, ob der String mit den Koordinaten über USART komplett empfangen wurde
//1 = komplett, 0 = noch nicht komplett
volatile uint8_t usart_strg_compl;
//Zähler für die Anzahl an übergebenen Zeichen über USART
volatile uint8_t usart_strg_count;
//Zähler für ms für die Abtastrate der Joysticks (Timer 0)
volatile uint8_t Joy_ms_count;
//Zähler für ms für die Aktualisierungsrate der Servoposition (Timer 0)
volatile uint8_t Serv_ms_count;
//Struct für den Zustand der Info-LEDs
led Led;

#endif