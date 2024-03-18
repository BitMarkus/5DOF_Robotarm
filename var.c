#include "var.h"

//Definition von globalen Variablen

//Tool Center Point = gewünschte Effektorposition
dot TCP = {0, 0, 0};
//Gelenkwinkel des Roboters
ang Ang = {0, 0, 0, 0, 0, 0, 0};
//(modifizierte) Pulsweite für die Servos
pul Pul = {0, 0, 0, 0, 0};
//ADC-Werte der drei Joysticks (x- und y-Achsen)
adcj adcJ = {0, 0, 0, 0, 0, 0};
//berechneter TCP über Vorwärtskinematik
dot TCP_calc = {0, 0, 0};
//Nullpunkt des Weltkoordinatensystems
dot P_Zero = {0, 0, 0};
//USART Empfangspuffer für die Koordinaten
char usart_rx_buffer[USART_RX_MAX + 1] = "";
//Variable gibt an, ob der String mit den Koordinaten über USART komplett empfangen wurde
//1 = komplett, 0 = noch nicht komplett
uint8_t usart_strg_compl = 0;
//Zähler für die Anzahl an übergebenen Zeichen über USART
uint8_t usart_strg_count = 0;
//Zähler für ms für die Abtastrate der Joysticks (Timer 0)
uint8_t Joy_ms_count = 0;
//Zähler für ms für die Aktualisierungsrate der Servoposition (Timer 0)
uint8_t Serv_ms_count = 0;
//Struct für den Zustand der Info-LEDs
led Led = {{1, 1, 1, 1, 1}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};