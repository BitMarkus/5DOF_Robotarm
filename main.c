//µC-Kurs, WS 21/22
//Projektarbeit Markus Reichold
//5DOF-Roboterarm

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "typedef.h"
#include "functions.h"
#include "var.h"

//Defines
#define USART_RX_MAX 100

//////////
// Main //
//////////

int main(void)
{	
	//PWM-Signal (Timer 1) initialisieren
	pwm_init();	
	//USART initialisieren
	usart_init();	
	//ADC für die Joysticks initialisieren
	adc_init();	
	//Timer 0 für die Abtastfrequenz der Joysticks und die Servogeschwindigkeit initialisieren
	ms_init();
	//Schieberegister (2xHC595) initialisieren
	shiftr_init();
	//Interrupts aktivieren
	sei();	
	
	//Gelenkwinkel und TCP auf Neutralstellung setzen
	reset_ang(&Ang);
	reset_tcp(&TCP);
	//Gelenkwinkel in Pulsweite (*2) umrechnen
	//und im Strut Pul speichern
	//Letzter Parameter gibt an, welche der zwei möglichen Gelenkkonfigurationen verwendet werden soll
	set_pulse(&Ang, &Pul, 1, &Led);
	
	//Eingabeaufforderung
	//Bei folgender Art der String-Initialisierung wird das \0 vom Compiler automatisch angehängt
	char text1[] = "Koordinaten [mm] im Format: x,y,z eingeben:\n\r";	
	usart_send_strg(text1);	
	
	//////////
	// Loop //
	//////////
	
    while(1) 
    {
		///////////////////////////
		// PWM Signalgenerierung //
		///////////////////////////
		
		//Servo-Pins auf LOW setzen, wenn
		//1) der gewünschte Wert in TCNT1 erreicht ist (bestimmt Pulslänge)
		//2) der Pin auf HIGH gesetzt ist
		//3) die Pulslänge in dem Bereich liegt, wo das Umschalten nötig ist (500-2600 x2 ms)		
		if(TCNT1 >= 1000 && TCNT1 <= 5200)
		{	
			//Pin B0: Servo 1 (Phi_1):
			if(TCNT1 >= Pul.pul_1 && bit_is_set(PORTB, SERVO_const[0][0]))
			{
				PORTB &= ~(1<<SERVO_const[0][0]);
			}
			//Pin B1: Servo 2 (Phi_2):
			if(TCNT1 >= Pul.pul_2 && bit_is_set(PORTB, SERVO_const[1][0]))
			{
				PORTB &= ~(1<<SERVO_const[1][0]);
			}
			//Pin B2: Servo 3 (Phi_3):
			if(TCNT1 >= Pul.pul_3 && bit_is_set(PORTB, SERVO_const[2][0]))
			{
				PORTB &= ~(1<<SERVO_const[2][0]);
			}	
			//Pin B3: Servo 4 (Phi_4):
			if(TCNT1 >= Pul.pul_4 && bit_is_set(PORTB, SERVO_const[3][0]))
			{
				PORTB &= ~(1<<SERVO_const[3][0]);
			}	
			//Pin B4: Servo 5 (Greifarm, Phi_5):
			if(TCNT1 >= Pul.pul_5 && bit_is_set(PORTB, SERVO_const[4][0]))
			{
				PORTB &= ~(1<<SERVO_const[4][0]);
			}	
		}
		
		//Anderer Code soll nur ausgeführt werden, wenn die Pulslänge außerhalb von 500-2600 (x2) ms liegt
		//und die Servoposition nicht gesetzt werden muss -> sorgt für bessere Geauigkeit beim PWM-Signal
		if(TCNT1 < 1000 || TCNT1 > 5200)
		{
			/////////////////////////////////
			// Manueller Modus (Joysticks) //
			/////////////////////////////////

			//Alle x ms die Pulsweite erhöhen/erniedrigen
			//Das Delta der Erhöhung/Erniedrigung wird über die Joystickauslenkung bestimmt (Servogeschwindigkeit)
			if(Serv_ms_count >= SERVO_ms_refresh)
			{
				Pul.pul_1 = joy_to_pulse(adcJ.joy_1_x, 1, Pul.pul_1, 0, &Led);
				Pul.pul_2 = joy_to_pulse(adcJ.joy_1_y, 2, Pul.pul_2, 1, &Led);
				Pul.pul_3 = joy_to_pulse(adcJ.joy_2_y, 3, Pul.pul_3, 0, &Led);
				Pul.pul_4 = joy_to_pulse(adcJ.joy_2_x, 4, Pul.pul_4, 0, &Led);
				Pul.pul_5 = joy_to_pulse(adcJ.joy_3_x, 5, Pul.pul_5, 0, &Led);
				
				//Info-LEDs anzeigen
				shiftr_out(shiftr_data(&Led));
				
				//Reset Counter
				Serv_ms_count = 0;				
			}
			
			//Alle x ms die Joystick-Eingabe speichern
			if(Joy_ms_count >= JOY_ms_refresh)
			{
				//ADC-Signal vom Joystick von 0-1023
				//Joysick-Mittelstellung bei 511
				//Joystick x-Achsen: links 0-511, rechts 512-1023
				//Joystick y-Achsen: oben 512-1023, unten 0-511
						
				//Joystick 1:
				adcJ.joy_1_x = adc_read(0);
				adcJ.joy_1_y = adc_read(1);		
				//Joystick 2:
				adcJ.joy_2_x = adc_read(2);				
				adcJ.joy_2_y = adc_read(3);				
				//Joystick 3:
				adcJ.joy_3_x = adc_read(4);			
				
				//Reset Counter
				Joy_ms_count = 0;	
			}

			///////////////////////////////////////
			// Automatischer Modus (Koordinaten) //
			///////////////////////////////////////
			
			//Wenn Koordinaten über USART erfolgreich eingelesen wurden
			if(usart_strg_compl == 1)
			{
				//Parsen des Strings und setzen der TCP-Koordinaten
				pars_rx_strg(usart_rx_buffer, &TCP);
				//Ausgabe der Koordinaten
				char text2[60];
				sprintf(text2, "Eingegebene Koordinaten: x=%.2f, y=%.2f, z=%.2f\n\r", TCP.x ,TCP.y ,TCP.z);
				usart_send_strg(text2);			
				
				//Gelenkwinkel über inverse Kinematik berechnen
				if(!inv_kinematic(&TCP, &Ang))
				{
					//Wenn die Berechnung der Gelenkwinkel nicht möglich ist:

					//Gelenkwinkel und TCP auf Neutralstellung setzen
					//reset_ang(&Ang);	
					//reset_tcp(&TCP);
					
					//Info-LEDs: Alle Rot
					for(uint8_t i=0; i<5; i++)
					{
						Led.led_g[i] = 0;
						Led.led_y[i] = 0;
						Led.led_r[i] = 1;
					}
					//Warnmeldung senden
					char text3[] = "Berechnung der Gelenkwinkel nicht moeglich!\n\r";
					usart_send_strg(text3);
				}
				else
				{
					//TCP über Vorwärtskinematik berechnen und mit ursprünglichen TCP vergleichen
					if(!fwd_kinematic(&Ang, &P_Zero, &TCP_calc, 1))
					{
						//Wenn der TCP nicht über die Vorwärtskinematik zurückgerechnet werden konnte:

						//Gelenkwinkel und TCP auf Neutralstellung setzen
						//reset_ang(&Ang);
						//reset_tcp(&TCP);
						
						//Info-LEDs: Alle Rot
						for(uint8_t i=0; i<5; i++)
						{
							Led.led_g[i] = 0;
							Led.led_y[i] = 0;
							Led.led_r[i] = 1;
						}	
						//Warnmeldung senden
						char text4[] = "Berechnung der Vorwaertskinematik nicht moeglich!\n\r";
						usart_send_strg(text4);
					}
					else
					{
						//Vergleich mit Toleranz (0,01 mm)
						double tol = 0.01;
						if(TCP_calc.x < (TCP.x-tol) || TCP_calc.x > (TCP.x+tol) ||
						   TCP_calc.y < (TCP.y-tol) || TCP_calc.y > (TCP.y+tol) ||
						   TCP_calc.z < (TCP.z-tol) || TCP_calc.z > (TCP.z+tol))
						{
							//Wenn der vorgegebene TCP vom berechneten TCP abweicht:

							//Gelenkwinkel und TCP auf Neutralstellung setzen
							//reset_ang(&Ang);
							//reset_tcp(&TCP);
							
							//Info-LEDs: Alle Rot
							for(uint8_t i=0; i<5; i++)
							{
								Led.led_g[i] = 0;
								Led.led_y[i] = 0;
								Led.led_r[i] = 1;
							}
							//Warnmeldung senden
							char text5[] = "Fehler bei der Berechnung der Vorwaertskinematik!\n\r";
							usart_send_strg(text5);
						}
						else
						{ 
							//Info-LEDs: Alle grün
							for(uint8_t i=0; i<5; i++)
							{
								Led.led_g[i] = 1;
								Led.led_y[i] = 0;
								Led.led_r[i] = 0;
							}
							
							//Gelenkwinkel erfolgreich gesetzt
							//grüne LED leuchten lassen
							//Erfolgsmeldung ausgeben
							char text6[120];
							sprintf(text6, "Berechnete Gelenkwinkel: Phi_1=%.2f, Phi_21=%.2f, Phi_22=%.2f, Phi_31=%.2f, Phi_32=%.2f\n\r",
							rad_in_grad(Ang.Phi_1), 
							rad_in_grad(Ang.Phi_21), 
							rad_in_grad(Ang.Phi_22), 
							rad_in_grad(Ang.Phi_31), 
							rad_in_grad(Ang.Phi_32));
							usart_send_strg(text6);
							
							//Ausgabe des berechneten TCPs über die Vorwärtskinematik
							char text7[60];
							sprintf(text7, "Berechneter TCP: x=%.2f, y=%.2f, z=%.2f\n\r",							
							TCP_calc.x,
							TCP_calc.y,
							TCP_calc.z);
							usart_send_strg(text7);
														
							//Gelenkwinkel in Pulsweite (*2) umrechnen
							//und im Strut Pul speichern
							set_pulse(&Ang, &Pul, 1, &Led);
							//Erfolgsmeldung ausgeben
							char text8[120];
							sprintf(text8, "Berechnete Pulslängen: Pul_1=%i, Pul_2=%i, Pul_3=%i, Pul_4=%i, Pul_5=%i\n\r",
							Pul.pul_1/2,
							Pul.pul_2/2,
							Pul.pul_3/2,
							Pul.pul_4/2,
							Pul.pul_5/2);
							usart_send_strg(text8);
						}
					}
				}	
						    
				//Puffer für das Einlesen neuer Koordinaten vorbereiten
				usart_strg_compl = 0;
				//Info-LEDs anzeigen
				shiftr_out(shiftr_data(&Led));
				//Eingabeaufforderung für die nächste Eingabe
				usart_send_strg(text1);
			}						
		}	
    }	
}

///////////////////////
// Interrupt Handler //
///////////////////////

//Timer 1 Handler für PWM-Signal
ISR(TIMER1_COMPA_vect)
{
	//Alle Pins an Port B auf HIGH setzen
	//Servos werden synchron angesteuert
	PORTB = 0xFF;		
}

//Timer 0 Handler für die Abtastrate der Joysticks und die Servogeschwindigkeit (Interrupt jede ms)
ISR(TIMER0_COMPA_vect)
{
	Joy_ms_count++;
	Serv_ms_count++;	
}

//USART Handler: USART Rx Complete (0x0024)
ISR(USART_RX_vect)
{
	//Übergabe der x-, y- und z-Koordinaten per Eingabe am seriellen Monitor (x, y, z)
	//Zeichen werden im C-String "usart_rx_buffer" abgelegt
	//wenn alle Zeichen des Strings in den Puffer geladen wurden, 
	//wird die Variable usart_strg_complete auf 1 gesetzt
	uint8_t nextChar;
	//Zeichen für Zeichen einlesen
	nextChar = UDR0;
	if(usart_strg_compl == 0) 
	{
		if(nextChar != '\n' && nextChar != '\r' && usart_strg_count < USART_RX_MAX) 
		{
			usart_rx_buffer[usart_strg_count] = nextChar;
			usart_strg_count++;
		}
		else 
		{
			usart_rx_buffer[usart_strg_count] = '\0';
			usart_strg_count = 0;
			usart_strg_compl = 1;
		}
	}
}


