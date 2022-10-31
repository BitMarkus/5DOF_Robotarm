#include "functions.h"

//Funktionsdefinitionen

//Umrechnung von Rad in Grad
double rad_in_grad(double rad)
{
	return rad * (180.0 / PI);
}
double grad_in_rad(double grad)
{
	return grad * (PI / 180.0);
}
//Arduino map() Funktion für double-Variablen
double mapd(double val, double in_min, double in_max, double out_min, double out_max)
{
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//USART
//USART initialisieren
void usart_init()
{
	//UBRR0:
	//Baudrate von 9600 bps im Register UBRR0H und UBRR0L festlegen (16 bit)
	//Register UBRR0H muss vor dem Register UBRR0L beschrieben werden!
	UBRR0H = (UBRR>>8);
	UBRR0L = (UBRR);
	//UCSR0B: Transmitter (TXEN0)/Receiver (RXEN0) enable
	//RXCIE0: Interrupts aktivieren, wenn empfangenes Byte bereit steht
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	//UCSR0C:
	//USARD Mode Select: UMSEL01 und UMSEL00 = 0 -> Asynchronous Mode
	//Parity Mode: UPM01 und UPM00 = 0 -> Disabled (kein Parity Bit)
	//Stop Bit: USBS0 = 0 -> Ein Stop Bit
	//Character Size: UCSZ02 = 0, UCSZ01 = 1, UCSZ00 = 1 -> 8-bit
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);	
}
//Einzelnes Zeichen über USART empfangen
uint8_t usart_rec_char(void)
{
	//Warten bis das USART data register bereit/leer ist
	while(!(UCSR0A & (1<<UDRE0))) {};
	//Empfangenes Zeichen als 8 bit Char zurückgeben	
	return UDR0;
}
//String über USART empfangen und in Puffer ablegen
void usart_rec_strg(char *Buffer, uint8_t MaxLen)
{
	uint8_t NextChar;
	uint8_t StringLen = 0;
	//Auf nächstes Zeichen warten und empfangen
	NextChar = usart_rec_char();
	//Solange Zeichen in den Puffer schreiben, bis
	//entweder der String aus ist (\n)
	//oder der Puffer voll ist
	while(NextChar != '\n' && NextChar != '\r' && StringLen < MaxLen) 
	{
		*Buffer++ = NextChar;
		StringLen++;
		NextChar = usart_rec_char();
	}
	//'\0' für C-String anhängen
	*Buffer = '\0';
}
//Einzelnes Zeichen über USART senden
void usart_send_char(char ch)
{
	//Warten bis das USART data register bereit/leer ist
	while(!(UCSR0A & (1<<UDRE0))) {}
	//Zeichen senden
	UDR0 = ch;
}
//String über USART an PC senden
void usart_send_strg(char *strg)
{
	while(*strg)
	{
		usart_send_char(*strg);
		strg++;
	}
	
	/*
	uint8_t i = 0;
	while(strg[i] != 0)
	{
		usart_send_char(strg[i]);	
	}
	*/
}
//Parsen des gesendeten Strings vom PC (Delimiter: ,)
//Koordinaten müssen in der Form: x,y,z gesendet werden
//Umwandeln der Substrings in Double-Variablen und  im TCP-Struct speichern
void pars_rx_strg(char *strg, dot *TCP)
{
	//String nach Delimiter auftrennen
	char *substring[3];
	char delimit[] = ",";
	uint8_t i=0;
	substring[i] = strtok(strg, delimit);
	while(substring[i] != NULL)
	{
		i++;
		substring[i] = strtok(NULL, delimit);
	}
	//Substrings in Double umwandeln
	//und x, y und z Werte im TCP-Struct speichern
	//Rückgabe von 0.0, wenn keine Umwandlung möglich ist
	TCP->x = atof(substring[0]);
	TCP->y = atof(substring[1]);
	TCP->z = atof(substring[2]);	
}

//Timer 1 für PWM-Signal
void pwm_init()
{
	//Alle Pins an Port B für die Servos auf Output stellen
	DDRB = 0xFF;
	//Register setzen
	//WGM11, 12 und 13: Mode 14
	//CS11: Prescaler 8 -> 40.000 clks für 20 ms
	//Output Compare A Match Interrupt Enable
	TCCR1A = (1<<WGM11);
	TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11);
	TIMSK1 = (1<<OCIE1A); 
	//ICR1: Wert entspricht 20 ms = eine Periode
	ICR1 = 39999; 	
	//40000 = 20 ms
	//2000 = 1 ms = 1000 µs
	//gewünschte Pulslänge x 2 = Wert in TCNT1
	//Pulslänge von 500 µs = TCNT1 1000	
	//Pulslänge von 1000 µs = TCNT1 2000
	//Pulslänge von 1500 µs (Mittelstellung) = TCNT1 3000	
	//Pulslänge von 2000 µs = TCNT1 4000
	//Pulslänge von 2500 µs = TCNT1 5000
}

//ADC für die Joysticks
void adc_init()
{
	//ADCSRA: ADC Control and Status Register A
	//ADEN=1: ADC aktivieren
	//ADPS0-2: Prescalar (ADPS2=1, ADPS1=1, ADPS0=1 -> 128 Teiler)
	//Benötigte Frequenz: 50-200kHz; 16.000.000/128 = 125.000 (einzig möglicher Teiler, ansonsten > 200kHz)
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	//ADMUX: ADC Multiplexer Selection Register
	//REFS1=0 und REFS0=1: AVcc (with external capacitor at AREF pin)
	ADMUX = (1<<REFS0);
}
//ADC auslesen
//Parameter ist der Kanal, der ausgelesen weden soll (PC0-PC5)
uint16_t adc_read(uint8_t ch)
{
	//7 mögliche Kanäle
	ch &= 0b00000111;
	ADMUX = (ADMUX & 0xF8)|ch;
	//Single convertion starten (1 in ADSC)
	ADCSRA |= (1<<ADSC);
	//Warten, dass die Konvertierung abgeschlossen ist (ADSC wird wieder 0)
	while(ADCSRA & (1<<ADSC));
		
	return (ADC);
}

//Timer 0 für die Abtastfrequenz der Joysticks und die Servogeschwindigkeit
void ms_init()
{
	//Abtastrate soll 1000 Hz btragen (alle 1 ms)
	//CPU Frequenz = 16.000.000 Hz / Teiler 64 = 250.000 [s]
	//1 ms entspricht also 250
	
	//Timer in CTC Mode -> vergleicht mit OCR0A
	//CTC-Mode -> In TCCR0A: WGM00=0, WGM01=1, In TCCR0B: WGM02=0
	TCCR0A = (1<<WGM01);	
	//Teiler: clk/64 -> In TCCR0B: CS02=0, CS01=1, CS00=1
	TCCR0B = (1<<CS00)|(1<<CS01);
	//Vergleichsregister OCR0A auf msTimer = 249 = 1 ms setzen
	OCR0A = 0xF9;
	//Interrupt wenn OCR0A erreicht ist -> In TIMSK0: OCIE0A=1
	TIMSK0 = (1<<OCIE0A);
     
	// initialize counter
	//TCNT0 = 0 ;
}

//Vektorrechnungen
//Betrag eines Vektors berechnen
double vec_betrag(vec *v)
{
	return sqrt((v->x * v->x) + (v->y * v->y) + (v->z * v->z));
}
//Skalarprodukt von zwei Vektoren berechnen
double vec_skalarprod(vec *v1, vec *v2)
{
	return (v1->x * v2->x) + (v1->y * v2->y) + (v1->z * v2->z);;	
}
//Zwei Vektoren subtrahieren -> Vektor
vec vec_subtract(vec *v1, vec *v2)
{
	vec v3 = {v1->x - v2->x, v1->y - v2->y, v1->z - v2->z};
	return v3;
}
//Ortsvektoren zweier Punkte subtrahieren -> Vektor
vec dot_subtract(dot *p1, dot *p2)
{
	vec v = {p1->x - p2->x, p1->y - p2->y, p1->z - p2->z};
	return v;	
}

//Matrizenrechnungen
//4x4 Matrizen multiplizieren
//Übergeben werden Matrix 1 und 2
//Das Ergebnis der Multiplikation wird in Matrix 3 geschrieben
void matr_x_matr(matr m_src1, matr m_src2, matr m_dst)
{
	uint8_t i, j;
    for(i = 0; i < 4; ++i)
    {
	    for(j = 0; j < 4; ++j)
	    {
		    m_dst[i][j] =
		    m_src1[i][0] * m_src2[0][j] +
		    m_src1[i][1] * m_src2[1][j] +
		    m_src1[i][2] * m_src2[2][j] +
		    m_src1[i][3] * m_src2[3][j];
	    }
    }	
}
//4x4 Matrix mit Punkt multiplizieren
//p_src = Punkt für die Multiplikation
//p_dst = Ergebnis der Multiplikation
//Punkte werden der Einfachheit halber nicht als homogene Koordinaten dargestellt (= 4. Variable h)
//h ist in aller Regel = 1, ausser die Matrix ist eine Projektionsmatrix (für perspektivische Darstellung)
//Dann müssen die x,y,z-Koordinaten durch h geteilt werden
void matr_x_dot(matr m, dot *p_src, dot *p_dst)
{
    //COLUMN-MAJOR Variante
    p_dst->x = m[0][0] * p_src->x + m[0][1] * p_src->y + m[0][2] * p_src->z + m[0][3];
    p_dst->y = m[1][0] * p_src->x + m[1][1] * p_src->y + m[1][2] * p_src->z + m[1][3];
    p_dst->z = m[2][0] * p_src->x + m[2][1] * p_src->y + m[2][2] * p_src->z + m[2][3];
}

//Kinematik
//Funktion für die inverse Kinematik
//Übergeben wird der gewünschte TCP
//Die berechneten Gelenkwinkel werden im globalen Struct Ang gespeichert
//Können die Winkel nicht berechnet werden, gibt die Funktion 0 zurück, ansonsten 1
uint8_t inv_kinematic(dot *TCP, ang *Ang)
{
    //x und y des TCP dürfen nicht beide 0 sein
    if(TCP->x == 0 && TCP->y == 0)	
	{
		return 0;
	}
	else
	{
        //////////////////////////
        // Berechnung von Phi_1 //
        //////////////////////////

        //Vektor mit den xy-Koordinaten des TCP erzeugen (vom Ursprung aus)
        vec V_TCPxy = {TCP->x, TCP->y, 0};
        //Betrag des Vektors berechnen (Hypotenuse)
        double L_TCPxy = vec_betrag(&V_TCPxy);
        //Phi1 berechnen (Kosinus)
        Ang->Phi_1 = acos(TCP->x/L_TCPxy);
        //Vorzeichen des Winkels bestimmen
        if(TCP->y < 0)
        {
	        Ang->Phi_1 *= -1.0;
        }		
		
        //////////////////////////
        // Berechnung von Phi_2 //
        //////////////////////////

        //Koordinaten des Punktes G2
        //Unbeweglich, abhängig von L1
        dot P_G2 = {0, 0, L_1};
        //Vektor G2_TCP berechnen
        vec V_G2_TCP = dot_subtract(TCP, &P_G2);
        //Betrag von V_G2_TCP bestimmen
        double L_G2_TCP = vec_betrag(&V_G2_TCP);
        //Wenn L_G2_TCP < L_2+L_3 ist, dann kann das Ziel nicht erreicht werden
        if(L_G2_TCP > (L_2 + L_3))
        {
	        return 0;
        }
        else
        {
			//Vektor parallel zur z-Achse
			vec V_z = {0, 0, -1.0};
			//Hilfswinkel Phi_2a berechnen = Winkel zwischen V_G2_TCP und V_z
			//Immer nur der spitze Winkel wird berechnet
			double Phi_2a = acos(vec_skalarprod(&V_G2_TCP, &V_z)/L_G2_TCP);	
			
            //Hilfswinkel Phi_2b über den Kosinussatz berechnen
            double Phi_2b = acos((L_2*L_2 + L_G2_TCP*L_G2_TCP - L_3*L_3)/(2.0 * L_2 * L_G2_TCP));
            //Phi_21 und Phi_22 berechnen (2 Möglichkeiten für die Gelenkstellung)
            Ang->Phi_21 = Phi_2a + Phi_2b;
            Ang->Phi_22 = Phi_2a - Phi_2b;
            //Phi_2a und Phi_2a auf die Neutralstellung des Roboters beziehen
            Ang->Phi_21 = (Ang->Phi_21 - PI/2.0) * -1.0;
            Ang->Phi_22 = (Ang->Phi_22 - PI/2.0) * -1.0;	
			
            //////////////////////////
            // Berechnung von Phi_3 //
            //////////////////////////

            //Phi_31 berechnen (Kosinussatz)
            Ang->Phi_31 = acos((L_3*L_3 + L_2*L_2 - L_G2_TCP*L_G2_TCP)/(2.0 * L_3 * L_2));
            //Phi_31 und Phi_32 auf die Neutralstellung des Roboters beziehen
            Ang->Phi_31 = PI - Ang->Phi_31;
            Ang->Phi_32 = Ang->Phi_31 * -1.0;	
			
            //Winkel auf unendliche Zahlen oder NAN prüfen
            if(isnan(Ang->Phi_1) || isnan(Ang->Phi_21) || isnan(Ang->Phi_22) || isnan(Ang->Phi_31) ||
               isinf(Ang->Phi_1) || isinf(Ang->Phi_21) || isinf(Ang->Phi_22) || isinf(Ang->Phi_31))
            {
			    return 0;	
			}
		}
	}	
	return 1;
}
//Funktion für die Vorwärtskinematik
//Übergeben wird das Struct Ang mit den Gelenkwinkeln,
//die Transformationsmatrix und der Punkt, der transformiert werden soll
//Ausserdem der Punkt, in dem die Transformation gespeichert wird
//Der letzte Parameter x gibt an, ob Gelenkstellung 1 oder 2 verwendet werden soll
//Kann der Punkt nicht berechnet werden, gibt die Funktion 0 zurück, ansonsten 1
uint8_t fwd_kinematic(ang *Ang, dot *p_src, dot *p_dst, uint8_t x)
{
	//Winkel setzen
	double Phi_1, Phi_2, Phi_3, Phi_4;
	Phi_1 = Ang->Phi_1;
	if(x == 1)
	{
		Phi_2 = Ang->Phi_21;	
		Phi_3 = Ang->Phi_31;			
	}
	else
	{
		Phi_2 = Ang->Phi_22;
		Phi_3 = Ang->Phi_32;		
	}
	Phi_4 = Ang->Phi_4;
	
    //Transformationsmatrizen
	matr M01 = {{cos(Phi_1),    0,      -sin(Phi_1),    0},
				{sin(Phi_1),    0,       cos(Phi_1),    0},
				{0,            -1.0,     0,             L_1},
				{0,             0,       0,             1.0}};
					
	matr M12 = {{cos(Phi_2),    -sin(Phi_2),    0,      L_2*cos(Phi_2)},
				{sin(Phi_2),     cos(Phi_2),    0,      L_2*sin(Phi_2)},
				{0,              0,             1.0,    0},
				{0,              0,             0,      1.0}};

	matr M23 = {{cos(Phi_3+PI/2),    0,      sin(Phi_3+PI/2),      0},
				{sin(Phi_3+PI/2),    0,     -cos(Phi_3+PI/2),      0},
				{0,                             1.0,    0,         0},
				{0,                             0,      0,         1.0}};

	matr M34 = {{cos(Phi_4),    -sin(Phi_4),    0,      0},
				{sin(Phi_4),     cos(Phi_4),    0,      0},
				{0,              0,             1.0,    L_3},
				{0,              0,             0,      1.0}};
					
	//Rotationsmatrizen multiplizieren
	//Überladung von Operatoren in C nicht möglich :-(
	matr M02, M03, M04;
	matr_x_matr(M01, M12, M02);
	matr_x_matr(M02, M23, M03);
	matr_x_matr(M03, M34, M04);
	
	//Gesamtmatrix mit übergebenem Punk multiplizieren
	matr_x_dot(M04, p_src, p_dst);
	
	//Koordinaten des transformierten Punktes auf inf und nan testen
    if(isnan(p_dst->x) || isnan(p_dst->y) || isnan(p_dst->z) ||
	   isinf(p_dst->x) || isinf(p_dst->y) || isinf(p_dst->z))
    {
	    return 0;
    }	
	else
	{
		return 1;	
	}		
}

//Umrechnung der Gelenkwinkel [rad] in Pulslängen [µs*2]
//Übergabe der Servonummer (1-5) und des Winkels [Rad]
uint16_t ang_to_puls(uint8_t s_nr, double ang, led *Led) 
{
	//Geradengleichung mit den Servo-Konstanten:
	//Puls y = ((Puls(0°)-Puls(-90°))/(Pi/2)) * Winkel x + Puls(0°)
    double pulswidth = ((((double)SERVO_const[(s_nr-1)][1] - SERVO_const[(s_nr-1)][2])/(PI/2)) * ang) + SERVO_const[(s_nr-1)][1];
	//Bewegungsfreiheit als Anschlagsbremse einschränken: MIN_MAX_PULSE
	if(pulswidth >= SERVO_const[(s_nr-1)][4])
	{
		pulswidth = (double)SERVO_const[(s_nr-1)][4];
		//Info-LEDs: Gelb
		Led->led_g[(s_nr-1)] = 0;
		Led->led_y[(s_nr-1)] = 1;
		Led->led_r[(s_nr-1)] = 0;
	}
	else if(pulswidth <= SERVO_const[(s_nr-1)][3])
	{
		pulswidth = (double)SERVO_const[(s_nr-1)][3];
		//Info-LEDs: Gelb
		Led->led_g[(s_nr-1)] = 0;
		Led->led_y[(s_nr-1)] = 1;
		Led->led_r[(s_nr-1)] = 0;
	}	
	else
	{
		//Info-LEDs: Grün
		Led->led_g[(s_nr-1)] = 1;
		Led->led_y[(s_nr-1)] = 0;
		Led->led_r[(s_nr-1)] = 0;	
	}	
	//Errechnete Pulsweiten in µs müssen für den Eintrag in TCNT1 mit 2 multipliziert werden
	pulswidth *= 2.0;
	//Pulsweite runden (Umwandlung in int)
	return (uint16_t)(pulswidth + 0.5);
}

//Pulslängen in Struct Pul speichern
//Der letzte Parameter x gibt an, ob Gelenkstellung 1 oder 2 verwendet werden soll
void set_pulse(ang *Ang, pul *Pul, uint8_t x, led *Led)
{
	Pul->pul_1 = ang_to_puls(1, Ang->Phi_1, Led);
	if(x == 1)
	{
		Pul->pul_2 = ang_to_puls(2, Ang->Phi_21, Led);
		Pul->pul_3 = ang_to_puls(3, Ang->Phi_31, Led);
	}
	else
	{
		Pul->pul_2 = ang_to_puls(2, Ang->Phi_22, Led);
		Pul->pul_3 = ang_to_puls(3, Ang->Phi_32, Led);
	}	
	Pul->pul_4 = ang_to_puls(4, Ang->Phi_4, Led);
	Pul->pul_5 = ang_to_puls(5, Ang->Phi_5, Led);
}

//Wandelt die Joystickeingaben in eine Pulsweite um
//j_val = analoger Wert der Joystickachse (1-1024)
//s_nr = Servonummer (1-5)
//s_puls_alt = aktueller Servopuls (µs*2)
//s_dir = gewünschte Servorichtung (1,0)
//led *Led = Referenz zum Struct für die Info-LEDs
uint16_t joy_to_pulse(uint16_t j_val, uint8_t s_nr, uint16_t s_puls_alt, uint8_t s_dir, led *Led)
{
	//Rückgabewert
	uint16_t s_puls_neu;
	//Die Pulsänderung über die Joystickeingabe berechnen
	uint16_t dpulse;
	if(j_val > (JOY_mid + JOY_dz))
	{
		//Quadratische Funktion, da eine lineare Funktion zu schnell zu einer hohen Servogeschwindigkeit führt
		//Die höchste Pulsänderung sollte bei ca 40 liegen
		dpulse = pow(((j_val - JOY_mid) / JOY_div), 2);
		//Die Pulsänderung zur aktuellen Pulsweite addieren bzw subtrahieren (Servorichtung)	
		if(s_dir == 1)
		{s_puls_neu = s_puls_alt + dpulse;}
		else
		{s_puls_neu = s_puls_alt - dpulse;}
	}
	else if(j_val < (JOY_mid - JOY_dz))
	{
		dpulse = pow(((JOY_mid - j_val) / JOY_div), 2);
		//Die Pulsänderung zur aktuellen Pulsweite addieren bzw subtrahieren (Servorichtung)
		if(s_dir == 1)
		{s_puls_neu = s_puls_alt - dpulse;}
		else
		{s_puls_neu = s_puls_alt + dpulse;}
	}
	else
	{
		s_puls_neu = s_puls_alt;	
	}	
	//Die Bewegungsfreiheit der Servos einschränken
	//und Info-LEDs setzen (Servoanschlag)
	if(s_puls_neu >= (SERVO_const[(s_nr-1)][4]*2))
	{
		s_puls_neu = SERVO_const[(s_nr-1)][4]*2;
		//Info-LEDs: Gelb
		Led->led_g[(s_nr-1)] = 0;
		Led->led_y[(s_nr-1)] = 1;
		Led->led_r[(s_nr-1)] = 0;
	}
	else if(s_puls_neu <= (SERVO_const[(s_nr-1)][3]*2))
	{
		s_puls_neu = SERVO_const[(s_nr-1)][3]*2;
		//Info-LEDs: Gelb
		Led->led_g[(s_nr-1)] = 0;
		Led->led_y[(s_nr-1)] = 1;
		Led->led_r[(s_nr-1)] = 0;
	}
	else
	{
		//Info-LEDs: Grün
		Led->led_g[(s_nr-1)] = 1;
		Led->led_y[(s_nr-1)] = 0;
		Led->led_r[(s_nr-1)] = 0;		
	}
	return s_puls_neu;
}

//Gelenkwinkel auf die Neutralstellung setzen
void reset_ang(ang *Ang)
{
	Ang->Phi_1 = 0;	
	Ang->Phi_21 = 0;
	Ang->Phi_22 = 0;
	Ang->Phi_31 = 0;
	Ang->Phi_32 = 0;
	Ang->Phi_4 = 0;
	Ang->Phi_5 = 0;
}

//TCP auf die Neutralstellung setzen
void reset_tcp(dot *TCP)
{
	TCP->x = L_1+L_2;
	TCP->y = 0;	
	TCP->z = L_3;	
}

//Funktionen für die Schieberegister (2xHC595)
//HC595 initialisieren
void shiftr_init()
{
	//DS, SH_CP und ST_CP Pins auf Output stellen
	HC595_DDR |= (1<<HC595_SH_CP_POS)|(1<<HC595_ST_CP_POS)|(1<<HC595_DS_POS);
}

//Puls auf Shift Clock Pin (SH_CP oder SRCLK)
void shiftr_shift()
{
	HC595_PORT |= (1<<HC595_SH_CP_POS);		//HIGH
	HC595_PORT &= (~(1<<HC595_SH_CP_POS));	//LOW
}

//Puls auf Store Clock Pin (ST_CP oder RCLK)
void shiftr_latch()
{
	HC595_PORT |= (1<<HC595_ST_CP_POS);		//HIGH
	//_delay_loop_1(1);
	HC595_PORT &= (~(1<<HC595_ST_CP_POS));	//LOW
	//_delay_loop_1(1);
}

//Zustände der Info-LEDs aus dem Struct "Led" Bit für Bit in uint16_t Variable schreiben
//Auffüllen von Rechts
//Bit 16 (links): ungenutzt, immer 0
uint16_t shiftr_data(led *Led)
{
	uint16_t data = 0b0000000000000000;

	//Servo 1: grün
	if(Led->led_g[0] == 1) {data |= (1<<0);}
	//Servo 1: orange
	if(Led->led_y[0] == 1) {data |= (1<<1);}
	//Servo 1: rot
	if(Led->led_r[0] == 1) {data |= (1<<2);}
	//Servo 2: grün
	if(Led->led_g[1] == 1) {data |= (1<<3);}
	//Servo 2: orange
	if(Led->led_y[1] == 1) {data |= (1<<4);}
	//Servo 2: rot
	if(Led->led_r[1] == 1) {data |= (1<<5);}
	//Servo 3: grün
	if(Led->led_g[2] == 1) {data |= (1<<6);}
	//Servo 3: orange
	if(Led->led_y[2] == 1) {data |= (1<<7);}
	//Servo 3: rot
	if(Led->led_r[2] == 1) {data |= (1<<8);}
	//Servo 4: grün
	if(Led->led_g[3] == 1) {data |= (1<<9);}
	//Servo 4: orange
	if(Led->led_y[3] == 1) {data |= (1<<10);}
	//Servo 4: rot
	if(Led->led_r[3] == 1) {data |= (1<<11);}
	//Servo 5: grün
	if(Led->led_g[4] == 1) {data |= (1<<12);}
	//Servo 5: orange
	if(Led->led_y[4] == 1) {data |= (1<<13);}
	//Servo 5: rot
	if(Led->led_r[4] == 1) {data |= (1<<14);}

	return data;
}

//Funktion gibt den Zustand der Info-LEDs über zwei Schieberegister (= 16 Byte) aus
//Da nur 15 LEDs vorhanden sind, ist das letzte Byte immer 0
void shiftr_out(uint16_t data)
{
	//Low level macros um Data Pin (DS) auf HIGH oder LOW zu setzen
	//#define HC595DataHigh() (HC595_PORT |= (1<<HC595_DS_POS))
	//#define HC595DataLow() (HC595_PORT &= (~(1<<HC595_DS_POS)))

	//Jeden der 15 Bits seriell senden (MSB zuerst)
	for(uint16_t i=0; i<SHIFTR_BITS; i++)
	{
		//Output über DS je nach Wert des MSB (HIGH oder LOW)
		if(data & 0b1000000000000000)
		{
			//MSB is1 1: output HIGH
			HC595_PORT |= (1<<HC595_DS_POS);		//HC595DataHigh();
		}
		else
		{
			//MSB ist 0: output LOW
			HC595_PORT &= (~(1<<HC595_DS_POS));		//HC595DataLow();
		}
		//Puls auf Shift-Clock (SH_CP)
		shiftr_shift();
		//Nächstes Bit auf die MSB Position bringen
		data = (data<<1);
	}
	//Alle 15 Bits ins Schieberegister geladen
	//Puls auf Store Clock (ST_CP) -> Output
	shiftr_latch();
}
