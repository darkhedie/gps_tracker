/*
 * main.c
 *
 *  Created on: 27.09.2011
 *      Author: darkhedie
 */

//#define	 _AVR_ATmega324P_
#include <avr/io.h>
//#include <avr/iom324.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include "SIRF.h"
#include "eeprom.h"
#include "I2C.h"
#include <avr/interrupt.h>
#define F_CPU 10000000UL
#define BAUD 9600
#include <util/setbaud.h>

volatile unsigned char ucMode = 0;	//Bestimmt ob im UART RX Interrupt die Daten weitergeleitet (1) oder verarbeitet (2) werden sollen. Alles andere ist IDLE und die daten des UART RX werden verworfen
volatile unsigned char ucValue = 0; //Daten für die Interrupt routine
volatile unsigned char ucFirst = 0;	//Erster start?
volatile unsigned char ucMode_2 = 3; // Modus des Programmes. 1 leitet daten Weiter, 2 Verarbeitet die Daten, 3 liest das EEPROM aus, 4 Factory reset
volatile unsigned char ucFlag = 0;	// Flag der Interrupt routine. Damit wird bestimmt ob nach einem Befehl noch Daten folgen und wenn ja, wie viele (byte)
volatile unsigned char ucBlinken = 0;
volatile unsigned char ucBlinkspeed = 0;
volatile unsigned char ucBlinkCounter = 0;
unsigned char 			GPS_Ready = 0;
unsigned char			ucFirstDataLog = 1;
unsigned char			ucSaveDelay = 5;	//Nach wie vielen Durchgängen soll ein Datensatz gespeichert werden

volatile char RX_Buffer[150] = {};
volatile unsigned char Buffer_Counter = 0;
volatile unsigned char ucSave = 0;
volatile unsigned char ucLogMSG = 41;
unsigned long ulEEPROMAddress = 0;
unsigned char ucEEPROM_Buffer[10] = {0};

struct EEPROM_DATA *EEPROM_Data;

#define	MAX_Capacity 127999 		//obergrenze der kapazität bei 1mbit entspricht 128kbyte

#define LED0_1		PORTA |= (1<<PA4)
#define LED0_0		PORTA &= ~(1<<PA4)

#define LED1_1		PORTA |= (1<<PA5)
#define LED1_0		PORTA &= ~(1<<PA5)

#define PowerEN_1		PORTC |= (1<<PC2)
#define PowerEN_0		PORTC &= ~(1<<PC2)

#define GPS_RST_1	PORTD |= (1<<PD5)
#define GPS_RST_0	PORTD &= ~(1<<PD5)

#define GPS_BOOT_1	PORTD |= (1<<PD4)
#define GPS_BOOT_0	PORTD &= ~(1<<PD4)

#define	Taster	( PINC & (1<<PINC7) )

void read_eeprom(void);
void Load_EEPROM_Data(void);
void clear_eeprom(void);
void write_buffer_EEPROM(char *Buffer);
void Write_EEPROM_Data(void);



int uart0_putc(unsigned char c)
{
    while (!(UCSR0A & (1<<UDRE0)))  /* warten bis Senden moeglich */
    {
    }

    UDR0 = c;                      /* sende Zeichen */
    return 0;
}



/* puts ist unabhaengig vom Controllertyp */
void uart0_puts (char *s)
{
    while (*s)
    {   /* so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" */
        uart0_putc(*s);
        s++;
    }
}


int uart1_putc(unsigned char c)
{
    while (!(UCSR1A & (1<<UDRE1)))  /* warten bis Senden moeglich */
    {
    }

    UDR1 = c;                      /* sende Zeichen */
    return 0;
}


/* puts ist unabhaengig vom Controllertyp */
void uart1_puts (char *s)
{
    while (*s)
    {   /* so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" */
        uart1_putc(*s);
        s++;
    }
}



void uart_init(void)
{

  // UBRR1H = UBRRH_VALUE;
  // UBRR1L = UBRRL_VALUE;


   UBRR1 = 64; //Für eine Baudrate von 57600  -> Geht nur wenn U2X nicht aktiv ist!
   UCSR1A |= (1 << U2X1); //U2X aktivieren damit 115200 Kbit möglich ist!


   UBRR0H = UBRRH_VALUE;
   UBRR0L = UBRRL_VALUE;
   /* evtl. verkuerzt falls Register aufeinanderfolgen (vgl. Datenblatt)
      UBRR = UBRR_VALUE;
   */
#if USE_2X
   /* U2X-Modus erforderlich */
  // UCSR1A |= (1 << U2X1);
   UCSR0A |= (1 << U2X0);
#else
   /* U2X-Modus nicht erforderlich */
  // UCSR1A &= ~(1 << U2X1);
   UCSR0A &= ~(1 << U2X0);
#endif

   // hier weitere Initialisierungen (TX und/oder RX aktivieren, Modus setzen
   UCSR0B |= (1<<RXEN0) |(1<<TXEN0) |(1<<RXCIE0);  // UART TX einschalten
   UCSR0C =  (1<<UCSZ01)|(1<<UCSZ00);  // Asynchron 8N1

   UCSR1B |= (1<<RXEN1) | (1<<TXEN1) |(1<<RXCIE1);  // UART RX einschalten TX erst einschalten wenn TUSB3410 bereit ist also nach empfangen eines zeichens
   UCSR1C =  (1<<UCSZ11)|(1<<UCSZ10);  // Asynchron 8N1
}


unsigned int intswap(unsigned int uiSwap)
{
	return ((uiSwap << 8) & 0xFF00) | ((uiSwap & 0xFF00) >> 8);
}

unsigned long longswap(unsigned long uiSwap)
{
	unsigned long ucTemp = 0;
	ucTemp = uiSwap;
	uiSwap = uiSwap & 0x00FFFF00;
	uiSwap = uiSwap | ((ucTemp << 24) & 0xFF000000) | ((ucTemp & 0xFF000000) >> 24);

	ucTemp = uiSwap;
	uiSwap = uiSwap & 0xFF0000FF;
	uiSwap = uiSwap | ((ucTemp << 8) & 0x00FF0000) | ((ucTemp & 0x00FF0000) >> 8);

	return uiSwap;

}

ISR(USART0_RX_vect)
{
	unsigned char nextChar;
	nextChar = UDR0;
	//LED1_1;
	//LED1_0;

	switch (ucMode_2) {
		case 0x01:
			uart1_putc(nextChar); //Daten weiterleiten
			break;
		case 0x02:
			  // Daten aus dem Puffer lesen
			  if((ucSave == 5) && (nextChar == ucLogMSG)) ucSave = 1; //Start erkannt und bestimmte MSG welche mit ucLogMSG bestimmt wird erkann. nun darf gespeichert werden
			  if((ucSave == 5) && !(nextChar == ucLogMSG))
			  {
				  ucSave = 0;
				  Buffer_Counter = 0;
			  }
			  if(ucSave == 4)
			  {
			  	  ucSave = 5;
			  	  RX_Buffer[Buffer_Counter] = nextChar;
			  	  Buffer_Counter++;
			    }
			  if(ucSave == 3)
			  {
				  ucSave = 4;
				  RX_Buffer[Buffer_Counter] = nextChar;
				  Buffer_Counter++;
			  }
			  if((ucSave == 0) && (nextChar == 0xA0)) ucSave = 2; //Möglicherweise Start erkannt... Wenn nächstes Zeichen A2 ist dann beginne mit speichern(save = 1)
			  if((ucSave == 2) && (nextChar == 0xA2)) ucSave = 3; //Start erkannt... Wenn ID 02 dann Speichern

			  if((ucSave == 10) && (nextChar == 0xB3)) ucSave = 11; //Ende Erkannt
			  if((ucSave == 10) && !(nextChar == 0xB3)) ucSave = 1; //War kein ende... Normal weiter
			  if((ucSave == 1) && (nextChar == 0xB0)) ucSave = 10; //Eventuell Ende erkannt

			  if((ucSave == 1) || (ucSave == 10))
			  {
				  RX_Buffer[Buffer_Counter] = nextChar;
				  Buffer_Counter++;
			  }
			break;

		default:
			break;
	}
}

ISR(TIMER1_COMPA_vect)
{
	ucBlinkCounter++;
	if(ucBlinken > 0)
	{
		if((ucBlinken == 1) && (ucBlinkCounter > ucBlinkspeed))
		{
			LED0_1;
			ucBlinken = 2;
			ucBlinkCounter = 0;
		}
		else
		{
			if((ucBlinkCounter > ucBlinkspeed))
			{
			LED0_0;
			ucBlinken = 1;
			ucBlinkCounter = 0;
			}
		}
	}
	else
	LED0_0;
}

/// Empfangsinterrupt auf PC Seite
ISR(USART1_RX_vect)
{
	char Buffer_LOCAL[10]={};
	unsigned char ucTemp = 0;
	unsigned int  uiTemp = 0;
	ucTemp = UDR1;

	if(ucTemp == '1')
	{
		ucMode_2 = 0x01;		//Daten weiterleiten -> GPS Binary -> PC
		ucBlinkspeed = 20;
	}
	if(ucTemp == '2')
	{
		ucMode_2 = 0x02; //Daten verarbeiten
		ucBlinken = 1; //Blinken eingeschaltet
	}
	if(ucTemp == '3')
	{
		ucMode_2 = 0x00; //IDLE Mode, keine UART Kommunikation von GPS -> PC
		ucBlinkspeed = 20;
		read_eeprom();
	}
	if(ucTemp == '4')
	{
		ucMode_2 = 0x00; //IDLE Mode, keine UART Kommunikation von GPS -> PC
		ucBlinkspeed = 20;
		EEPROM_Data->Datensaetze = 0;
		EEPROM_Data->Last_Address = 10;
		EEPROM_Data->Memory_Type = 128;	//128kByte
		EEPROM_Data->Reserved = 88;		//88 ist eine schnapszahl
		ulEEPROMAddress = 10;
		Write_EEPROM_Data();
	}
	if(ucTemp == '5')
	{
		ucMode_2 = 0x00; //IDLE Mode, keine UART Kommunikation von GPS -> PC
		ucBlinkspeed = 20;
		ltoa(EEPROM_Data->Last_Address, Buffer_LOCAL, 10);
		uart1_puts(" Adresse im EEPROM: ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts(" Adresse aktuell: ");
		ltoa(ulEEPROMAddress, Buffer_LOCAL, 10);
		uart1_puts(Buffer_LOCAL);
		uart1_puts(" Datensaetze: ");
		ltoa(EEPROM_Data->Datensaetze, Buffer_LOCAL, 10);
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");
	}
	if(ucTemp == '6')
	{
		ucMode_2 = 0x00; //IDLE Mode, keine UART Kommunikation von GPS -> PC
		ucBlinkspeed = 20;

		ltoa(I2C_Read_8bit(0x01), Buffer_LOCAL, 10);
		uart1_puts(" Status Register (0x01): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x0A), Buffer_LOCAL, 10);
		uart1_puts(" Temperatur MSB (0x0A): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x0B), Buffer_LOCAL, 10);
		uart1_puts(" Temperatur LSB (0x0B): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x0C), Buffer_LOCAL, 10);
		uart1_puts(" Spannung MSB (0x0C): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x0D), Buffer_LOCAL, 10);
		uart1_puts(" Spannung LSB (0x0D): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x0E), Buffer_LOCAL, 10);
		uart1_puts(" Strom MSB (0x0E): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x0F), Buffer_LOCAL, 10);
		uart1_puts(" Strom LSB (0x0F): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x10), Buffer_LOCAL, 10);
		uart1_puts(" Accumulated  Strom MSB (0x10): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x11), Buffer_LOCAL, 10);
		uart1_puts(" Accumulated  Strom LSB (0x11): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x61), Buffer_LOCAL, 10);
		uart1_puts(" Offset Bias Register (0x61): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

		ltoa(I2C_Read_8bit(0x62), Buffer_LOCAL, 10);
		uart1_puts(" Accumulation Bias Register (0x62): ");
		uart1_puts(Buffer_LOCAL);
		uart1_puts("\r\n");

	}


	if(ucTemp == 'A')	//Datensätze senden
	{
		ucMode_2 = 0x00; //IDLE Mode, keine UART Kommunikation von GPS -> PC
		ucBlinkspeed = 20;
		ltoa(EEPROM_Data->Datensaetze, Buffer_LOCAL, 10);
		uart1_puts(Buffer_LOCAL);
	}

	if(ucTemp == 'B')	//Höchste und letzte bekannte adresse
	{
		ucMode_2 = 0x00; //IDLE Mode, keine UART Kommunikation von GPS -> PC
		ucBlinkspeed = 20;
		ltoa(EEPROM_Data->Last_Address, Buffer_LOCAL, 10);
		uart1_puts(Buffer_LOCAL);
	}

	if(ucTemp == 'Z')	//Datenstring senden mit komma als Trennung
	{
		ucMode_2 = 0x00; //IDLE Mode, keine UART Kommunikation von GPS -> PC
		ucBlinkspeed = 20;

		uart1_puts("6,"); //Anzahl Einträge senden
		ltoa(EEPROM_Data->Datensaetze, Buffer_LOCAL, 10);
		uart1_puts(Buffer_LOCAL);
		uart1_putc(',');
		ltoa(EEPROM_Data->Last_Address, Buffer_LOCAL, 10);
		uart1_puts(Buffer_LOCAL);
		uart1_putc(',');
		ltoa(I2C_Read_8bit(0x01), Buffer_LOCAL, 10);
		uart1_puts(Buffer_LOCAL);
		uart1_putc(',');

		ltoa( (((I2C_Read_8bit(0x0C) << 8 ) + I2C_Read_8bit(0x0D)) >> 5) , Buffer_LOCAL, 10); //Voltage
		uart1_puts(Buffer_LOCAL);
		uart1_putc(',');
		ltoa((((I2C_Read_8bit(0x0A) << 8) + I2C_Read_8bit(0x0B)) >> 5), Buffer_LOCAL, 10); //Temparature
		uart1_puts(Buffer_LOCAL);
		uart1_putc(',');
		ltoa(((I2C_Read_8bit(0x0E) << 8) + I2C_Read_8bit(0x0F)), Buffer_LOCAL, 10); //Current
		uart1_puts(Buffer_LOCAL);
		uart1_putc(';');
	}

}

void clear_eeprom(void)
{
	ulEEPROMAddress = 0;
	while(ulEEPROMAddress != 200)
	{
		I2C_Write_17bit(0xA0,ulEEPROMAddress,0x00);
		ulEEPROMAddress++;
	}
}


void read_eeprom(void)
{
	unsigned long ulTempAddr = 0;
	ulTempAddr = 10; //Von beginn an
	while(ulTempAddr != EEPROM_Data->Last_Address)
	{
		uart1_putc((I2C_Read_17bit(0xA0,ulTempAddr)));
		ulTempAddr++;
	}
}

//////////////////////////////////////////////////////////////
// 		Schreibt den Buffer Inhalt ins EEPROM ab Adresse	//
//		welche zuvor mit ulEEPROMAddress definiert wurde	//
//////////////////////////////////////////////////////////////
void write_buffer_EEPROM(char *Buffer)
{
	unsigned char ucTemp = 0;
	while(ucTemp < strlen(Buffer))
	{
		I2C_Write_17bit(0xA0,ulEEPROMAddress, Buffer[ucTemp]);
		ulEEPROMAddress++;
		ucTemp++;
	}
}


//////////////////////////////////////////////////////////////
// 		Schreibt den Buffer Inhalt ins EEPROM ab Adresse	//
//		welche zuvor mit ulEEPROMAddress definiert wurde	//
//////////////////////////////////////////////////////////////
void Load_EEPROM_Data(void)
{
	unsigned char ucCounter = 0;
	while(ucCounter != 10)
	{
		ucEEPROM_Buffer[ucCounter] = I2C_Read_17bit(0xA0,ucCounter);
		ucCounter++;
	}
}

//////////////////////////////////////////////////////////////
// 		Schreibt den Buffer Inhalt ins EEPROM ab Adresse	//
//		welche zuvor mit ulEEPROMAddress definiert wurde	//
//////////////////////////////////////////////////////////////
void Write_EEPROM_Data(void)
{
	unsigned char ucCounter = 0;
	while(ucCounter != 10)
	{
		I2C_Write_17bit(0xA0,ucCounter,ucEEPROM_Buffer[ucCounter]);
		ucCounter++;
	}
}



int main(void)
{

	unsigned char ucCounter = 0;	//Zähler wie oft ein gültiger datensatz vorhanden war.

	DDRC &= ~(1 << DDC7);	//Taster Eingang
	DDRB &= ~(1 << DDB2);	//1PPS
	DDRD &= ~(1 << DDD0);	//RX1
	DDRD &= ~(1 << DDD2);	//RX2
	DDRA &= ~(1 << DDA6);	//SDA
	DDRD &= ~(1 << DDD4);	//RX2


	DDRA |= (1 << DDA7);	//SCL
	DDRD |= (1 << DDD1);	//TX1
	DDRD |= (1 << DDD3);	//TX2
	//DDRD |= (1 << DDD4);	//GPS Boot
	DDRD |= (1 << DDD5);	//GPS RST

	DDRC |= (1 << DDC2);	//PowerEN Ausgang
	DDRA |= (1 << DDA4) | (1 << DDA5);	//LED1 & 2

	PowerEN_1;

	LED0_1;

	//while(1);


	TCCR1A = (1<<WGM01); // CTC Modus
	TCCR1B |= (1<<CS02) | (1<<CS00); // Prescaler 1024
	OCR1A  = 65535;
	TIMSK1 |= (1<<OCIE0A);

	ucBlinken = 1;		//Standard
	ucBlinkspeed = 10;	//Langsames Blinken mit etwa 2 Sekunde -> Wartet auf befehl -> ist IDLE

	//GPS_BOOT_0;
	//GPS_RST_0;
	LED0_1;

	//_delay_ms(200);
	GPS_RST_1;




	while(Taster);
	_delay_ms(400);


	uart_init();
	sei();
/*
	I2C_Write_17bit(0xA0,60000,0xFA);
	_delay_ms(5);
	if(I2C_Read_17bit(0xA0,60000) == 0xFA)	LED0_0;
*/
	LED0_0;





	Load_EEPROM_Data(); //EEPROM Daten laden
	EEPROM_Data = (struct EEPROM_DATA *)&ucEEPROM_Buffer[0]; //Daten mit Struct verknüpfen
	ulEEPROMAddress = EEPROM_Data->Last_Address;	//Ende des EEPROMs als Start verwenden

	LED1_1;

	_delay_ms(1000);
	uart0_puts("$PSRF100,0,9600,8,1,0*0C\r\n"); //Auf Sirf binary wechseln

	struct ID_41 *GPS_Pos;
	char Buffer[20]={};

	LED1_0;

	ucMode_2 = 2; //Modus verarbeitung

	while(1)
	{

		if((ucSave == 11) && (ucCounter >= ucSaveDelay)) //Prüfen, ob GPS Daten verarbeitet sind und ob das Delay erreicht wurde.
		{
			ucCounter = 0;	//Delay Counter zurücksetzen
			GPS_Pos = (struct ID_41 *)&RX_Buffer[0];
			ucLogMSG = 41;
			ucSave = 12; //IDLE... nichts tun

			if(GPS_Pos->Latitude > 0)	//Gültige Daten vorhanden
			{
				ucBlinken = 0;
				LED0_1;			//Kurzes aufblitzen wenn Datensatz gespeichert wird
				_delay_ms(70);
				LED0_0;
				_delay_ms(70);
				LED0_1;
				GPS_Ready = 1;
			}
			else	//Keine Gültigen Daten
			{
				ucBlinken = 1;
				ucBlinkspeed = 5;
				GPS_Ready = 0;
			}

			if(GPS_Ready == 1)
			{
				if(ucFirstDataLog == 1)	//Prüfen ob beginn einer neuen Datalog sequenz. Wenn ja, Anfangszeichen speichern (<)
				{
					I2C_Write_17bit(0xA0,ulEEPROMAddress,'<');
					//Uhrzeit datum schreiben
					LED1_1;
					ulEEPROMAddress++;
					ucFirstDataLog = 0;
				}
				/// ---> prüfen ob kapazität erreicht ist :)
				I2C_Write_17bit(0xA0,ulEEPROMAddress,'G');
				ulEEPROMAddress++;
			}

			uart1_puts("Latitude:   ");
			ltoa((longswap(GPS_Pos->Latitude) / 10000000 ), Buffer, 10 );
			uart1_puts(Buffer);
			if(GPS_Ready == 1)
			{
				write_buffer_EEPROM(Buffer);	//Schreibe Buffer Inhalt in EEPROM
				I2C_Write_17bit(0xA0,ulEEPROMAddress,'.');
				ulEEPROMAddress++;
			}

			uart1_putc('.');
			ltoa(((longswap(GPS_Pos->Latitude) % 10000000 ) / 100 ), Buffer, 10 );
			uart1_puts(Buffer);
			if(GPS_Ready == 1)
			{
				write_buffer_EEPROM(Buffer);
				I2C_Write_17bit(0xA0,ulEEPROMAddress,',');
				ulEEPROMAddress++;
			}
			uart1_puts("\r\n");


			uart1_puts("Longtitude: ");
			ltoa((longswap(GPS_Pos->Longtitude) / 10000000 ), Buffer, 10 );
			uart1_puts(Buffer);
			if(GPS_Ready == 1)
			{
				write_buffer_EEPROM(Buffer);
				I2C_Write_17bit(0xA0,ulEEPROMAddress,'.');
				ulEEPROMAddress++;
			}

			uart1_putc('.');
			ltoa(((longswap(GPS_Pos->Longtitude) % 10000000 ) / 100 ), Buffer, 10 );
			uart1_puts(Buffer);
			if(GPS_Ready == 1)
			{
				write_buffer_EEPROM(Buffer);
				I2C_Write_17bit(0xA0,ulEEPROMAddress,';');
				ulEEPROMAddress++;
			}
			uart1_puts("\r\n");



			Buffer_Counter = 0; //Reset
			ucSave = 0;			//Reset


		}
		else
		{
			if(ucSave == 11)	//Nur Counter erhöhen wenn auch Daten verarbeitet wurden. Ansonsten würde hier immer hochgezählt werden.
			{					//wenn z.b. kein ucSave 11 vorhanden ist, da es eine && verknüpfung ist
				ucCounter++; //Delay Counter erhöhen
				Buffer_Counter = 0; //Reset
				ucSave = 0;			//Reset
			}
		}




		if(Taster)
		{
			while(Taster);
			if(EEPROM_Data->Last_Address < ulEEPROMAddress)	//Prüfen ob eine Speicherveränderung stattgefunden hat.
			{
				I2C_Write_17bit(0xA0,ulEEPROMAddress,'>');	//Abschluss setzen
				ulEEPROMAddress++;
				EEPROM_Data->Datensaetze = EEPROM_Data->Datensaetze + 1;
				EEPROM_Data->Last_Address = ulEEPROMAddress;
				Write_EEPROM_Data(); //EEPROM Daten speichern
			}
			_delay_ms(500);
			PowerEN_0;

		}

	}

}
