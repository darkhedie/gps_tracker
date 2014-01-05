/*
 * eeprom.h
 *
 *  Created on: 18.10.2011
 *      Author: darkhedie
 */

#ifndef EEPROM_H_
#define EEPROM_H_


extern unsigned long ulEEPROMAddress;

struct EEPROM_DATA{
	unsigned int  	Datensaetze;
	unsigned long	Last_Address;
	unsigned int	Memory_Type; //Speichergrösse in kbyte
	unsigned char	Reserved;
};


void clear_eeprom(void);
void read_eeprom(void);
void write_buffer_EEPROM(char *Buffer);

#endif /* EEPROM_H_ */
