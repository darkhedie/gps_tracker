/*
 * i2c.h
 *
 *  Created on: 18.11.2010
 *      Author: darkhedie
 */

#ifndef I2C_H_
#define I2C_H_


// Pin Configurations


#define SCL_1  PORTA |= (1<<PA7)
#define SCL_0  PORTA &= ~(1<<PA7)
#define SDA_1  PORTA |= (1<<PA6)
#define SDA_0  PORTA &= ~(1<<PA6)
#define SDA	   ( PINA & (1<<PINA6))

#define set_output 	DDRA |= ( 1 << DDA6);
#define	set_input 	DDRA &= ~( 1 << DDA6 );


#define I2C_Delay 		25 //Delay in ms

#define I2C_Addr		0x90 //Adresse des DS2745

void I2C_Init(void);
unsigned char I2C_Read_17bit(unsigned char ucI2CAddr, unsigned long uiAddr);
void I2C_Write_17bit(unsigned char ucI2CAddr, unsigned long uiAddr, unsigned char ucData);
unsigned char I2C_Read_16bit(unsigned int uiAddr);
void I2C_Write_16bit(unsigned int uiAddr, unsigned char ucData);
unsigned char I2C_Read_8bit(unsigned char ucAddr);
void I2C_Write_8bit(unsigned char ucAddr, unsigned char ucData);

#endif /* I2C_H_ */
