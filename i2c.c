/*
 * i2c.c
 *
 *  Created on: 18.11.2010
 *      Author: darkhedie
 */

#include <avr/io.h>
#include <util/delay.h>
#include "i2c.h"


 //DDRB &= ~( 1 << DDB0 );


 //PORTB |= (1<<PB4)	//Pin auf High
//		PORTB &= ~(1<<PB2) // Pin auf low

void I2C_Init(void)
{
	//DDRA |= ( 1 << PA0); //AVR
}

//SCL = 1  SDA 1 -> 0
void I2C_Start(void)
{
	set_output;
	SDA_1;
	SCL_1;
	_delay_us(I2C_Delay);
	SDA_0;
	_delay_us(I2C_Delay);
	SCL_0;
}

void I2C_Clock(void)
{
	SCL_0;
	_delay_us(I2C_Delay);
	SCL_1;
	_delay_us(I2C_Delay);
	SCL_0;
}

//SCL = 1  SDA 0 -> 1
void I2C_Stop(void)
{
	set_output;
	SDA_0;
	SCL_1;
	_delay_us(I2C_Delay);
	SDA_1;
	_delay_us(I2C_Delay);
	SCL_0;
}


unsigned char i2c_rx(char ack)
{
char x, d=0;
  set_input;
  for(x=0; x<8; x++) {
    d <<= 1;
      SCL_1;
      _delay_us(I2C_Delay);
    if(SDA) d |= 1;
    SCL_0;
    _delay_us(I2C_Delay);
  }
  set_output;
  if(ack) SDA_0;
  else SDA_1;
  SCL_1;
  _delay_us(I2C_Delay);             // send (N)ACK bit
  SCL_0;
  return d;
}


void I2C_Send_8(unsigned char ucData)
{
	unsigned char ucCounter = 0;
	unsigned char ucTemp = 0;
	set_output;
	while(ucCounter != 8)
	{
		ucTemp = ucData & 0x80;
		if(ucTemp > 0) SDA_1;
		if(ucTemp == 0) SDA_0;
		ucData = ucData << 1;
		I2C_Clock();
		ucCounter++;
	}
	set_input;
	_delay_us(I2C_Delay);
	SCL_1;
	while(SDA);
	_delay_us(I2C_Delay);
	SCL_0;
}

void I2C_Send_8_nack(unsigned char ucData)
{
	unsigned char ucCounter = 0;
	unsigned char ucTemp = 0;
	set_output;
	while(ucCounter != 8)
	{
		ucTemp = ucData & 0x80;
		if(ucTemp > 0) SDA_1;
		if(ucTemp == 0) SDA_0;
		ucData = ucData << 1;
		I2C_Clock();
		ucCounter++;
	}
	set_input;
	_delay_us(I2C_Delay);
	SCL_1;
	while(!SDA);	//auf NACK warten
	_delay_us(I2C_Delay);
	SCL_0;
}



void I2C_Write_16bit(unsigned int uiAddr, unsigned char ucData)
{
	I2C_Start();
	I2C_Send_8(I2C_Addr & 0xFE);	//Adresse ausgeben
	I2C_Send_8((uiAddr & 0xFF00) >> 8);
	I2C_Send_8(uiAddr & 0x00FF);
	I2C_Send_8(ucData);
	I2C_Stop();
}

void I2C_Write_17bit(unsigned char ucI2CAddr, unsigned long uiAddr, unsigned char ucData)
{
	I2C_Start();
	I2C_Send_8((ucI2CAddr & 0xFC)| ((uiAddr >> 15) & 0x02) );	//Adresse ausgeben RW = 0 und A16 setzen (beachte das Datenblatt: Seite 11 http://cdn.shopify.com/s/files/1/0038/9582/files/m24m01-r.pdf?1271986432
	I2C_Send_8((uiAddr & 0xFF00) >> 8);
	I2C_Send_8(uiAddr & 0x00FF);
	I2C_Send_8(ucData);	//nack
	I2C_Stop();
	_delay_ms(5);
}

unsigned char I2C_Read_17bit(unsigned char ucI2CAddr, unsigned long uiAddr)
{
	unsigned char temp = 0;

	I2C_Start();
	I2C_Send_8((ucI2CAddr & 0xFC) | ((uiAddr >> 15) & 0x02)  );
	I2C_Send_8((uiAddr & 0xFF00) >> 8);
	I2C_Send_8(uiAddr & 0x00FF);
	I2C_Start();
	I2C_Send_8(((ucI2CAddr & 0xFD) | ((uiAddr >> 15) & 0x2)) | 0x01 );	//Lesen
	temp = i2c_rx(0);  //NACK
	I2C_Stop();
	return temp;
}

unsigned char I2C_Read_16bit(unsigned int uiAddr)
{
	unsigned char temp = 0;

	I2C_Start();
	I2C_Send_8(I2C_Addr & 0xFE);	//Adresse ausgeben
	I2C_Send_8((uiAddr & 0xFF00) >> 8);
	I2C_Send_8(uiAddr & 0x00FF);
	I2C_Start();
	I2C_Send_8(I2C_Addr | 0x01);	//Lesen
	temp = i2c_rx(0);  //NACK
	I2C_Stop();
	return temp;
}



unsigned char I2C_Read_8bit(unsigned char ucAddr)
{
	unsigned char temp = 0;
	I2C_Start();
	I2C_Send_8(I2C_Addr & 0xFE);	//Schreiben
	I2C_Send_8(ucAddr);
	I2C_Start();
	I2C_Send_8(I2C_Addr | 0x01);	//Lesen
	temp = i2c_rx(0);  //NACK
	I2C_Stop();
	return temp;
}

void I2C_Write_8bit(unsigned char ucAddr, unsigned char ucData)
{
	I2C_Start();
	I2C_Send_8(I2C_Addr & 0xFE);	//Schreiben
	I2C_Send_8(ucAddr);
	I2C_Send_8(ucData);
	I2C_Stop();
}


