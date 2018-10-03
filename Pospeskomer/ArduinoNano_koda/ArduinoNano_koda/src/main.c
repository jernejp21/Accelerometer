/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */

#define F_CPU 16000000L
#define BAUD 9600

#include <asf.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include "Serial.h"


#define CE PD5
#define CS PD6

#define SET_CE		(PORTD |= (1 << CE))
#define RESET_CE	(PORTD &= ~(1 << CE))
#define SET_CS		(PORTD |= (1 << CS))
#define RESET_CS	(PORTD &= ~(1 << CS))

void radioBegin(char *sprejemnik, char *oddajnik);
void radioWrite(void);
void radioReare(void);


int main (void)
{
	// Initialization code
	//DDRB |= (1<<PB5);
	SPI_MasterInit();
	//SPI_SlaveInit();
	Serial_begin();
	
	char oddajnik[5] = {0xE1, 0xF0, 0xF0, 0xE8, 0xE8};
	char sprejemnik[5] = {0x88, 0xF0, 0xF0, 0xE8, 0xE8};
	
	radioBegin(sprejemnik, oddajnik);

	while(1)
	{
		//PORTB ^= (1<<PB5);
		//_delay_ms(100);
		
		SPI_MasterTransmit('a');
		
		//char test = SPI_SlaveReceive();
		//Serial_write(test);
		
	}

	// Insert application code here, after the board has been initialized.
}


void radioBegin(char *sprejemnik, char *oddajnik)
{
	// Nastavi RX naslov za 0. pipo
	//LSBajt gre prvi
	RESET_CS;
	
	SPI_MasterTransmit(0x0A | (1 <<5));
	for(int i = 0; i < 5; i++)
	{
		SPI_MasterTransmit(*oddajnik++);
	}
	
	SET_CS;
	
	// Nastavi RX naslov za 1. pipo
	//LSBajt gre prvi
	RESET_CS;
	
	SPI_MasterTransmit(0x0B | (1 <<5));
	for(int i = 0; i < 5; i++)
	{
		SPI_MasterTransmit(*sprejemnik++);
	}
	
	SET_CS;
	
	// Nastavi TX naslov
	//LSBajt gre prvi
	RESET_CS;
	
	SPI_MasterTransmit(0x0A | (1 <<5));
	for(int i = 0; i < 5; i++)
	{
		SPI_MasterTransmit(*oddajnik++);
	}
	
	SET_CS;
	
	// Nastavi 5 bajtov za sprejem za 0. pipo
	RESET_CS;
	SPI_MasterTransmit(0x11 | (1 << 5));
	SPI_MasterTransmit(32);
	SET_CS;
	
	// Nastavi 5 bajtov za sprejem za 1. pipo
	RESET_CS;
	SPI_MasterTransmit(0x12 | (1 << 5));
	SPI_MasterTransmit(32);
	SET_CS;
	
	// Nastavi RX in vklopi
	RESET_CS;
	SPI_MasterTransmit(0x00 | (1 << 5));
	SPI_MasterTransmit(0x0F);
	SET_CS;
	
	// Nastavi frekvenco na 2400 + 76 MHz
	RESET_CS;
	SPI_MasterTransmit(0x05 | (1 << 5));
	SPI_MasterTransmit(76);
	SET_CS;
	
	// Nastavi SETUP_RETR register
	RESET_CS;
	SPI_MasterTransmit(0x04 | (1 << 5));
	SPI_MasterTransmit(0x5F);
	SET_CS;
	
	// Nastavi RF_SETUP register
	RESET_CS;
	SPI_MasterTransmit(0x06 | (1 << 5));
	SPI_MasterTransmit(0x02);
	SET_CS;
	
	// Nastavi STATUS register
	RESET_CS;
	SPI_MasterTransmit(0x07 | (1 << 5));
	SPI_MasterTransmit(0x70);
	SET_CS;
	
	// Splakni RX in TXbuffer
	// Splakni RX buffer
	RESET_CS;
	SPI_MasterTransmit(0xE2);
	SET_CS;
	
	// Splakni TX buffer
	RESET_CS;
	SPI_MasterTransmit(0xE1);
	SET_CS;
}