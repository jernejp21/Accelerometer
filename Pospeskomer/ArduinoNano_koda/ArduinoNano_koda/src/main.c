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




int main (void)
{
	// Initialization code
	//DDRB |= (1<<PB5);
	SPI_MasterInit();
	//SPI_SlaveInit();
	Serial_begin();

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
