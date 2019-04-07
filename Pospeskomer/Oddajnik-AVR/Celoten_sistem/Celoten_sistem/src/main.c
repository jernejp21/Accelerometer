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

#define F_CPU 8000000L
#define BAUD 9600

#include <asf.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include "Serial.h"


#define CE 1
#define CS_NRF 2
#define CS_MPU 5
#define LED 3
#define MPU_NASLOV 0x68

#define SET_CE		(PORTB |= (1 << CE))
#define RESET_CE	(PORTB &= ~(1 << CE))
#define SET_CS_NRF		(PORTB |= (1 << CS_NRF))
#define RESET_CS_NRF	(PORTB &= ~(1 << CS_NRF))
#define SET_CS_MPU		(PORTD |= (1 << CS_MPU))
#define RESET_CS_MPU	(PORTD &= ~(1 << CS_MPU))
#define SET_LED		(PORTD |= (1 << LED))
#define RESET_LED	(PORTD &= ~(1 << LED))

void radioBegin(char *sprejemnik, char *oddajnik);
void radioWrite(uint8_t *paket);
void radioRead(void);
void printWord(int16_t word);

uint8_t test = 0;
int test1 = 0;
uint8_t RxSPI [2];
uint8_t paket[14];

//int16_t podatki[999];

uint8_t zapis = 0;

void printWord(int16_t word){
	//transmiteByte('0'+(word/1000000000));		//za 32 bit števila
	//transmiteByte('0'+(word/100000000)%10);
	//transmiteByte('0'+(word/10000000)%10);
	//transmiteByte('0'+(word/1000000)%10);
	//transmiteByte('0'+(word/100000)%10);
	
	if(word > 0)
	{
		Serial_write('0'+(word/10000)%10);		// za 16 bitna števila
		Serial_write('0'+((word/1000)%10));		
		Serial_write('0'+((word/100)%10));
		Serial_write('0'+((word/10)%10));
		Serial_write('0'+(word % 10));
	}
	else
	{
		word = word * (-1);
		Serial_write('-');
		Serial_write('0'+(word/10000)%10);		// za 16 bitna števila
		Serial_write('0'+((word/1000)%10));
		Serial_write('0'+((word/100)%10));
		Serial_write('0'+((word/10)%10));
		Serial_write('0'+(word % 10));
		
	}
}

ISR(TIMER1_COMPA_vect){		//prekinitvena rutina
	test = 0;
	
	//PORTB &= ~(1 << LED);
	
	TCCR1B = 0;	//izklop timerja 1
	TCNT1=0;
}

int main (void)
{
	
	
	// Initialization code
	
	/*while(1)
	{
		PORTD &= ~(1 << 3);
		_delay_ms(100);
		PORTD |= (1 << 3);
		_delay_ms(100);
		
	}  */
	
	CLKPR = 0x80;
	CLKPR = (1 << 0);
	
	// vklopi prekinitev
	sei();
	
	//TCCR1B |= (1 << CS12) | (1 << CS10);	//vklop timerja 1, prescale 1024
	OCR1A = 15624; //compare register
	TIMSK1 |= (1 << OCIE1A); // vklopi prekinitev za TIM1, COMP A
	
	DDRB |= (1 << CE) | (1 << CS_NRF) | (1 << CS_MPU);	// CS in CE pina sta izhod
	PORTB &= ~(1 << CE);			// CE pin je nizko stanje
	
	DDRD |= (1 << LED) | (1 << CS_MPU);		// LED in CS_MPU sta izhoda
	
	PORTD |= (1 << LED);
	_delay_ms(1000);
	PORTD &= ~(1 << LED);
	
	SET_CS_MPU;
	SET_CS_NRF;
	
	SPI_MasterInit();
	Serial_begin();
	//I2C_Begin();
	
	_delay_ms(10);
	
	RESET_CS_MPU;
	SPI_Transmit(0x6A);
	SPI_Transmit(0x10);
	SET_CS_MPU;
	
	//char oddajnik[5] = {0x88, 0xF0, 0xF0, 0xE8, 0xE8};
	char oddajnik[5] = {0x77, 0xF0, 0xF0, 0xE8, 0xE8};
	char sprejemnik[5] = {0xE1, 0xF0, 0xF0, 0xE8, 0xE8};
	
	
	radioBegin(sprejemnik, oddajnik);
	
	// navij vzorèenje na 1000 Hz
	//I2C_Transmit(MPU_NASLOV, 29, 0x0F);
	/*I2C_Transmit(MPU_NASLOV, 26, 0x00);
	I2C_Transmit(MPU_NASLOV, 27, 0x18);*/
	
	paket[0] = 0;
	
	while(1)
	{
		// Insert application code here, after the board has been initialized.
		
		// Za branje moram pred vsakim SPI_Receive ukazom tudi nekaj poslati,
		// drugaèe povedano, predem hoèem brati, moram nekaj napisati (ponavadi 0xFF),
		// saj je to NOP za nRF24L01+

		/*PORTB &= ~(1 << CS);
		SPI_Transmit(0x04);
		//test = SPI_Receive();
		//SPI_Transmit(0x00);
		//test = SPI_Receive();
		_delay_ms(1);
		PORTB |= (1 << CS);*/
		
		/*if(Serial_read() == 's')
		{
			Serial_write('j');
			test = 1;
			test1 = 0;
			zapis = 1;
			TCCR1B |= (1 << CS12) | (1 << CS10);	//vklop timerja 1, prescale 1024
		}
		
		while(test)
		{	
			//PORTB |= (1 << LED);		
			I2C_Read_array(MPU_NASLOV, 0x3B, 14, paket);
			radioWrite(paket);
			podatki[test1] = (paket[0] << 8) | paket[1];
			test1++;
		}
		
		if(zapis)
		{
			for(int i = 0; i < 999; i++)
			{
				Serial_write('[');
				printWord(i);
				Serial_write(']');
				Serial_write(' ');
				printWord(podatki[i]);
				Serial_write('\n');
			}
			zapis = 0;
		}	 */
		
		/*int16_t acc_x = (paket[0] << 8) + paket[1];
		int16_t acc_y = (paket[2] << 8) + paket[3];
		int16_t acc_z = (paket[4] << 8) + paket[5];

		Serial_write('X');
		Serial_write('=');
		printWord(acc_x);		
		Serial_write('\n');
		Serial_write('Y');
		Serial_write('=');
		printWord(acc_y);
		Serial_write('\n');
		Serial_write('Z');
		Serial_write('=');
		printWord(acc_z);
		Serial_write('\n');*/
		
		//_delay_ms(1000);
		
		//if(Serial_read() == 's')
		//{
			//Serial_write('j');
			//I2C_Read_array(MPU_NASLOV, 0x3B, 14, paket);
			
			RESET_CS_MPU;
			SPI_Transmit(0x3B | (1 << 7));
			paket[0] = SPI_Receive(0);
			paket[1] = SPI_Receive(0);
			paket[2] = SPI_Receive(0);
			paket[3] = SPI_Receive(0);
			paket[4] = SPI_Receive(0);
			paket[5] = SPI_Receive(0);
			paket[6] = SPI_Receive(0);
			paket[7] = SPI_Receive(0);
			paket[8] = SPI_Receive(0);
			paket[9] = SPI_Receive(0);
			paket[10] = SPI_Receive(0);
			paket[11] = SPI_Receive(0);
			paket[12] = SPI_Receive(0);
			paket[13] = SPI_Receive(0);
			SET_CS_MPU;

			/*SPI_Transmit(0x3B | (1 << 7));
			paket[0] = SPI_Receive(0);
			paket[1] = SPI_Receive(0);	 */
									
			/*paket[0] = 0xBB;
			paket[1] = 0xBB;
			paket[2] = 0xBB;
			paket[3] = 0xBB;
			paket[4] = 0xBB;
			paket[5] = 0xBB;
			paket[6] = 0xBB;
			paket[7] = 0xBB;
			paket[8] = 0xBB;
			paket[9] = 0xBB;
			paket[10] = 0xBB;
			paket[11] = 0xBB;
			paket[12] = 0xBB;
			paket[13] = 0xBB;*/
			
			radioWrite(paket);
			_delay_us(1700);
		//}
		//paket[0] = test++;
		
	}
}


void radioBegin(char *sprejemnik, char *oddajnik)
{
	int i = 0;
	// Nastavi RX naslov za 0. pipo
	//LSBajt gre prvi
	RESET_CS_NRF;
	
	SPI_Transmit(0x0A | (1 <<5));
	for(i = 0; i < 5; i++)
	{
		SPI_Transmit(*(oddajnik + i));
	}
	
	SET_CS_NRF;
	
	// Nastavi RX naslov za 1. pipo
	//LSBajt gre prvi
	RESET_CS_NRF;
	
	SPI_Transmit(0x0B | (1 <<5));
	for(i = 0; i < 5; i++)
	{
		SPI_Transmit(*(sprejemnik + i));
	}
	
	SET_CS_NRF;
	
	// Nastavi TX naslov
	//LSBajt gre prvi
	RESET_CS_NRF;
	
	SPI_Transmit(0x10 | (1 <<5));
	for(i = 0; i < 5; i++)
	{
		SPI_Transmit(*(oddajnik + i));
	}
	
	SET_CS_NRF;
	
	// Nastavi 5 bajtov za sprejem za 0. pipo
	RESET_CS_NRF;
	SPI_Transmit(0x11 | (1 << 5));
	SPI_Transmit(14);
	SET_CS_NRF;
	
	// Nastavi 5 bajtov za sprejem za 1. pipo
	RESET_CS_NRF;
	SPI_Transmit(0x12 | (1 << 5));
	SPI_Transmit(14);
	SET_CS_NRF;
	
	// Nastavi RX in vklopi
	RESET_CS_NRF;
	SPI_Transmit(0x00 | (1 << 5));
	SPI_Transmit(0x0F);
	SET_CS_NRF;
	
	// Nastavi frekvenco na 2400 + 76 MHz
	RESET_CS_NRF;
	SPI_Transmit(0x05 | (1 << 5));
	SPI_Transmit(76);
	SET_CS_NRF;
	
	// Nastavi SETUP_RETR register
	RESET_CS_NRF;
	SPI_Transmit(0x04 | (1 << 5));
	SPI_Transmit(0x5F);
	SET_CS_NRF;
	
	// Nastavi RF_SETUP register
	RESET_CS_NRF;
	SPI_Transmit(0x06 | (1 << 5));
	SPI_Transmit(0x08);
	SET_CS_NRF;
	
	// Nastavi STATUS register
	RESET_CS_NRF;
	SPI_Transmit(0x07 | (1 << 5));
	SPI_Transmit(0x70);
	SET_CS_NRF;
	
	// Splakni RX in TXbuffer
	// Splakni RX buffer
	RESET_CS_NRF;
	SPI_Transmit(0xE2);
	SET_CS_NRF;
	
	// Splakni TX buffer
	RESET_CS_NRF;
	SPI_Transmit(0xE1);
	SET_CS_NRF;
}

void radioWrite(uint8_t *paket)
{
	// 1. postavi PRIM_RX = 0
	//postavi CE=0
	RESET_CE;
	//nastavi PRIM_RX=0; TX mode
	RESET_CS_NRF;
	SPI_Transmit(0 | (1 << 5));
	SPI_Transmit(0x0E);
	SET_CS_NRF;

	//pobrišem bite RX_DR, TX_DS in MAX_RT
	RESET_CS_NRF;
	SPI_Transmit(0x07 | (1 << 5));
	SPI_Transmit(0x70);
	SET_CS_NRF;
	
	
	// 2. naloži podatke v pomnilnik
	RESET_CS_NRF;
	SPI_Transmit(0xA0); // naslov pomnilnika
	for(int i = 0; i < 14; i++)
	{
		SPI_Transmit(*(paket + i));
	}
	SET_CS_NRF;

	// 3. pošlji vsaj 10 us pulz na CE za prenos paketa
	SET_CE;
	_delay_us(20);
	RESET_CE;
	
	// 4. ali so podatki prišli?
	RxSPI[0] = 0x17;
	RxSPI[1] = 0xFF;
	
	RESET_CS_NRF;
	//SPI_Transmit(0x17);
	RxSPI[0] = SPI_Receive(0x17);
	//SPI_Transmit(0xFF);
	RxSPI[1] = SPI_Receive(0xFF);
	SET_CS_NRF;
	
	if(RxSPI[0] & (1 << 5))
	{//podatki so bili dostavljeni
		RESET_CS_NRF;
		SPI_Transmit(7 | (1 << 5));
		SPI_Transmit(0x70);
		SET_CS_NRF;
	}
	else
	{// splaknem TX FIFO
		RESET_CS_NRF;
		SPI_Transmit(0xE);
		SET_CS_NRF;
	}
}