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

#define SET_CE (PORTB |= (1 << CE))
#define RESET_CE (PORTB &= ~(1 << CE))
#define SET_CS_NRF (PORTB |= (1 << CS_NRF))
#define RESET_CS_NRF (PORTB &= ~(1 << CS_NRF))
#define SET_CS_MPU (PORTD |= (1 << CS_MPU))
#define RESET_CS_MPU (PORTD &= ~(1 << CS_MPU))
#define SET_LED (PORTD |= (1 << LED))
#define RESET_LED (PORTD &= ~(1 << LED))

#define PERIODA 1000 //1000 ciklov števca je 1 ms

// Funkcije za debagiranje
void printWord(int16_t word);

void sist_init(void);
void dolocim_cas(uint16_t cas_za_izvedbo);
void poslji_nastavitve(void);
uint8_t poslusaj_nRF(void);
uint8_t preberem_podatke(void);

void radio_init(char *sprejemnik, char *oddajnik);
void radio_poslji(uint8_t *paket);
void radio_sprejmi(void);

volatile unsigned int imam_cas = 1;
volatile unsigned int trenutni_cas;
uint8_t cakalna_vrsta = 0;
volatile unsigned int TX = 0;
uint8_t stevilo_nastavitev = 0;
uint8_t nastavitve[14];
uint8_t paket[14];
uint8_t status;

uint8_t RxSPI[2];

char oddajnik[5] = {0x77, 0xF0, 0xF0, 0xE8, 0xE8}; //to sem jaz - to je moj naslov
char sprejemnik[5] = {0xE1, 0xF0, 0xF0, 0xE8, 0xE8};


// Funkcije za debagiranje
void printWord(int16_t word)
{
	//transmiteByte('0'+(word/1000000000));		//za 32 bit števila
	//transmiteByte('0'+(word/100000000)%10);
	//transmiteByte('0'+(word/10000000)%10);
	//transmiteByte('0'+(word/1000000)%10);
	//transmiteByte('0'+(word/100000)%10);

	if (word > 0)
	{
		Serial_write('0' + (word / 10000) % 10); // za 16 bitna števila
		Serial_write('0' + ((word / 1000) % 10));
		Serial_write('0' + ((word / 100) % 10));
		Serial_write('0' + ((word / 10) % 10));
		Serial_write('0' + (word % 10));
	}
	else
	{
		word = word * (-1);
		Serial_write('-');
		Serial_write('0' + (word / 10000) % 10); // za 16 bitna števila
		Serial_write('0' + ((word / 1000) % 10));
		Serial_write('0' + ((word / 100) % 10));
		Serial_write('0' + ((word / 10) % 10));
		Serial_write('0' + (word % 10));
	}
}
//---------------------------------------------------------------------------------

// Prekinitvena rutina.
ISR(TIMER1_COMPA_vect)
{
	// Preberi podatke s pospeškomera.
	// Preberem Ax, Ay, Az, fix, fiy, fiz in T.
	// Vsak parameter je 16 bitno predznačeno število - int16_t, pri čemer
	// je sestavljen iz dveh 8-bitnih števil.
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
	
	radio_poslji(paket);
	
	RESET_CS_NRF;
	SPI_Transmit(0 | (1 << 5));
	SPI_Transmit(0x3F); // vklopim RX, tako da postavim bit 0 na 1
	SET_CS_NRF;
	
	SET_CE;
	
	// prekinitvena rutina za ledico na pinu D3 (PD3)
	PORTD ^= (1 << LED);
	imam_cas = 1;
	
}

ISR(INT0_vect)
{
	//Najprej izklopim prikinitev za pošiljanje podatkov.
	TCCR1B &= ~(1 << CS11);	//izklop timerja 1
	TCNT1 = 0;
	
	//radio_sprejmi();
	//poslji_nastavitve();
	
	//Pobrišem RX_DR bit
	RESET_CS_NRF;
	SPI_Transmit(0x07 | (1 << 5));
	SPI_Transmit(0x40);
	SET_CS_NRF;
	
	PORTD |= (1 << 7);
	_delay_us(200);
	PORTD &= ~(1 << 7);
	
	//Splaknem RX buffer
	RESET_CS_NRF;
	SPI_Transmit(0xE2);
	SET_CS_NRF;
	
	//Serial_write('I');Serial_write('N');Serial_write('T');Serial_write('0');Serial_write('\n');
	
	//Ko vse nastavin, spet vklopim prekinitev za pošiljanje podatkov
	//vsako mili sekundo.
	TCCR1B |= (1 << CS11);	//vklop timerja 1, prescale 8
}

//Glavna zanka in glavni del programa
int main(void)
{
	// Takoj po zagonu je potrebno spremeniti frekvenco iz 16 MHz na 8 MHz.
	CLKPR = 0x80;
	CLKPR = (1 << 0);
	
	Serial_begin();
	DDRD |= (1 << 7); //pin D7 (PD7)
	

	Serial_write('b');
	//Inizializacija
	sist_init();
	
	// LED pomežikne, naznanja da je konec inicializacije.
	PORTD |= (1 << LED);
	_delay_ms(1000);
	PORTD &= ~(1 << LED);

	//debagiranje

	//glavni program
	TX = 0;

	stevilo_nastavitev = 0;
	
	Serial_write('i');
	TCCR1B |= (1 << CS11);	//vklop timerja 1, prescale 8
	
	SET_CE;
	
	while(1)
	{
		
	}
}

void sist_init()
{
	DDRB |= (1 << CE) | (1 << CS_NRF) | (1 << CS_MPU); // CS in CE pina sta izhod
	PORTB &= ~(1 << CE);							   // CE pin je nizko stanje

	DDRD |= (1 << LED) | (1 << CS_MPU); // LED in CS_MPU sta izhoda
	PORTD |= (1 << PD2); //port PD2 (pin D2) imam pull-up.
	
	// Vklopim prekinitve
	sei();
	
	// Prekinitev za TIM1; na vsako ms pošljem paket podatkov
	TCCR1B |= (1 << WGM12); // CTC način delovanja (clear timer on compare)
	OCR1A = PERIODA;			 // compare register
	TIMSK1 |= (1 << OCIE1A); // vklopi prekinitev za TIM1, COMP A
	
	Serial_write('a');
	//Zunanja prekinitev na pinu IRQ (PD2). Če pride kaj na sprejemnik,
	//spremenim nastavitve.
	EICRA |= (1 << ISC01); //prekinitev na padajočem robu.
	EIMSK |= (1 << INT0); //aktiviram zunanjo prekinitev INT0 na pinu PD2.

	Serial_write('a');

	SET_CS_MPU; // postavi CS pin za MPU v visoko stanje
	SET_CS_NRF; // postavi CS pin za nRF v visoko stanje

	SPI_MasterInit();
	_delay_ms(10);

	// Nastavi MPU na SPI komunikacijo.
	RESET_CS_MPU;
	SPI_Transmit(0x6A);
	SPI_Transmit(0x10);
	SET_CS_MPU;


	//Inizializiraj nRF
	radio_init(sprejemnik, oddajnik);
}

void radio_init(char *sprejemnik, char *oddajnik)
{
	int i = 0;
	// Nastavi RX naslov za 0. pipo
	//LSBajt gre prvi
	RESET_CS_NRF;
	SPI_Transmit(0x0A | (1 << 5));
	for (i = 0; i < 5; i++)
	{
		SPI_Transmit(*(oddajnik + i));
	}
	SET_CS_NRF;

	// Nastavi RX naslov za 1. pipo
	//LSBajt gre prvi
	RESET_CS_NRF;
	SPI_Transmit(0x0B | (1 << 5));
	for (i = 0; i < 5; i++)
	{
		SPI_Transmit(*(sprejemnik + i));
	}
	SET_CS_NRF;

	// Nastavi TX naslov
	//LSBajt gre prvi
	RESET_CS_NRF;
	SPI_Transmit(0x10 | (1 << 5));
	for (i = 0; i < 5; i++)
	{
		SPI_Transmit(*(oddajnik + i));
	}
	SET_CS_NRF;

	// Nastavi 14 bajtov za sprejem za 0. pipo
	RESET_CS_NRF;
	SPI_Transmit(0x11 | (1 << 5));
	SPI_Transmit(14);
	SET_CS_NRF;

	// Nastavi 14 bajtov za sprejem za 1. pipo
	RESET_CS_NRF;
	SPI_Transmit(0x12 | (1 << 5));
	SPI_Transmit(14);
	SET_CS_NRF;

	// Nastavi RX in vklopi
	RESET_CS_NRF;
	SPI_Transmit(0x00 | (1 << 5));
	SPI_Transmit(0x3F);
	SET_CS_NRF;

	// Nastavi frekvenco na 2400 + 76 MHz
	RESET_CS_NRF;
	SPI_Transmit(0x05 | (1 << 5));
	SPI_Transmit(76);
	SET_CS_NRF;

	// Nastavi SETUP_RETR register
	RESET_CS_NRF;
	SPI_Transmit(0x04 | (1 << 5));
	SPI_Transmit(0x03);
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

	// Splakni RX in TX buffer
	// Splakni RX buffer
	RESET_CS_NRF;
	SPI_Transmit(0xE2);
	SET_CS_NRF;

	// Splakni TX buffer
	RESET_CS_NRF;
	SPI_Transmit(0xE1);
	SET_CS_NRF;
}

void radio_poslji(uint8_t *paket)
{
	// 1. postavi PRIM_RX = 0
	//postavi CE=0
	RESET_CE;
	//nastavi PRIM_RX=0; TX mode
	RESET_CS_NRF;
	SPI_Transmit(0 | (1 << 5));
	SPI_Transmit(0x3E);
	SET_CS_NRF;
	
	_delay_us(150);

	//pobri�em bite RX_DR, TX_DS in MAX_RT
	RESET_CS_NRF;
	SPI_Transmit(0x07 | (1 << 5));
	SPI_Transmit(0x70);
	SET_CS_NRF;
	
	
	// 2. nalo�i podatke v pomnilnik
	RESET_CS_NRF;
	SPI_Transmit(0xA0); // naslov pomnilnika
	for(int i = 0; i < 14; i++)
	{
		SPI_Transmit(*(paket + i));
	}
	SET_CS_NRF;

	// 3. po�lji vsaj 10 us pulz na CE za prenos paketa
	SET_CE;
	_delay_us(300);
	RESET_CE;
	
	// 4. ali so podatki pri�li?
	RESET_CS_NRF;
	RxSPI[0] = SPI_Receive(0x17);
	RxSPI[1] = SPI_Receive(0xFF);
	SET_CS_NRF;
	
	//Serial_write('S');
	
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
		SPI_Transmit(0xE1);
		SET_CS_NRF;
	}
}

void radio_sprejmi()
{
	//Preberi podatke iz nRFja
	// 1. preberi podatke iz RX FIFOTA
	RESET_CS_NRF;
	SPI_Transmit(0x61);
	nastavitve[0] = SPI_Receive(0);
	nastavitve[1] = SPI_Receive(0);
	nastavitve[2] = SPI_Receive(0);
	nastavitve[3] = SPI_Receive(0);
	nastavitve[4] = SPI_Receive(0);
	nastavitve[5] = SPI_Receive(0);
	nastavitve[6] = SPI_Receive(0);
	nastavitve[7] = SPI_Receive(0);
	nastavitve[8] = SPI_Receive(0);
	nastavitve[9] = SPI_Receive(0);
	nastavitve[10] = SPI_Receive(0);
	nastavitve[11] = SPI_Receive(0);
	nastavitve[12] = SPI_Receive(0);
	nastavitve[13] = SPI_Receive(0);
	SET_CS_NRF;
	
	// 2. pocisti RX_DR IRQ zastavico
	RESET_CS_NRF;
	SPI_Transmit(0x07 | (1 << 5));
	SPI_Transmit(0x70);
	SET_CS_NRF;
	
	// 2. splakni RX pomnilnik
	RESET_CS_NRF;
	SPI_Transmit(0x07 | (1 << 5));
	SPI_Transmit(0x70);
	SET_CS_NRF;
	
	
}

void dolocim_cas(uint16_t cas_za_izvedbo)
{
	// Funkcija sprejema parameter koliko časa se izvaja sledeča funkcija. Čas
	// je v mikro sekundah.
	// Funkcija postavi "imam_cas" na 1, če lahko v preostalem času do prekinitve
	// izvedem sledečo funkcijo.
	// Funkcija postavi "imam_cas" na 0, če ne morem v preostalem času do
	// prekinitve izvesti sledeče funkcije.
	//
	// Funkcija nič ne vrača, samo spreminja "imam_cas".
	
	trenutni_cas = TCNT1;

	if (cas_za_izvedbo < (PERIODA - trenutni_cas))
	{
		imam_cas = 1;
	}
	else
	{
		imam_cas = 0;
	}
}

void poslji_nastavitve()
{
	// funkcija ničesar ne vrača.
	int i = 0;
	for (i = 0; i < 15; i=i+2)
	{
		if (nastavitve[i] != 0xFF)
		{
			if (nastavitve[i] < 0x12)
			{
				//paket pošljem na MPU
				RESET_CS_MPU;
				SPI_Transmit(nastavitve[i]);
				SPI_Transmit(nastavitve[i+1]);
				SET_CS_MPU;
			}
			else
			{
				//paket pošljem na nRF
				RESET_CS_NRF;
				SPI_Transmit(nastavitve[i] | (1 << 5));
				SPI_Transmit(nastavitve[i+1]);
				SET_CS_NRF;
			}
		}
	}
}

uint8_t poslusaj_nRF()
{
	int podatki_so = 0;
	// Funkcija posluša, če je od sprejemnika pršla kakšna nastavitev.
	
	if (Serial_read() == 's')
	{
		podatki_so = 1;
		Serial_write('P');
	}

	return podatki_so;
}

uint8_t preberem_podatke()
{
	// Nastavitve se shranijo v vektor "nastavitve". Vedno se prebere 14 bajtov.
	// To je 7 nastavitev (naslov, vrednost, naslov, vrednost...). Ni nujno, da
	// je vedno 7 nastavitev. Določim tudi "stevilo_nastavitev".

	// funkcija vrača število nastavitev
	
	Serial_write('n');

	return 5;
}