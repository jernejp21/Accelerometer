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
void poslji_nastavitev(uint8_t nastavitev);
uint8_t poslusaj_nRF(void);
uint8_t preberem_podatke(void);

void radio_init(char *sprejemnik, char *oddajnik);
void radio_poslji(uint8_t *paket);
void radioRead(void);

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

uint8_t imam_cas = 1;
uint8_t cakalna_vrsta = 0;
uint8_t TX = 0;
uint8_t stevilo_nastavitev = 0;
uint8_t nastavitve[14];
uint8_t paket[14];

char oddajnik[5] = {0x77, 0xF0, 0xF0, 0xE8, 0xE8}; //to sem jaz - to je moj naslov
char sprejemnik[5] = {0xE1, 0xF0, 0xF0, 0xE8, 0xE8};

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
	
}

//Glavna zanka in glavni del programa
int main(void)
{
	// Takoj po zagonu, je potrebno spremeniti frekvenco iz 16 MHz na 8 MHz.
	CLKPR = 0x80;
	CLKPR = (1 << 0);

	//Inizializacija
	sist_init();
	// LED pomežikne, naznanja da je konec inicializacije.
	PORTD |= (1 << LED);
	_delay_ms(1000);
	PORTD &= ~(1 << LED);

	//debagiranje
	Serial_begin();

	//glavni program
	TX = 0;

	stevilo_nastavitev = 0;
	
	while(1)
	{
		// V glavni zanki se preverja, če so prišle nastavitve za poskeškomer
		// ali za oddajnik. V prekinitveni zanki se preberejo podatki iz pospeškomera
		// in pošljejo preko oddajnika.

		// Ko pridem iz prekinitvene rutine, je imam_cas = 1, saj sem ravnokar poslal
		// podatke in imam še čas, da nastavim nastavitve. Če mi zmanjka časa za nas-
		// tavitve, se vrtim v while(1) dokler ne pošljem podatkov; takrat je
		// imam_cas = 0.

		if (imam_cas == 1)
		{
			if (cakalna_vrsta == 1)
			{

				// Določim, koliko časa potrebujem za 1 nastavitev. Čas izvajanja
				// funkcije se določi empirično. Funkcija postavi "imam_cas" na 1, če imam
				// dovolj časa do prekinitve za izvedbo funkcije, v nasprotnem primeru
				// je "imam_cas" = 0.
				dolocim_cas(0);

				if (imam_cas == 1)
				{
					while (1)
					{
						//poslji_nastavitev(nastavitve);
						stevilo_nastavitev--;

						if (stevilo_nastavitev == 0)
						{
							// Ko pošljem vse nastavitve, grem v while(TX == 0), kjer poslušam
							// ali so novi podatki na voljo.
							cakalna_vrsta = 0;
							break;
						}

						// Določim, koliko časa potrebujem za 1 nastavitev. Čas izvajanja
						// funkcije se določi empirično. Funkcija postavi "imam_cas" na 1, če imam
						// dovolj časa do prekinitve za izvedbo funkcije, v nasprotnem primeru
						// je "imam_cas" = 0.
						dolocim_cas(0);

						if (imam_cas == 0)
						{
							// Če še imam čas, pošljem še eno nastavitev na bodisi nRF bodisi MPU.
							// Če nimam več časa, grem v while(1) in čakam do prekinitve. V nasled-
							// njem krogu se bo poslala naslednja nastavitev.
							break;
						}
					}
				}
			}
			else
			{
				// Če ni čakalne vrste (ni potrebno nič nastaviti), poslušam, le je kaj na
				// liniji. Če kaj pride na linijo, preberem podatke. TX = 1 pomeni, da je
				// nekaj prišlo na nRF. Podatke se prebere samo prvič ko pridejo. Če zmanjka
				// časa za obdelavo, se podatki preberejo v naslednjem krogu in čakajo v
				// FIFOtu od nRFja. Vedno se morajo nastaviti najnovejše nastavitve, tako da
				// ni problema, če se prispele nastavitve iz 1. kroga prepišejo z drugimi,
				// medtem ko smo pošiljali podatke ven.

				while (TX == 0)
				{
					// Poslušam, če je kaj na liniji. Če je linija prazna, je TX = 0, če kaj
					// pride, funkcija poslusaj_nRF() spremeni TX na 1.
					TX = poslusaj_nRF();
				}

				// Določim, koliko časa potrebujem za branje podatkov. Čas izvajanja
				// funkcije se določi empirično. Funkcija postavi "imam_cas" na 1, če imam
				// dovolj časa do prekinitve za izvedbo funkcije, v nasprotnem primeru
				// je "imam_cas" = 0.
				dolocim_cas(0);

				if (imam_cas == 1)
				{
					// Preberem podatke, ki so prišli preko nRFja. To so nastavitve tako za
					// nRF kot za MPU. Prebrane podatke se shrani v globalni vektor "nastavitve".
					stevilo_nastavitev = preberem_podatke();

					// Ko sem prebral podatke, imam nastavitve že v čakalni vrsti.
					cakalna_vrsta = 1;

					// Ko se bo čakalna vrsta spraznila, bom zopet pripravljen na poslušanje,
					// tako da tukaj postavimo TX na 0.
					TX = 0;
				}
			}
		}
	}
}

void sist_init()
{
	// Vklopim prekinitve
	sei();

	TCCR1B |= (1 << CS11);	//vklop timerja 1, prescale 8
	OCR1A = PERIODA;			 // compare register
	TIMSK1 |= (1 << OCIE1A); // vklopi prekinitev za TIM1, COMP A

	DDRB |= (1 << CE) | (1 << CS_NRF) | (1 << CS_MPU); // CS in CE pina sta izhod
	PORTB &= ~(1 << CE);							   // CE pin je nizko stanje

	DDRD |= (1 << LED) | (1 << CS_MPU); // LED in CS_MPU sta izhoda

	SET_CS_MPU; // postavi CS pin za MPU v visoko stanje
	SET_CS_NRF; // postavi CS pin za nRF v visoko stanje

	SPI_MasterInit();
	_delay_ms(10);

	// Nastavi MPU na SPI komunikacijo.
	RESET_CS_MPU;
	SPI_Transmit(0x6A);
	SPI_Transmit(0x10);
	SET_CS_MPU;

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

void radio_poslji(uint8_t *paket)
{
	uint8_t RxSPI[2];

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
	for (int i = 0; i < 14; i++)
	{
		SPI_Transmit(*(paket + i));
	}
	SET_CS_NRF;

	// 3. pošlji vsaj 10 us pulz na CE za prenos paketa
	SET_CE;
	_delay_us(20);
	RESET_CE;

	// 4. ali so podatki bili poslani?

	RESET_CS_NRF;
	RxSPI[0] = SPI_Receive(0x17);
	RxSPI[1] = SPI_Receive(0xFF);
	SET_CS_NRF;

	if (RxSPI[0] & (1 << 5))
	{ //podatki so bili dostavljeni, zato počistim TX_DS status bit
		RESET_CS_NRF;
		SPI_Transmit(7 | (1 << 5));
		SPI_Transmit(0x70);
		SET_CS_NRF;
	}
	else
	{ // splaknem TX FIFO
		RESET_CS_NRF;
		SPI_Transmit(0xE);
		SET_CS_NRF;
	}
}

void dolocim_cas(uint16_t cas_za_izvedbo)
{
	// funkcija sprejema parameter koliko časa se izvaja sledeča funkcija.
	// funkcija postavi "imam_cas" na 1, če lahko v preostalem času do prekinitve
	// izvedem sledečo funkcijo.
	// funkcija postavi "imam_cas" na 0, če ne morem v preostalem času do
	// prekinitve izvesti sledeče funkcije.
	//
	// funkcija nič ne vrača, samo spreminja "imam_cas"
	
	uint16_t trenutni_cas = TCNT1;

	if (cas_za_izvedbo > (PERIODA - trenutni_cas))
	{
		imam_cas = 1;
	}
	else
	{
		imam_cas = 0;
	}
}

void poslji_nastavitev(uint8_t nastavitev)
{
	// funkcija ničesar ne vrača.
}

uint8_t poslusaj_nRF()
{
	int podatki_so = 0;
	// Funkcija posluša, če je od sprejemnika pršla kakšna nastavitev.

	return podatki_so;
}

uint8_t preberem_podatke()
{
	// Nastavitve se shranijo v vektor "nastavitve". Vedno se prebere 14 bajtov.
	// To je 7 nastavitev (naslov, vrednost, naslov, vrednost...). Ni nujno, da
	// je vedno 7 nastavitev. Določim tudi "stevilo_nastavitev".

	// funkcija vrača število nastavitev

	return 5;
}