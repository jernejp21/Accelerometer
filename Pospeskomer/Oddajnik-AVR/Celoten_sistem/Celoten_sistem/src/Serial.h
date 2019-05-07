/**
  Serial.h - library for all 3 serial communications available
  on AVR microcontrollers:
  - USART (commonly referred as serial communication)
  - I2C (TWI- two wire serial interface)
  - SPI (Serial Peripheral Interface)

  Copyright (c) 2017 Jernej Pangerc.  All right reserved.

  GCC version used to make this library: 5.4.0 20160609
  Copyright (C) 2015 Free Software Foundation, Inc.
  This is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation.
  There is NO warranty; not even for MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.

  This library is distributed in the hope that it will be useful,


  PAY ATTENTION!

  - This library is made for atmega328p, so far.
*/



#ifndef Serial_h
#define Serial_h
#endif

void SPI_MasterInit(void);
void SPI_Transmit(const char cData);
inline void SPI_SlaveInit(void);
char SPI_Receive(const char cData);
void I2C_Begin(void);
inline void I2C_Transmit(const uint8_t slave_address, const uint8_t reg_address, const uint8_t data);
inline uint8_t I2C_Read_one(const uint8_t slave_address, const uint8_t reg_address);
inline void I2C_Read_array(const uint8_t slave_address, const uint8_t reg_address, const uint8_t len, uint8_t *arrayp);
void Serial_begin(void);
void Serial_write(const uint8_t data);
inline char Serial_read(void);


#define I2C_WRITE 0
#define I2C_READ 1
#define MOSI	3
#define SCK		5
#define MISO	4
#define SS		2

void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	DDRB |= (1<<MOSI)|(1<<SCK)|(1<<SS);
	PORTB |= (1 << SS);
	/* Enable SPI, Master, set clock rate fck/16 */
	//SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	
	/* Enable SPI, Master, set clock rate fck/8 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	SPSR |= (1 << SPI2X);

	/* Enable SPI, Master, set clock rate fck/4 */	
	//SPCR = (1<<SPE)|(1<<MSTR);
}

void SPI_Transmit(const char cData)
{
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
}

void SPI_SlaveInit(void)
{
	/* Set MISO output, all others input */
	DDRB = (1<<MISO);
	/* Enable SPI */
	SPCR = (1<<SPE);
}
char SPI_Receive(const char cData)
{
	/* Start transmission */
	SPDR = cData;
	asm volatile("nop");
	/* Wait for reception complete */
	while(!(SPSR & (1<<SPIF)));
	/* Return Data Register */
	return SPDR;
}

void I2C_Begin(void)
{
    TWBR = 12;
    //TWSR |= 0x03;
	PORTC |= (1 << 4);   // vklopi pull-up za SDA
	PORTC |= (1 << 5);   // vklopi pull-up za SCL
}

inline void I2C_Transmit(const uint8_t slave_address, const uint8_t reg_address, const uint8_t data)
{
    /**start signal*/
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT))); /**wait until bus is free*/


    /**Phase one:
    send slave address - 7-bit address+0 (0 on bit 0 means write, 1 means read)*/
    TWDR = slave_address<<1 | I2C_WRITE;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));


    /**Phase two:
    send register address - 8-bit address*/
    TWDR = reg_address;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));


    /**Phase three:
    send data into register - 8-bit data*/
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));


    /**stop signal*/
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

inline uint8_t I2C_Read_one(const uint8_t slave_address, const uint8_t reg_address)
{
    /**start signal*/
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT))); /**wait until bus is free*/


    /**Phase one:
    send slave address - 7-bit address+0 (0 on bit 0 means write, 1 means read)*/
    TWDR = slave_address<<1 | I2C_WRITE;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));


    /**Phase two:
    send register address - 8-bit address
    Which register we want to read*/
    TWDR = reg_address;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));


    /**start signal once again*/
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT))); /**wait until bus is free*/


    /**Phase three:
    send slave address - 7-bit address+1 (0 on bit 0 means write, 1 means read)
    this time we inform slave to send us register value*/
    TWDR = slave_address<<1 | I2C_READ;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (0==(TWCR & (1<<TWINT)));


    TWCR = (1<<TWINT) | (1<<TWEN);
    while (0==(TWCR & (1<<TWINT)));


    /**stop signal*/
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);


    return TWDR;
}

inline void I2C_Read_array(const uint8_t slave_address, const uint8_t reg_address, const uint8_t len, uint8_t *arrayp)
{
    /**start signal*/
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT))); /**wait until bus is free*/


    /**Phase one:
    send slave address - 7-bit address+0 (0 on bit 0 means write, 1 means read)*/
    TWDR = slave_address<<1 | I2C_WRITE;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));


    /**Phase two:
    send register address - 8-bit address
    Which register we want to read*/
    TWDR = reg_address;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));


    /**start signal once again*/
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT))); /**wait until bus is free*/


    /**Phase three:
    send slave address - 7-bit address+1 (0 on bit 0 means write, 1 means read)
    this time we inform slave to send us register value*/
    TWDR = slave_address<<1 | I2C_READ;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (0==(TWCR & (1<<TWINT)));


    int i=0;

    for(i=0; i<len-1; i++)
    {
        /**master sending ACK signal after data is received*/
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
        while (0==(TWCR & (1<<TWINT)));
        *(arrayp+i) = TWDR;
    }

    /**master sending NACK signal after data is received*/
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (0==(TWCR & (1<<TWINT)));
    *(arrayp+i) = TWDR;


    /**stop signal*/
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void Serial_begin(void)
{
    UBRR0H=UBRRH_VALUE;    //define baud rate
    UBRR0L=UBRRL_VALUE;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);    //turn on TX and RX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);	//8-bit communication
}

void Serial_write(const uint8_t data)
{
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

inline char Serial_read(void)
{
    if ( (UCSR0A & (1<<RXC0)))
    {
        while ( !(UCSR0A & (1<<RXC0)) );
        return UDR0;
    }

    return 0;
}


