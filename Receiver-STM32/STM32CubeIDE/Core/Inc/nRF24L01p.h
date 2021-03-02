#ifndef __nRF24L01p_h
#define __nRF24L01p_h

/*Knjižnjica, ki vključuje vsa imena registrov in bitov ter njihove lokacije.*/

//Imena ukazov

#define R_REGISTER(register) (register) // 1 do 5 bytov, LSByte; ukaz za branje
#define W_REGISTER(register) (register | (1 << 5)) // 1 do 5 bytov, LSByte; ukaz za pisanje
#define R_RX_PAYLOAD 0b01100001
#define W_TX_PAYLOAD 0b10100000
#define FLUSH_TX 0b11100001
#define FLUSH_RX 0b11100010
#define REUSE_TX_PL 0b11100011
#define R_RX_PL_WID 0b01100000
#define W_ACK_PAYLOAD(pipe) (0b10101000 | pipe)
#define W_TX_PAYLOAD_NOACK 0b10110000
#define NOP 0xFF


// Imena registrov in imena bitov v registru

// 0x00 CONFIG
#define CONFIG 0x00
#define MASK_RX_DR 6
#define MASK_TX_DR 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0

// 0x01 EN_AA
#define EN_AA 0x01
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0

// 0x02 EN_RXADDR
#define EN_RXADDR 0x02
#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0

//0x03 SETUP_AW
#define SETUP_AW 0x03
#define AW 0

//0x04 SETUP_RETR
#define SETUP_RETR 0x04
#define ADR 4
#define ARC 0

//0x05 RF_CH
#define RF_CH 0x05
#define RF_CHAN 0

//0x06 RF_SETUP
#define RF_SETUP 0x06
#define CONT_WAVE 7
#define RF_DR_LOW 5
#define PLL_LOCK 4
#define RF_DR_HIGH 3
#define RF_PWR 1

//0x07 STATUS
#define STATUS 0x07
#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4
#define RX_P_NO 1
#define STATUS_TX_FULL 0

//0x08 OBSERVE_TX
#define PLOS_CNT 4
#define ARC_CNT 0

//0x09 RPD
#define RPD 0x09
#define RPD_RPD 0

//0x0A RX_ADDR_P0
#define RX_ADDR_P0 0x0A

//0x0B RX_ADDR_P1
#define RX_ADDR_P1 0x0B

//0x0C RX_ADDR_P2
#define RX_ADDR_P2 0x0C

//0x0D RX_ADDR_P3
#define RX_ADDR_P3 0x0D

//0x0E RX_ADDR_P4
#define RX_ADDR_P4 0x0E

//0x0F RX_ADDR_P5
#define RX_ADDR_P5 0x0F

//0x10 TX_ADDR
#define TX_ADDR 0x10

//0x11 RX_PW_P0
#define RX_PW_P0 0x11

//0x12 RX_PW_P1
#define RX_PW_P1 0x12

//0x13 RX_PW_P2
#define RX_PW_P2 0x13

//0x14 RX_PW_P3
#define RX_PW_P3 0x14

//0x15 RX_PW_P4
#define RX_PW_P4 0x15

//0x16 RX_PW_P5
#define RX_PW_P5 0x16

//0x17 FIFO_STATUS
#define FIFO_STATUS 0x17
#define TX_REUSE 6
#define FIFO_STATUS_TX_FULL 5
#define TX_EMPTY 4
#define RX_FULL 1
#define RX_EMPTY

//0x1C DYNPD
#define DYNPD 0x1C
#define DPL_P5 5
#define DPL_P4 4
#define DPL_P3 3
#define DPL_P2 2
#define DPL_P1 1
#define DPL_P0 0

//0x1D FEATURE
#define FEATURE 0x1D
#define EN_DPL 2
#define EN_ACK_PAY 1
#define EN_DYN_ACK 0


#endif
