/*
nrf24l01 lib 0x02

copyright (c) Davide Gironi, 2012

References:
  -  This library is based upon nRF24L01 avr lib by Stefan Engelke
     http://www.tinkerer.eu/AVRLib/nRF24L01
  -  and arduino library 2011 by J. Coliz
     http://maniacbug.github.com/RF24

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/

#ifndef _CFG_NRF24L01_H_
#define _CFG_NRF24L01_H_

#include <avr/io.h>

//power setup
#define NRF24L01_RF24_PA NRF24L01_RF24_PA_MAX

//speed setup
#define NRF24L01_RF24_SPEED NRF24L01_RF24_SPEED_1MBPS

//crc setup
#define NRF24L01_RF24_CRC NRF24L01_RF24_CRC_16

//transmission channel
#define NRF24L01_CH 76

//payload lenght
#define NRF24L01_PAYLOAD 16

//auto ack enabled
#define NRF24L01_ACK 1

//auto retransmit delay and count
#define NRF24L01_RETR (0b0100 << NRF24L01_REG_ARD) | (0b0111 << NRF24L01_REG_ARC)       //1500uS, 15 times

//enable / disable pipe
#define NRF24L01_ENABLEDP0 1    //pipe 0
#define NRF24L01_ENABLEDP1 1    //pipe 1
#define NRF24L01_ENABLEDP2 1    //pipe 2
#define NRF24L01_ENABLEDP3 1    //pipe 3
#define NRF24L01_ENABLEDP4 1    //pipe 4
#define NRF24L01_ENABLEDP5 1    //pipe 5

//pipe address
#define NRF24L01_ADDRP0 {0xE8, 0xE8, 0xF0, 0xF0, 0xE2}  //pipe 0, 5 byte address
#define NRF24L01_ADDRP1 {0xC1, 0xC2, 0xC2, 0xC2, 0xC2}  //pipe 1, 5 byte address
#define NRF24L01_ADDRP2 {0xC1, 0xC2, 0xC2, 0xC2, 0xC3}  //pipe 2, 5 byte address
#define NRF24L01_ADDRP3 {0xC1, 0xC2, 0xC2, 0xC2, 0xC4}  //pipe 3, 5 byte address
#define NRF24L01_ADDRP4 {0xC1, 0xC2, 0xC2, 0xC2, 0xC5}  //pipe 4, 5 byte address
#define NRF24L01_ADDRP5 {0xC1, 0xC2, 0xC2, 0xC2, 0xC6}  //pipe 5, 5 byte address
#define NRF24L01_ADDRTX {0xE8, 0xE8, 0xF0, 0xF0, 0xE2}  //tx default address*/

 //enable print info function
#define NRF24L01_PRINTENABLE 1


#endif
