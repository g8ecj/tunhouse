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

#ifndef _HW_NRF24L01_H_
#define _HW_NRF24L01_H_

#include <avr/io.h>
#include "cfg/macros.h"   /* BV() */


//CE and CSN functions
#define nrf24l01_CSNhi PORTB |=  BV(PB2);
#define nrf24l01_CSNlo PORTB &= ~BV(PB2);
#define nrf24l01_CEhi  PORTB |=  BV(PB1);
#define nrf24l01_CElo  PORTB &= ~BV(PB1);

#define nrf24l01_hw_init              \
	do {                               \
	DDRB |= (1<<PB2); \
	DDRB |= (1<<PB1);  \
	} while (0)




#endif
