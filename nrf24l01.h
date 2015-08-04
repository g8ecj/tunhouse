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

#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include <avr/io.h>
#include "cfg/cfg_nrf24l01.h"


//address size
#define NRF24L01_ADDRSIZE 5

/**
 * \name Values for CONFIG_NRF24L01_RF24_PA.
 *
 * Select the power output of the transmitter
 * $WIZ$ nrf24l01_rf24_power = "NRF24L01_RF24_PA_MIN" "NRF24L01_RF24_PA_LOW" "NRF24L01_RF24_PA_HIGH" "NRF24L01_RF24_PA_MAX"
 */
#define NRF24L01_RF24_PA_MIN 1
#define NRF24L01_RF24_PA_LOW 2
#define NRF24L01_RF24_PA_HIGH 3
#define NRF24L01_RF24_PA_MAX 4

/**
 * \name Values for CONFIG_NRF24L01_RF24_SPEED
 *
 * Select the power output of the transmitter
 * $WIZ$ nrf24l01_rf24_speed = "NRF24L01_RF24_SPEED_250KBPS" "NRF24L01_RF24_SPEED_1MBPS" "NRF24L01_RF24_SPEED_2MBPS"
 */
#define NRF24L01_RF24_SPEED_250KBPS 1
#define NRF24L01_RF24_SPEED_1MBPS 2
#define NRF24L01_RF24_SPEED_2MBPS 3

/**
 * \name Values for CONFIG_NRF24L01_RF24_CRC
 *
 * Select the power output of the transmitter
 * $WIZ$ nrf24l01_rf24_crc = "NRF24L01_RF24_CRC_DISABLED" "NRF24L01_RF24_CRC_8" "NRF24L01_RF24_CRC_16"
 */
#define NRF24L01_RF24_CRC_DISABLED 1
#define NRF24L01_RF24_CRC_8 2
#define NRF24L01_RF24_CRC_16 3

extern void nrf24l01_init(void);
extern uint8_t nrf24l01_getstatus(void);
extern uint8_t nrf24l01_readready(uint8_t* pipe);
extern void nrf24l01_read(uint8_t *data);
extern uint8_t nrf24l01_write(uint8_t *data);
extern void nrf24l01_setrxaddr(uint8_t channel, uint8_t *addr);
extern void nrf24l01_settxaddr(uint8_t *addr);
extern uint8_t nrf24_retransmissionCount(void);
extern void nrf24l01_setRX(void);
#if NRF24L01_PRINTENABLE == 1
extern void nrf24l01_printinfo(void);
#endif

#endif
