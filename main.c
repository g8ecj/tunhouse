/**
 * \file
 * <!--
 * This file is part of Robin's Tunnel house window opener
 *
 * Bertos is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * As a special exception, you may use this file as part of a free software
 * library without restriction.  Specifically, if other files instantiate
 * templates or use macros or inline functions from this file, or you compile
 * this file and link it with other files to produce an executable, this
 * file does not by itself cause the resulting executable to be covered by
 * the GNU General Public License.  This exception does not however
 * invalidate any other reasons why the executable file might be covered by
 * the GNU General Public License.
 *
 * Copyright 2015 Robin Gilks (www.gilks.org)
 *
 * -->
 *
 * \author Robin Gilks (g8ecj@gilks.org)
 *
 * \brief Window opener with nrf24l01 RF remote linking
 *
 * This project uses an i2c connected LCD display, SPI connected RF module, 
 * 1-wire connected sensors, digital i/o to relays or H-bridges, analog to 
 * track battery voltage and a serial interface for debugging.
 */

#include <cfg/debug.h>

#include <cpu/irq.h>

#include <drv/timer.h>
#include <drv/ser.h>

#include "measure.h"
#include "rtc.h"
#include "eeprommap.h"
#include "window.h"
#include "nrf.h"
#include "ui.h"

Serial serial;

#define DEBUG 0


/* I/O pins used by the tunnel house window controller

GND        4

PD2 D2     5     FET driver          } LO motor driver
PD3 D3     6     FET driver          } HI
PD4 D4     7     1-wire              } LO
PD5 D5     8     1-wire              } HI temperature sensors
PD6 D6     9     1-wire              } EX
PD7 D7    10     FET driver          } LO motor driver

PB0 D8    11     CE                  } NRF24L01
PB1 D9    12     CSN                 } interface
PB2 D10   13     FET driver          } HI motor driver
PB3 D11   14     MOSI                }
PB4 D12   15     MISO                }
PB5 D13   16     SCK                 }

PC0 A0    19     up                  }
PC1 A1    20     centre              } buttons
PC2 A2    21     down                }

PC4 A4    23     SDA                 } LCD
PC5 A5    24     SCL                 }

    A6    25     Battery             } analog

5V        27
GND       29
VIN       30
*/



static void
init (void)
{

   /* Initialize system timer */
   timer_init ();

   /* Initialize UART0 */
   ser_init (&serial, SER_UART0);
   /* Configure UART0 to work at 115.200 bps */
   ser_setbaudrate (&serial, 115200);

   // get the config stuff & last time setting
   load_eeprom_values ();

   // real time clock
   rtc_init ();

   // temperature sensors
   measure_init ();

   // open/closing of windows
   window_init ();

   /* Enable all the interrupts */
   IRQ_ENABLE;

   // initialise RF link to remote
   nrf_init();

   // display and button handling
   ui_init ();


}


int
main (void)
{
   uint8_t key = 0;

   init ();

   while (1)
   {
      // keep real time clock stuff up to date
      run_rtc ();
      // run temperature reading stuff on the 1-wire interface
      run_measure ();
      // run state machine for window opening motors
      run_windows ();
      // send data back to base
      key = run_nrf ();
      // display stuff on the LCD & get user input
      run_ui (key);
   }
}

#if DEBUG > 0

extern uint8_t _end;
extern uint8_t __stack;

#define STACK_CANARY  0xc5

void StackPaint (void) __attribute__ ((naked))
   __attribute__ ((section (".init1")));
uint16_t StackCount (void);

void
StackPaint (void)
{
   uint8_t *p = &_end;

   while (p <= &__stack)
   {
      *p = STACK_CANARY;
      p++;
   }
}

uint16_t
StackCount (void)
{
   const uint8_t *p = &_end;
   uint16_t c = 0;

   while (*p == STACK_CANARY && p <= &__stack)
   {
      p++;
      c++;
   }

   return c;
}

#endif
