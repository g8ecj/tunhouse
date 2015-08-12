/**
 * \file
 * <!--
 * This file is part of BeRTOS.
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
 * Copyright 2010 Develer S.r.l. (http://www.develer.com/)
 *
 * -->
 *
 * \author Andrea Righi <arighi@develer.com>
 *
 * \brief Empty project.
 *
 * This is a minimalist project, it just initializes the hardware of the
 * supported board and proposes an empty main.
 */

#include <cfg/debug.h>

#include <cpu/irq.h>

#include <drv/timer.h>
#include <drv/ser.h>
#include <drv/term.h>
#include <net/nrf24l01.h>

#include "measure.h"
#include "rtc.h"
#include "eeprommap.h"
#include "window.h"
#include "ui.h"

#include "features.h"


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

#define KEYSTROKE 'K'

Serial serial;

uint8_t addrtx0[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP0;
uint8_t addrtx1[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP1;


static uint8_t
run_nrf (void)
{
   int8_t status = 1, row;
   uint8_t ret = 0, r, c;
   uint8_t buffer[NRF24L01_PAYLOAD];

   // always see if any remote key presses
   if (nrf24l01_readready (NULL))
   {
      //read buffer
      nrf24l01_read (buffer);
      // see if a keyboard command. If so return the keycode
      if (buffer[0] == KEYSTROKE)
         ret = buffer[1];
   }

   // throttle data transfer by only doing every 'n' ms, controlled by the UI
   if (!ui_refresh_check())
      return ret;

   // get a screenfull of data from the UI and send it
   while ((row = ui_termrowget(&buffer[2])) >= 0)
   {
      nrf24l01_settxaddr (addrtx1);
      buffer[0] = row + '0';
      buffer[1] = '0';
      status &= nrf24l01_write(buffer);
      buffer[22] = 0;
      timer_delay(10);
   }

   // get the current cursor address if it is on and send it
   if (ui_termcursorget(&r, &c))
   {
      nrf24l01_settxaddr (addrtx1);
      buffer[0] = 'A';
      buffer[1] = r;
      buffer[2] = c;
      status &= nrf24l01_write(buffer);
   }
   // otherwise send an indication the cursor is off
   else
   {
      nrf24l01_settxaddr (addrtx1);
      buffer[0] = 'C';
      buffer[1] = r;
      buffer[2] = c;
      status &= nrf24l01_write(buffer);
   }

   // debug report via serial interface
   if (status != 1)
   {
      kfile_printf (&serial.fd, "> Tx failed\r\n");

      /* Retranmission count indicates the tranmission quality */
      status = nrf24_retransmissionCount ();
      kfile_printf (&serial.fd, "> Retranmission count: %d\r\n", status);
   }

   // return to RX mode ready for more remote keys
   nrf24l01_setRX();

   return ret;
}


static void
init (void)
{
#if DEBUG
   /* Initialize debugging module (allow kprintf(), etc.) */
   kdbg_init ();
#endif

   /* Initialize system timer */
   timer_init ();

   /*
    * XXX: Arduino has a single UART port that was previously
    * initialized for debugging purpose.
    * In order to activate the serial driver you should disable 
    * the debugging module.
    */
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

   /* init hardware pins */
   nrf24l01_init ();

   // display and button handling
   ui_init ();


}

#if DEBUG == 1 && NRF24L01_PRINTENABLE == 1
static void
debug_prints (const char * s)
{
   kfile_printf(&serial.fd, "%s", s);
}
#endif

int
main (void)
{
   uint8_t key = 0;

   init ();

#if DEBUG == 1 && NRF24L01_PRINTENABLE == 1
   nrf24l01_printinfo (debug_prints);
#endif

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
