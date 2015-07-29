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

#include "measure.h"
#include "rtc.h"
#include "eeprommap.h"
#include "window.h"
#include "ui.h"
#include "nrf24.h"

#include "features.h"


/* I/O pins used by the tunnel house window controller

PD2 D2    1-wire or FET drive } LO motor drivers
PD3 D3              FET drive }
PD4 D4    1-wire or FET drive } HI
PD5 D5              FET drive }

PD6 D6    1-wire              } LO
PD7 D7    1-wire              } HI temperature sensors
PB0 D8    1-wire              } EX

PB1 D9    CSN                 }
PB2 D10   SS                  }
PB3 D11   MOSI                } NRF24L01
PB4 D12   MISO                } interface
PB5 D13   SCK                 }

PC0 A0    up                  }
PC1 A1    centre              } buttons
PC2 A2    down                }
PC3 A3
PC4 A4    SDA                 } LCD
PC5 A5    SCL                 }

    A6    Battery             } analog
    A7
*/



Serial serial;

uint8_t tx_address[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
uint8_t rx_address[5] = { 0xD7, 0xD7, 0xD7, 0xD7, 0xD7 };



static void
nrf_init (void)
{
   /* init hardware pins */
   nrf24_init ();

   /* Channel #2 , payload length: whatever */
   nrf24_config (2, sizeof (gValues));

   /* Set the device addresses */
   nrf24_tx_address (tx_address);
   nrf24_rx_address (rx_address);
}


static void
run_nrf (void)
{
   uint8_t temp;
   /* Automatically goes to TX mode */
   nrf24_send ((uint8_t *)gValues);

   /* Wait for transmission to end */
   while (nrf24_isSending ());

   /* Make analysis on last tranmission attempt */
   temp = nrf24_lastMessageStatus ();

   if (temp == NRF24_TRANSMISSON_OK)
   {
//            xprintf("> Tranmission went OK\r\n");
   }
   else if (temp == NRF24_MESSAGE_LOST)
   {
//            xprintf("> Message is lost ...\r\n");    
   }

   /* Retranmission count indicates the tranmission quality */
   temp = nrf24_retransmissionCount ();
//    xprintf("> Retranmission count: %d\r\n",temp);

   /* Optionally, go back to RX mode ... */
   nrf24_powerUpRx ();

   /* Or you might want to power down after TX */
   // nrf24_powerDown();            

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
   nrf_init ();

   // display and button handling
   ui_init ();


}

extern void ui_tst(void);

int
main (void)
{

   init ();

   while (1)
   {
      // keep real time clock stuff up to date
      run_rtc ();
      // run temperature reading stuff on the onewire interface
      run_measure ();
      // run state machine for window opening motors
      run_windows ();
      // send data back to base
//      run_nrf ();
      // display stuff on the LCD & get user input
      run_ui ();
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
#if 1
   uint8_t *p = &_end;

   while (p <= &__stack)
   {
      *p = STACK_CANARY;
      p++;
   }
#else
   __asm volatile ("    ldi r30,lo8(_end)\n" "    ldi r31,hi8(_end)\n" "    ldi r24,lo8(0xc5)\n"   /* STACK_CANARY = 0xc5 */
                   "    ldi r25,hi8(__stack)\n"
                   "    rjmp .cmp\n"
                   ".loop:\n"
                   "    st Z+,r24\n"
                   ".cmp:\n"
                   "    cpi r30,lo8(__stack)\n"
                   "    cpc r31,r25\n" "    brlo .loop\n" "    breq .loop"::);
#endif
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
