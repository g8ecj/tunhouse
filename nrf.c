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

#include <drv/ser.h>
#include <drv/timer.h>
#include <net/nrf24l01.h>
#include "ui.h"
#include "nrf.h"


extern Serial serial;

#define KEYSTROKE 'K'

uint8_t addrtx0[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP0;
uint8_t addrtx1[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP1;

#if NRF24L01_PRINTENABLE == 1
static void
debug_prints (const char * s)
{
   kfile_printf(&serial.fd, "%s", s);
}
#endif


void
nrf_init(void)
{
   /* init hardware pins */
   nrf24l01_init ();
#if NRF24L01_PRINTENABLE == 1
   nrf24l01_printinfo (debug_prints);
#endif
}


uint8_t
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

   nrf24l01_settxaddr (addrtx1);
   // get a screenfull of data from the UI and send it
   while ((row = ui_termrowget(&buffer[2])) >= 0)
   {
      buffer[0] = row + '0';
      buffer[1] = '0';
      buffer[23] = 0;
      status &= nrf24l01_write(buffer);
   }

   // report backlight state
   if (ui_backlight_check())
   {
      buffer[0] = 'B';
   }
   else
   {
      buffer[0] = 'b';
   }
   status &= nrf24l01_write(buffer);

   // get the current cursor address if it is on and send it
   if (ui_termcursorget(&r, &c))
   {
      buffer[0] = 'C';
      buffer[1] = r;
      buffer[2] = c;
   }
   // otherwise send an indication the cursor is off
   else
   {
      nrf24l01_settxaddr (addrtx1);
      buffer[0] = 'c';
      buffer[1] = r;
      buffer[2] = c;
   }
   status &= nrf24l01_write(buffer);



   // debug report via serial interface
   if (status != 1)
   {
      kfile_printf (&serial.fd, "> Tx failed\r\n");

      /* Retranmission count indicates the tranmission quality */
      status = nrf24_retransmissionCount ();
      kfile_printf (&serial.fd, "> Retranmission count: %d\r\n", status);
   }

   return ret;
}

