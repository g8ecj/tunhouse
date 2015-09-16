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
#include <cpu/power.h>

#include <drv/timer.h>
#include <drv/ser.h>

#include <drv/lcd_hd44.h>
#include <drv/term.h>
#include <net/nrf24l01.h>
#include <drv/kbd.h>

static Serial serial;
static Term term;

static const char lcd_degree[8] = { 0x1c, 0x14, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00 };  /* degree - char set B doesn't have it!! */

#define DEGREE 1

#define NOSIGNAL 5000
#define BACKLIGHT 15000

uint8_t addrtx0[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP0;
uint8_t addrtx1[NRF24L01_ADDRSIZE] = NRF24L01_ADDRP1;

int16_t gWinState[2];
int16_t gValues[3][3]; // current, max and min temperatures for each of 3 sensor


#if NRF24L01_PRINTENABLE == 1
static void
debug_prints (const char *s)
{
   kfile_printf (&serial.fd, "%s", s);
}
#endif



static void
init (void)
{

   /* Initialize system timer */
   timer_init ();

   /* Initialize UART0 */
   ser_init (&serial, SER_UART0);
   /* Configure UART0 to work at 115.200 bps */
   ser_setbaudrate (&serial, 115200);

   /* Enable all the interrupts */
   IRQ_ENABLE;

   // set up lcd display
   lcd_hw_init ();
   lcd_remapChar (lcd_degree, DEGREE); // put the degree symbol on character 0x01

   // terminal emulator
   term_init (&term);

   // keyboard (push buttons)
   kbd_init ();

   // initialise RF link to remote
   nrf24l01_init ();
#if NRF24L01_PRINTENABLE == 1
   nrf24l01_printinfo (debug_prints);
#endif



}


int
main (void)
{
   uint16_t c = 0, r = 0, t = 0;
   uint8_t bufferin[33];
   uint8_t bufferout[33];
   uint8_t cursor = false, x = ' ';
   ticks_t backlight_timer, nosignal_timer;
   keymask_t key;


   init ();

   lcd_backlight (1);
   kfile_printf(&term.fd, "%c%c%c%cStarting...", TERM_CLR, TERM_CPC, TERM_ROW + 1, TERM_COL + 5);
   backlight_timer = timer_clock ();
   nosignal_timer = timer_clock ();

   while (1)
   {

      if (nrf24l01_readready (NULL))
      {                         //if data is ready
         //read buffer
         nrf24l01_read (bufferin);

         if (bufferin[0] < '4')
         {
            t = bufferin[0] - '0';
            // if on the cursor row, pick up the character to the left of the cursor
            if (t == r)
               x = bufferin[c+1];
            kfile_printf (&term.fd, "%c%c%c%.20s", TERM_CPC, TERM_ROW + t, TERM_COL + 0, &bufferin[2]);
         }
         // check for backlight on/off commands
         else if (bufferin[0] == 'B')
         {
            lcd_backlight (1);
         }
         else if (bufferin[0] == 'b')
         {
            lcd_backlight (0);
         }
         // cursor on.
         else if (bufferin[0] == 'C')
         {
            r = bufferin[1];
            c = bufferin[2];
            cursor = true;
            kfile_printf (&term.fd, "%c", TERM_BLINK_ON);
         }
         // cursor off
         else if (bufferin[0] == 'c')
         {
            cursor = false;
            kfile_printf (&term.fd, "%c", TERM_BLINK_OFF);
         }

         // binary statistics to send out the serial port
         else if (bufferin[0] == 'S')
         {
            uint32_t now;
            memcpy(&now, &bufferin[1], sizeof(now));
            memcpy(&gValues, &bufferin[5], sizeof(gValues));
            memcpy(&gWinState, &bufferin[5 + sizeof(gValues)], sizeof(gWinState));

         }
         // rewrite the character to the left of the cursor that we extracted earlier
         if (cursor)
            kfile_printf (&term.fd, "%c%c%c%c%c", TERM_CPC, TERM_ROW + r, TERM_COL + c - 1, x, TERM_BLINK_ON);

         nosignal_timer = timer_clock ();
         backlight_timer = timer_clock ();

      }
// just processed a cursor command, should have time to return a keyboard character
//      if ((bufferin[0] == 'C') || (bufferin[0] == 'c'))
      {
         bufferin[0] = 'X';
         key = kbd_peek ();

         if (key == 0)
         {
            // if no pushbutton then check for local serial input
            key = kfile_getc (&serial.fd);
            if ((int16_t) key == EOF)
               key = 0;
         }
         if (key > 0)
         {
            bufferout[0] = 'K';
            bufferout[1] = key;

            // set tx address for pipe 0
            nrf24l01_settxaddr (addrtx0);
            if (nrf24l01_write (bufferout) == 0)
               kfile_printf(&serial.fd, "Key TX failed, tried %d times \r\n", nrf24_retransmissionCount());
         }
      }
      // Notify user if no signal
      if (timer_clock () - nosignal_timer > ms_to_ticks (NOSIGNAL))
      {
         nosignal_timer = timer_clock ();
         if (backlight_timer)
         {
            lcd_backlight (1);
            kfile_printf(&term.fd, "%c%c%c%cNo Signal", TERM_CLR, TERM_CPC, TERM_ROW + 1, TERM_COL + 5);
         }
      }
      // if backlight timer expired (if it exists) turn off backlight.
      if ((backlight_timer) && (timer_clock () - backlight_timer > ms_to_ticks (BACKLIGHT)))
      {
         lcd_backlight (0);
         backlight_timer = 0;
      }

   }
}
