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
 * \brief Interface to the 1-wire sensors
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <avr/io.h>

#include <algo/crc8.h>

#include <io/kfile.h>

#include <drv/ow_1wire.h>
#include <drv/ow_ds2413.h>
#include <drv/ow_ds18x20.h>
#include <drv/ser.h>
#include "minmax.h"
#include "rtc.h"
#include "analog.h"
#include "eeprommap.h"
#include "measure.h"
#include "window.h"



MINMAX daymax[NUMSENSORS];
MINMAX daymin[NUMSENSORS];


int16_t gValues[NUMSENSORS][NUMINDEX]; // current, max and min temperatures for each sensor
int16_t gLimits[NUMSENSORS][NUMLIMIT]; // upper and lower limits for driving window motors
int16_t gBattery;
int16_t gBatCal;
int16_t gCurrent[NUMSENSORS];
int16_t gStall[NUMSENSORS];

uint8_t gpioid = 0;
uint8_t gthermid = 0;
uint32_t lasthour;


// do a bit of init for testing
void
measure_init (void)
{
   uint8_t i, j;

   lasthour = uptime ();
   // initialise all the min/max buffers (hourly and daily)
   for (i = 0; i < NUMSENSORS; i++)
   {
      minmax_init (&daymin[i], 24, false);
      minmax_init (&daymax[i], 24, true);
      for (j = 0; j < NUMINDEX; j++)
         gValues[i][j] = 0;
   }

   // start off temperature conversion on all sensors
   if (ow_set_bus (&PIND, &PORTD, &DDRD, PD4) == 0)          // SENSOR_LOW
   {
      ow_ds18x20_resolution(NULL, 11);
      ow_ds18X20_start (NULL, false);
      while (ow_busy());
      ow_ds18X20_start (NULL, false);
   }
   else
   {
      minmax_add (&daymin[SENSOR_LOW], 0);      // clear min/max if no sensor
      minmax_add (&daymax[SENSOR_LOW], 0);
   }

   if (ow_set_bus (&PIND, &PORTD, &DDRD, PD5) == 0)          // SENSOR_HIGH
   {
      ow_ds18x20_resolution(NULL, 11);
      ow_ds18X20_start (NULL, false);
      while (ow_busy());
      ow_ds18X20_start (NULL, false);
   }
   else
   {
      minmax_add (&daymin[SENSOR_HIGH], 0);
      minmax_add (&daymax[SENSOR_HIGH], 0);
   }

   if (ow_set_bus (&PIND, &PORTD, &DDRD, PD6) == 0)          // SENSOR_OUT
   {
      ow_ds18x20_resolution(NULL, 11);
      ow_ds18X20_start (NULL, false);
      while (ow_busy());
      ow_ds18X20_start (NULL, false);
   }
   else
   {
      minmax_add (&daymin[SENSOR_OUT], 0);
      minmax_add (&daymax[SENSOR_OUT], 0);
   }
}


// get current value and limts - used by window motor control
int8_t
getlims (uint8_t sensor, int16_t * now, int16_t * up, int16_t * down)
{
   int8_t ret;

   if ((sensor == SENSOR_LOW) || (sensor == SENSOR_HIGH))
   {
      *now = gValues[sensor][TINDEX_NOW];
      *up = gLimits[sensor][LIMIT_UP];
      *down = gLimits[sensor][LIMIT_DN];
      ret = true;
   }
   else
      ret = false;
   return ret;
}

// keep temperature values between -50 and 90 degrees
static int16_t
validate_value(int16_t value)
{
   if (value < -9990)
      return -9990;
   if (value > 9990)
      return 9990;
   return value;
}

#define ALPHA 0.15
// poll round our sensors in turn, if conversion finished then note the value and start a new conversion
void
run_measure (void)
{
   static char rotate = 0;
   int16_t t;
   int8_t i;
   uint16_t volts;

   gBattery = (uint32_t) analog_read (6) * V_SCALE * (10000 + gBatCal) / 100000;
   volts = analog_read (3) / RSHUNT;
   gCurrent[SENSOR_LOW] = (int16_t) ((ALPHA * (float) volts) + (1 - ALPHA) * (float) gCurrent[SENSOR_LOW]);

   volts = analog_read (7) / RSHUNT;
   gCurrent[SENSOR_HIGH] = (int16_t) ((ALPHA * (float) volts) + (1 - ALPHA) * (float) gCurrent[SENSOR_HIGH]);

#if 0
extern Serial serial;
   static uint8_t pass = 0;

   if (++pass > 10)
   {
      pass = 0;
      kfile_printf(&serial.fd, "V %d\n", gBattery);
//      kfile_printf(&serial.fd, "I dn %d\n", gCurrent[SENSOR_LOW]);
//      kfile_printf(&serial.fd, "I up %d\n", gCurrent[SENSOR_HIGH]);
   }
#endif

   // do one sensor each time round
   switch (rotate & 3)
   {
   case SENSOR_LOW:
      // low level sensor
      if (ow_set_bus (&PIND, &PORTD, &DDRD, PD4))          // SENSOR_LOW
         break;
      if (!ow_busy ())
      {
         if (ow_ds18X20_read_temperature (NULL, &t))
         {
            t = validate_value(t);
            gValues[SENSOR_LOW][TINDEX_NOW] = t;
            minmax_add (&daymin[SENSOR_LOW], t);
            minmax_add (&daymax[SENSOR_LOW], t);
         }
         ow_ds18X20_start (NULL, false);
     }
      break;
   case SENSOR_HIGH:
      // high level (roof) sensor
      if (ow_set_bus (&PIND, &PORTD, &DDRD, PD5))         // SENSOR_HIGH
         break;
      if (!ow_busy ())
      {
         if (ow_ds18X20_read_temperature (NULL, &t))
         {
            t = validate_value(t);
            gValues[SENSOR_HIGH][TINDEX_NOW] = t;
            minmax_add (&daymin[SENSOR_HIGH], t);
            minmax_add (&daymax[SENSOR_HIGH], t);
         }
         ow_ds18X20_start (NULL, false);
      }
      break;
   case SENSOR_OUT:
      // outside sensor
      if (ow_set_bus (&PIND, &PORTD, &DDRD, PD6))          // SENSOR_OUT
         break;
      if (!ow_busy ())
      {
         if (ow_ds18X20_read_temperature (NULL, &t))
         {
            t = validate_value(t);
            gValues[SENSOR_OUT][TINDEX_NOW] = t;
            minmax_add (&daymin[SENSOR_OUT], t);
            minmax_add (&daymax[SENSOR_OUT], t);
         }
         ow_ds18X20_start (NULL, false);
      }
      break;
   }
   rotate++;

   // see if we have finished an hour, if so then move to a new hour
   if (uptime () >= lasthour + 3600)
   {
      lasthour = uptime ();
      for (i = 0; i < NUMSENSORS; i++)
      {
         minmax_tick (&daymin[i]);
         minmax_tick (&daymax[i]);
      }

   }

   // update max and min values each time round
   for (i = 0; i < NUMSENSORS; i++)
   {
      gValues[i][TINDEX_MIN] = minmax_get (&daymin[i]);
      gValues[i][TINDEX_MAX] = minmax_get (&daymax[i]);
   }

}
