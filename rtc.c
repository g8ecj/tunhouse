//---------------------------------------------------------------------------
// Copyright (C) 2011 Robin Gilks
//
//
//  rtc.c   -   Real time clock - provides date, time and seconds since 1st Jan 1970
//
//  History:   1.0 - First release. 
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

// include files

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <drv/timer.h>
#include <avr/eeprom.h>

#include "eeprommap.h"
#include "rtc.h"


int16_t gSECOND;
int16_t gMINUTE;
int16_t gHOUR;
int16_t gDAY;
int16_t gMONTH;
int16_t gYEAR;
int16_t gAdjustTime;

static volatile uint32_t Epoch;
static volatile ticks_t LastTicks;
static volatile uint32_t start_of_day;


#define HOUR       0
#define MINUTE     1
#define SECOND     2
#define NOFLASH    3

#define YEAR        0
#define MONTH       1
#define DAY         2


// Lookup table holding the length of each mont. The first element is a dummy.
//    this could be placed in progmem too, but the arrays are accessed quite
//    often - so leaving them in RAM is better...
char MonthLength[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };


// time in seconds since midnight, 1st Jan 1970
void
set_epoch_time (void)
{
   uint8_t i;
   uint32_t t;
   DT_t DateTime;

   t = (gYEAR * 365);           // days in year
   for (i = 0; i < gYEAR; i += 4)
      t += 1;                   // leap years (includes 2000, excludes current year)

   if ((gMONTH > 2) && ((gYEAR & 3) == 0))
      t += 1;                   // add leap day for this leap year

   for (i = 0; i < gMONTH; i++)
      t += MonthLength[i];      // days in month (not including this month!!)

   t += gDAY - 1;               // don't include today!!
   t *= 24;                     // days -> hours
   t += gHOUR;
   t *= 60;
   t += gMINUTE;
   t *= 60;
   t += gSECOND;
   t += 946638000;              // correction to base to 1970 (Unix time)

// use the difference between the new value and the old to adjust start of day
   start_of_day -= Epoch - t;
   Epoch = t;

   DateTime.S = gSECOND;
   DateTime.M = gMINUTE;
   DateTime.H = gHOUR;
   DateTime.d = gDAY;
   DateTime.m = gMONTH;
   DateTime.y = gYEAR;
   eeprom_write_block ((const void *) &DateTime, (void *) &eeDateTime, sizeof (DateTime));

}

void
get_datetime (uint16_t * year, uint8_t * month, uint8_t * day, uint8_t * hour, uint8_t * min, uint8_t * sec)
{
   *sec = gSECOND;
   *min = gMINUTE;
   *hour = gHOUR;
   *day = gDAY;
   *month = gMONTH;
   *year = gYEAR + 2000;

}

uint32_t
uptime (void)
{
   return Epoch - start_of_day;
}

/******************************************************************************
*
*   Function name:  RTC_init
*
*   returns:        none
*
*   parameters:     none
*
*   Purpose:        Start Timer/Counter2 in asynchronous operation using a
*                   32.768kHz crystal.
*
*******************************************************************************/
void
rtc_init (void)
{
   DT_t DateTime;

   // initial time and date setting
   eeprom_read_block ((void *) &DateTime, (const void *) &eeDateTime, sizeof (DateTime));
   eeprom_read_block ((void *) &gAdjustTime, (const void *) &eeAdjustTime, sizeof (gAdjustTime));
   gSECOND = DateTime.S;
   gMINUTE = DateTime.M;
   gHOUR = DateTime.H;
   gDAY = DateTime.d;
   gMONTH = DateTime.m;
   gYEAR = DateTime.y;

   LastTicks = timer_clock ();
   set_epoch_time ();
   start_of_day = Epoch;
}


uint32_t
time (void)
{
   return Epoch;
}






/******************************************************************************
*
*   Timer/Counter2 Overflow Interrupt Routine
*
*   Purpose: Increment the real-time clock
*            The interrupt occurs once a second (running from the 32kHz crystal)
*
*******************************************************************************/

void
run_rtc (void)
{
   int8_t LeapMonth;
   int32_t diff;
   static int16_t lastHour = 0;

   // find out how far off the exact number of ticks we are
   diff = timer_clock () - LastTicks - ms_to_ticks (1000);
   if (diff < 0)
      return;

   // add in the number of ticks we drifted by
   LastTicks = timer_clock () - diff;

   Epoch++;                     // count seconds since epoch (1st Jan 1970)

   gSECOND++;                   // increment second

   if (gSECOND == 60)
   {
      gSECOND = 0;
      gMINUTE++;

      if (gMINUTE > 59)
      {
         gMINUTE = 0;
         gHOUR++;

         if (gHOUR > 23)
         {
            gHOUR = 0;
            gDAY++;

            // Check for leap year if month == February
            if (gMONTH == 2)
               if (!(gYEAR & 0x0003))   // if (gYEAR%4 == 0)
                  LeapMonth = 1;
               else
                  LeapMonth = 0;
            else
               LeapMonth = 0;

            // Now, we can check for month length
            if (gDAY > (MonthLength[gMONTH] + LeapMonth))
            {
               gDAY = 1;
               gMONTH++;

               if (gMONTH > 12)
               {
                  gMONTH = 1;
                  gYEAR++;
               }
            }
         }
         // save time to eeprom on the hour
         set_epoch_time ();
      }
   }

   // still looking at the top of the hour but everything has been updated by now
   // adjust time for slow/fast crystal every hour when minutes are zero and seconds are half way
   // this allows 30 * 24 seconds adjustment per day (i.e. +/- 720) without messing with minute under/overruns
   // remember at what hour we did any change so we only do it once!
   if ((gMINUTE == 0) && (gSECOND == 30) && (lastHour != gHOUR))
   {
      gSECOND += gAdjustTime / 24;
      lastHour = gHOUR;
      // once a day adjust time for any remaining seconds where hourly changes loose resolution
      if (gHOUR == 0)
      {
         gSECOND += gAdjustTime % 24;
      }
   }
}
