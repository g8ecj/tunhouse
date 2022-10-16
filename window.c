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
 * \brief Window opener state machine
 */

#include <stdint.h>

#include <avr/pgmspace.h>

#include <drv/timer.h>
#include <drv/ow_1wire.h>
#include <drv/ow_ds2413.h>

#include "rtc.h"
#include "window.h"
#include "measure.h"


// state of the windows on the 2 sensors
int16_t gWinState[2];
// whether the window is in auto or manual mode
int16_t gWinAuto[2];
// motor run time
int16_t gMotorRun;
// open/close timer rather than wait for a contact closure
uint32_t gWinTimer[2];

static void do_motorup (uint8_t sensor);
static void do_motordn (uint8_t sensor);
static void do_motoroff (uint8_t sensor);
static void do_motorcan (uint8_t sensor);
static void winmachine (uint8_t sensor, uint8_t event);


/*
 Pins used to drive the relays
PD2 D2    FET driver          } LO motor UP
PD3 D3    FET driver          } HI motor UP
PD7 D7    FET driver          } LO motor DOWN
PB0 D8    FET driver          } HI motor DOWN
*/

#define LO_UP(x)      { if (x) PORTD |= BV(2); else PORTD &=~BV(2); } while(0)
#define HI_UP(x)      { if (x) PORTD |= BV(3); else PORTD &=~BV(3); } while(0)
#define LO_DN(x)      { if (x) PORTD |= BV(7); else PORTD &=~BV(7); } while(0)
#define HI_DN(x)      { if (x) PORTB |= BV(0); else PORTB &=~BV(0); } while(0)



typedef struct PROGMEM
{
    uint8_t nextstate;
    void (*pFunc) (uint8_t);
} WINDOW_NEXTSTATE;

#ifndef NULL
#define NULL ((void *)0)
#endif

         /* *INDENT-OFF* */
const WINDOW_NEXTSTATE window_nextstate[][6] PROGMEM = {
// events  TEMPGREATER              TEMPLESSER                  MANUALOPEN             MANUALCLOSE                MANUALCANCEL                 TIMEOUT
// states
// MANOPENING
   {{MANOPENING, NULL},        {MANOPENING, NULL},       {MANOPENING, NULL},        {MANCLOSING, do_motordn},   {MANOPEN, do_motorcan},     {MANOPEN, do_motoroff}},
// MANCLOSING
   {{MANCLOSING, NULL},        {MANCLOSING, NULL},       {MANOPENING, do_motorup},  {MANCLOSING, NULL},         {MANCLOSED, do_motorcan},   {MANCLOSED, do_motoroff}},
// MANOPEN
   {{MANOPEN, NULL},           {MANOPEN, NULL},          {MANOPEN, NULL},           {MANCLOSING, do_motordn},   {MANOPEN, NULL},            {WINOPEN, NULL}},
// MANCLOSED
   {{MANCLOSED, NULL},         {MANCLOSED, NULL},        {MANOPENING, do_motorup},  {MANCLOSED, NULL},          {MANCLOSED, NULL},          {WINCLOSED, NULL}},
// WINOPENING
   {{WINOPENING, NULL},        {WINCLOSING, do_motordn}, {WINOPENING, NULL},        {MANCLOSING, do_motordn},   {WINOPENING, NULL},         {WINOPEN, do_motoroff}},
// WINCLOSING
   {{WINOPENING, do_motorup},  {WINCLOSING, NULL},       {MANOPENING, do_motorup},  {WINCLOSING, NULL},         {WINCLOSING, NULL},         {WINCLOSED, do_motoroff}},
// WINOPEN
   {{WINOPEN, NULL},           {WINCLOSING, do_motordn}, {WINOPEN, NULL},           {MANCLOSING, do_motordn},   {WINOPEN, NULL},            {WINOPEN, NULL}},
// WINCLOSED 
   {{WINOPENING, do_motorup},  {WINCLOSED, NULL},        {MANOPENING, do_motorup},  {WINCLOSED, NULL},          {WINCLOSED, NULL},          {WINCLOSED, NULL}},
};
         /* *INDENT-ON* */


// start with windows closed
void
window_init (void)
{
   gWinState[SENSOR_LOW] = WINCLOSED;
   gWinState[SENSOR_HIGH] = WINCLOSED;
   gWinTimer[SENSOR_LOW] = 0;
   gWinTimer[SENSOR_HIGH] = 0;
   DDRD |= BV(2) | BV(3) | BV(7);
   DDRB |= BV(0);
   LO_UP(0);
   LO_DN(0);
   HI_UP(0);
   HI_DN(0);

}



// manually open a window
void
windowopen (int8_t sensor)
{
    winmachine (sensor, MANUALOPEN);
}


// manually close a window
void
windowclose (int8_t sensor)
{
    winmachine (sensor, MANUALCLOSE);
}


// cancel lockout timer
void
windowcan (int8_t sensor)
{
    winmachine (sensor, MANUALCANCEL);
}


// find out if window still opening/closing manually
uint8_t
windowidle(uint8_t sensor)
{
   if (sensor == SENSOR_OUT)
      return true;

   if ((gWinState[sensor] == MANOPENING) || (gWinState[sensor] == MANCLOSING))
      return false;
   else
      return true;
}



// drive round the state machine, moving between states and initiating actions
static void
winmachine (uint8_t sensor, uint8_t event)
{
    void (*pStateFunc) (uint8_t);
    uint8_t state, nextstate;

    state = gWinState[sensor];
    nextstate = pgm_read_byte (&window_nextstate[state][event].nextstate);
    pStateFunc = (PGM_VOID_P) pgm_read_word (&window_nextstate[state][event].pFunc);
    if (pStateFunc)
    {
        pStateFunc (sensor);
    }
    gWinState[sensor] = nextstate;
}


// start motor unspooling to open a window
static void
do_motorup (uint8_t sensor)
{

    // start timer if motor started
    gWinTimer[sensor] = uptime() + gMotorRun;

    // set direction relay for upwards motion (port A)
    // turn on power to this motor   (port B)
   if (sensor == SENSOR_LOW)
   {
      LO_UP(1);
      LO_DN(0);
   }
   else
   {
      HI_UP(1);
      HI_DN(0);
   }
}


// start motor spooling to close a window
static void
do_motordn (uint8_t sensor)
{
    // start timer if motor started
    gWinTimer[sensor] = uptime() + gMotorRun;
    // direction relay defaults to down so ensure its off (port A)
    // turn on power to this motor (port B)
   if (sensor == SENSOR_LOW)
   {
      LO_UP(0);
      LO_DN(1);
   }
   else
   {
      HI_UP(0);
      HI_DN(1);
   }

}


// stop motor
static void
do_motoroff (uint8_t sensor)
{
    // start lockout timer if motor stopped
    gWinTimer[sensor] = uptime() + LOCKOUTVALUE;
    // make sure both relays are de-energized
    // default direction = downwards (relay off)
    // motor off
   if (sensor == SENSOR_LOW)
   {
      LO_UP(0);
      LO_DN(0);
   }
   else
   {
      HI_UP(0);
      HI_DN(0);
   }

}

// stop motor to cancel a movement
static void
do_motorcan (uint8_t sensor)
{
    // set timer so we have no more movements for LOCKOUT seconds
    gWinTimer[sensor] = uptime() + LOCKOUTVALUE;
    // make sure both relays are de-energized
    // default direction = downwards (relay off)
    // motor off
   if (sensor == SENSOR_LOW)
   {
      LO_UP(0);
      LO_DN(0);
   }
   else
   {
      HI_UP(0);
      HI_DN(0);
   }

}


// called from main on a regular basis to run state machine
void
run_windows (void)
{
   uint8_t sensor;
   int16_t now, up, down;

   // for each sensor
   for (sensor = SENSOR_LOW; sensor <= SENSOR_HIGH; sensor++)
   {
      if (gWinState[sensor] >= WINOPENING)
         gWinAuto[sensor] = 2;
      else
         gWinAuto[sensor] = 3;

      getlims (sensor, &now, &up, &down);
      if (now >= up)
         winmachine (sensor, TEMPGREATER);
      else if (now <= down)
         winmachine (sensor, TEMPLESSER);

#if 1
      // treat exceeding stall current as timeout - stop motor!
      if (gCurrent[sensor] > (gStall[sensor] * 10))
      {
         gWinTimer[sensor] = 0;
         winmachine (sensor, TIMEOUT);
      }
#endif
      if ((gWinTimer[sensor]) && (uptime () > gWinTimer[sensor]))
      {
         // timers handled here so its all done from the main line, not from an interrupt callback
         gWinTimer[sensor] = 0;
         winmachine (sensor, TIMEOUT);
      }
   }
}
