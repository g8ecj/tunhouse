
#include <stdint.h>

#include <avr/pgmspace.h>

#include <drv/timer.h>
#include <drv/ow_1wire.h>
#include <drv/ow_ds2413.h>

#include "window.h"
#include "measure.h"


extern uint8_t ids[6][OW_ROMCODE_SIZE];

// state of the windows on the 2 sensors
int16_t gWinState[2];
// open/close timer rather than wait for a contact closure
ticks_t gWinTimer[2];


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
// WINCLOSED 
   {{WINOPENING, do_motorup},  {WINCLOSED, NULL},        {MANOPENING, do_motorup},  {WINCLOSED, NULL},          {WINCLOSED, do_motorcan},   {WINCLOSED, NULL}},
// WINOPENING
   {{WINOPENING, NULL},        {WINCLOSING, do_motordn}, {WINOPENING, NULL},        {MANCLOSING, do_motordn},   {WINOPEN, do_motorcan},     {WINOPEN, do_motoroff}},
// WINOPEN
   {{WINOPEN, NULL},           {WINCLOSING, do_motordn}, {WINOPEN, NULL},           {MANCLOSING, do_motordn},   {WINOPEN, do_motorcan},     {WINOPEN, NULL}},
// WINCLOSING
   {{WINOPENING, do_motorup},  {WINCLOSING, NULL},       {MANOPENING, do_motorup},  {WINCLOSING, NULL},         {WINCLOSED, do_motorcan},   {WINCLOSED, do_motoroff}},
// MANOPENING
   {{MANOPENING, NULL},        {MANOPENING, NULL},       {MANOPENING, NULL},        {MANCLOSING, do_motordn},   {WINOPEN, do_motorcan},     {MANOPEN, do_motoroff}},
// MANCLOSING
   {{MANCLOSING, NULL},        {MANCLOSING, NULL},       {MANOPENING, do_motorup},  {MANCLOSING, NULL},         {WINCLOSED, do_motorcan},   {MANCLOSED, do_motoroff}},
// MANOPEN
   {{MANOPEN, NULL},           {MANOPEN, NULL},          {MANOPEN, NULL},           {MANCLOSING, do_motordn},   {WINOPEN, do_motorcan},     {WINOPEN, NULL}},
// MANCLOSED
   {{MANCLOSED, NULL},         {MANCLOSED, NULL},        {MANOPENING, do_motorup},  {MANCLOSED, NULL},          {WINCLOSED, do_motorcan},   {WINCLOSED, NULL}},
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


// return status of opening completed yet
uint8_t
windowopening (uint8_t sensor)
{

    return (gWinState[sensor] != MANOPENING);

}

// return status of closing completed yet
uint8_t
windowclosing (uint8_t sensor)
{

    return (gWinState[sensor] != MANCLOSING);

}

// return status of canceling lockout timer
uint8_t
windowcanceling (uint8_t sensor)
{

//   return (gWinState[sensor] != MANCLOSING);
    return (gWinTimer[sensor] == 0);

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


// start motor unspooling to open a window
void
do_motorup (uint8_t sensor)
{

    // start timer if motor started
    gWinTimer[sensor] = RUNVALUE;

    // set direction relay for upwards motion (port A)
    // turn on power to this motor   (port B)
    ow_ds2413_write(ids[idmap[sensor+NUMSENSORS]], 0x00);

}


// start motor spooling to close a window
void
do_motordn (uint8_t sensor)
{
    // start timer if motor started
    gWinTimer[sensor] = RUNVALUE;
    // direction relay defaults to down so ensure its off (port A)
    // turn on power to this motor (port B)
    ow_ds2413_write(ids[idmap[sensor+NUMSENSORS]], 0x01);

}


// stop motor
void
do_motoroff (uint8_t sensor)
{
    // start lockout timer if motor stopped
    gWinTimer[sensor] = LOCKOUTVALUE;
    // make sure both relays are de-energized
    // default direction = downwards (relay off)
    // motor off
    ow_ds2413_write(ids[idmap[sensor+NUMSENSORS]], 0x03);

}

// stop motor to cancel a movement
void
do_motorcan (uint8_t sensor)
{
    // start display timer as we stop the motor
    gWinTimer[sensor] = CANCELVALUE;
    // make sure both relays are de-energized
    // default direction = downwards (relay off)
    // motor off
    ow_ds2413_write(ids[idmap[sensor+NUMSENSORS]], 0x03);

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
        getlims (sensor, &now, &up, &down);
        if (now >= up)
            winmachine (sensor, TEMPGREATER);
        else if (now <= down)
            winmachine (sensor, TEMPLESSER);

        if ((gWinTimer[sensor]) && (timer_clock () - gWinTimer[sensor] > ms_to_ticks (1000)))
        {
        // timers handled here so its all done from the main line, not from an interrupt callback
            gWinTimer[sensor] = 0;
            winmachine (sensor, TIMEOUT);
        }
    }

}
