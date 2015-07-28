
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
// open/close timer rather than wait for a contact closure
uint32_t gWinTimer[2];

static void do_motorup (uint8_t sensor);
static void do_motordn (uint8_t sensor);
static void do_motoroff (uint8_t sensor);
static void do_motorcan (uint8_t sensor);
static void winmachine (uint8_t sensor, uint8_t event);



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
   {{MANOPENING, NULL},        {MANOPENING, NULL},       {MANOPENING, NULL},        {MANCLOSING, do_motordn},   {WINOPEN, do_motorcan},     {MANOPEN, do_motoroff}},
// MANCLOSING
   {{MANCLOSING, NULL},        {MANCLOSING, NULL},       {MANOPENING, do_motorup},  {MANCLOSING, NULL},         {WINCLOSED, do_motorcan},   {MANCLOSED, do_motoroff}},
// MANOPEN
   {{MANOPEN, NULL},           {MANOPEN, NULL},          {MANOPEN, NULL},           {MANCLOSING, do_motordn},   {WINOPEN, do_motorcan},     {WINOPEN, NULL}},
// MANCLOSED
   {{MANCLOSED, NULL},         {MANCLOSED, NULL},        {MANOPENING, do_motorup},  {MANCLOSED, NULL},          {WINCLOSED, do_motorcan},   {WINCLOSED, NULL}},
// WINOPENING
   {{WINOPENING, NULL},        {WINCLOSING, do_motordn}, {WINOPENING, NULL},        {MANCLOSING, do_motordn},   {WINOPEN, do_motorcan},     {WINOPEN, do_motoroff}},
// WINCLOSING
   {{WINOPENING, do_motorup},  {WINCLOSING, NULL},       {MANOPENING, do_motorup},  {WINCLOSING, NULL},         {WINCLOSED, do_motorcan},   {WINCLOSED, do_motoroff}},
// WINOPEN
   {{WINOPEN, NULL},           {WINCLOSING, do_motordn}, {WINOPEN, NULL},           {MANCLOSING, do_motordn},   {WINOPEN, do_motorcan},     {WINOPEN, NULL}},
// WINCLOSED 
   {{WINOPENING, do_motorup},  {WINCLOSED, NULL},        {MANOPENING, do_motorup},  {WINCLOSED, NULL},          {WINCLOSED, do_motorcan},   {WINCLOSED, NULL}},
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



// set the motor 1-wire bus based on the sensor
static void
setonewire (uint8_t sensor)
{

   if (sensor == SENSOR_LOW)
      ow_set_bus (&PIND, &PORTD, &DDRD, PD2);
   else
      ow_set_bus (&PIND, &PORTD, &DDRD, PD3);

}


// start motor unspooling to open a window
static void
do_motorup (uint8_t sensor)
{

    // start timer if motor started
    gWinTimer[sensor] = uptime() + RUNVALUE;

    // set direction relay for upwards motion (port A)
    // turn on power to this motor   (port B)
    setonewire (sensor);
    ow_ds2413_write(NULL, 0x00);

}


// start motor spooling to close a window
static void
do_motordn (uint8_t sensor)
{
    // start timer if motor started
    gWinTimer[sensor] = uptime() + RUNVALUE;
    // direction relay defaults to down so ensure its off (port A)
    // turn on power to this motor (port B)
    setonewire (sensor);
    ow_ds2413_write(NULL, 0x01);

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
    setonewire (sensor);
    ow_ds2413_write(NULL, 0x03);

}

// stop motor to cancel a movement
static void
do_motorcan (uint8_t sensor)
{
    // start display timer as we stop the motor
    gWinTimer[sensor] = uptime() + CANCELVALUE;
    // make sure both relays are de-energized
    // default direction = downwards (relay off)
    // motor off
    setonewire (sensor);
    ow_ds2413_write(NULL, 0x03);

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

        if ((gWinTimer[sensor]) && (uptime() > gWinTimer[sensor]))
        {
        // timers handled here so its all done from the main line, not from an interrupt callback
            gWinTimer[sensor] = 0;
            winmachine (sensor, TIMEOUT);
        }
    }

}
