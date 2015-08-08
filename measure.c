
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

#include "features.h"




//MINMAX hourmax[NUMSENSORS];
MINMAX daymax[NUMSENSORS];
//MINMAX hourmin[NUMSENSORS];
MINMAX daymin[NUMSENSORS];


int16_t gValues[NUMSENSORS][NUMINDEX]; // current, max and min temperatures for each sensor
int16_t gLimits[NUMSENSORS][NUMLIMIT]; // upper and lower limits for driving window motors
int16_t gBattery;
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
//      minmax_init (&hourmax[i], 60, true);
//      minmax_init (&hourmin[i], 60, false);
      minmax_init (&daymin[i], 24, false);
      minmax_init (&daymax[i], 24, true);
      for (j = 0; j < NUMINDEX; j++)
         gValues[i][j] = 0;
   }

   // start off temperature conversion on all sensors
   ow_set_bus (&PIND, &PORTD, &DDRD, PD4);          // SENSOR_LOW
   ow_ds18x20_resolution(NULL, 10);
   ow_ds18X20_start (NULL, false);

   ow_set_bus (&PIND, &PORTD, &DDRD, PD5);          // SENSOR_HIGH
   ow_ds18x20_resolution(NULL, 10);
   ow_ds18X20_start (NULL, false);

   ow_set_bus (&PINB, &PORTD, &DDRD, PD6);          // SENSOR_OUT
   ow_ds18x20_resolution(NULL, 10);
   ow_ds18X20_start (NULL, false);
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
   if (value < -5000)
      return -5000;
   if (value > 9000)
      return 9000;
   return value;
}


// poll round our sensors in turn, if conversion finished then note the value and start a new conversion
void
run_measure (void)
{
   static char rotate = 0;
   int16_t t;
   int8_t i;

   gBattery = analog_read (6);

   // do one sensor each time round
   switch (rotate & 3)
   {
   case SENSOR_LOW:
      // low level sensor
      ow_set_bus (&PIND, &PORTD, &DDRD, PD4);          // SENSOR_LOW
      if (!ow_busy ())
      {
         ow_ds18X20_read_temperature (NULL, &t);
         t = validate_value(t);
         gValues[SENSOR_LOW][TINDEX_NOW] = t;
         minmax_add (&daymin[SENSOR_LOW], t);
         minmax_add (&daymax[SENSOR_LOW], t);
         ow_ds18X20_start (NULL, false);
      }
      break;
   case SENSOR_HIGH:
      // high level (roof) sensor
      ow_set_bus (&PIND, &PORTD, &DDRD, PD5);         // SENSOR_HIGH
      if (!ow_busy ())
      {
         ow_ds18X20_read_temperature (NULL, &t);
         t = validate_value(t);
         gValues[SENSOR_HIGH][TINDEX_NOW] = t;
         minmax_add (&daymin[SENSOR_HIGH], t);
         minmax_add (&daymax[SENSOR_HIGH], t);
         ow_ds18X20_start (NULL, false);
      }
      break;
   case SENSOR_OUT:
      // outside sensor
      ow_set_bus (&PIND, &PORTD, &DDRD, PD6);          // SENSOR_OUT
      if (!ow_busy ())
      {
         ow_ds18X20_read_temperature (NULL, &t);
         t = validate_value(t);
         gValues[SENSOR_OUT][TINDEX_NOW] = t;
         minmax_add (&daymin[SENSOR_OUT], t);
         minmax_add (&daymax[SENSOR_OUT], t);
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
