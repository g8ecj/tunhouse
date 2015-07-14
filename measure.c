
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

extern Serial serial;



//MINMAX hourmax[NUMSENSORS];
MINMAX daymax[NUMSENSORS];
//MINMAX hourmin[NUMSENSORS];
MINMAX daymin[NUMSENSORS];


int16_t gValues[NUMSENSORS][NUMINDEX]; // current, max and min temperatures for each sensor
int16_t gLimits[NUMSENSORS][NUMLIMIT]; // upper and lower limits for driving window motors
int16_t gBattery;
uint8_t gpioid = 0;
uint8_t gthermid = 0;
uint8_t ids[6][OW_ROMCODE_SIZE]; // only expect to find up to 5 actually!!
uint8_t idmap[6];


// do a bit of init for testing
void
measure_init (void)
{
   uint8_t i, diff;

   for (diff = OW_SEARCH_FIRST, i = 0; diff != OW_LAST_DEVICE; i++)
   {
      diff = ow_rom_search (diff, ids[i]);

      if ((diff == OW_PRESENCE_ERR) || (diff == OW_DATA_ERR))
         break;                 // <--- early exit!

#if DEBUG > 0
      kfile_printf (&serial.fd,
                    "Found device %02x:%02x%02x%02x%02x%02x%02x:%02x\r\n",
                    ids[i][0], ids[i][1], ids[i][2], ids[i][3], ids[i][4],
                    ids[i][5], ids[i][6], ids[i][7]);
      if (crc8 (ids[i], 8))
         kfile_print (&serial.fd, "CRC suspect\r\n");
#endif
      if (ids[i][0] == SSWITCH_FAM)
         gpioid++;
      if ((ids[i][0] == DS18S20_FAMILY_CODE)
          || (ids[i][0] == DS18B20_FAMILY_CODE)
          || (ids[i][0] == DS1822_FAMILY_CODE))
         gthermid++;

   }

   // initialise all the min/max buffers (hourly and daily)
   for (i = 0; i < NUMSENSORS; i++)
   {
//      minmax_init (&hourmax[i], 60, true);
//      minmax_init (&hourmin[i], 60, false);
      minmax_init (&daymax[i], 24, true);
      minmax_init (&daymin[i], 24, false);
   }

   // start off temperature conversion on all sensors
   ow_ds18X20_start (ids[idmap[SENSOR_LOW]], false);

   ow_ds18X20_start (ids[idmap[SENSOR_HIGH]], false);

   ow_ds18X20_start (ids[idmap[SENSOR_OUT]], false);
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



// interpret a command from the console interface
// empty string outputs the current data
static void
process_command (char *command, uint8_t count)
{
    uint8_t i;

    if (count == 0)             // display now (empty input!!)
    {
        return;
    }
    if (strncmp (command, "config", 6) == 0)
    {
        // skip command string
        command += 6;
        // skip whitespace
        while (*command == ' ')
            command++;
        if (command[0] == '\0')
        {
            for (i = 0; i < 5; i++)
               kfile_putc(idmap[i] | 0x30, &serial.fd);
        }
        else
        {
            for (i = 0; i < 5; i++)
               idmap[i] = command[i] & 0x0f;
            save_eeprom_values ();
        }
    }
    else
    {
        kfile_printf (&serial.fd,
                      "?huh?\r\nCommands are: config\r\n");
    }

}




// get a string from the user with backspace 
static void
get_input (void)
{
    int16_t c;
    static uint8_t bcnt = 0;
#define CBSIZE 20
    static char cbuff[CBSIZE];  /* serial I/O buffer       */

    while ((c = kfile_getc (&serial.fd)) != EOF)
    {
        c &= 0x7f;
        switch ((char) c)
        {
        case 0x03:             // ctl-c
            bcnt = 0;
        case '\r':
// process what is in the buffer
            cbuff[bcnt] = '\0'; // don't include terminator in count
            kfile_printf (&serial.fd, "\r\n");
            process_command (cbuff, bcnt);
            bcnt = 0;
            break;
        case 0x08:             // backspace
        case 0x7f:             // rubout
            if (bcnt > 0)
            {
                kfile_putc (0x08, &serial.fd);
                kfile_putc (' ', &serial.fd);
                kfile_putc (0x08, &serial.fd);
                bcnt--;
            }
            break;
        default:
            if ((c >= ' ') && (bcnt < CBSIZE))
            {
                cbuff[bcnt++] = c;
                kfile_putc (c, &serial.fd); // echo...
            }
            break;
        }

    }
}








// poll round our sensors in turn, if conversion finished then note the value and start a new conversion
void
run_measure (void)
{
   static char rotate = 0;
   int16_t t;
   int8_t i;
   static int8_t firstrun = true;
   static uint32_t lasthour = 0;

   if (firstrun)
   {
      firstrun = false;
      lasthour = uptime ();
   }

   gBattery = analog_read (6);
   get_input ();

   // do one sensor each time round
   switch (rotate & 3)
   {
   case SENSOR_LOW:
      // low level sensor
      if (!ow_busy ())
      {
         ow_ds18X20_read_temperature (ids[idmap[SENSOR_LOW]], &t);
         gValues[SENSOR_LOW][TINDEX_NOW] = t;
         minmax_add (&daymax[SENSOR_LOW], t);
         ow_ds18X20_start (ids[idmap[SENSOR_LOW]], false);
      }
      break;
   case SENSOR_HIGH:
      // high level (roof) sensor
      if (!ow_busy ())
      {
         ow_ds18X20_read_temperature (ids[idmap[SENSOR_HIGH]], &t);
         gValues[SENSOR_HIGH][TINDEX_NOW] = t;
         minmax_add (&daymax[SENSOR_HIGH], t);
         ow_ds18X20_start (ids[idmap[SENSOR_HIGH]], false);
      }
      break;
   case SENSOR_OUT:
      // outside sensor
      if (!ow_busy ())
      {
         ow_ds18X20_read_temperature (ids[idmap[SENSOR_OUT]], &t);
         gValues[SENSOR_OUT][TINDEX_NOW] = t;
         minmax_add (&daymax[SENSOR_OUT], t);
         ow_ds18X20_start (ids[idmap[SENSOR_OUT]], false);
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
         minmax_tick (&daymax[i]);
         minmax_tick (&daymin[i]);
      }

   }

   // update max and min values each time round
   for (i = 0; i < NUMSENSORS; i++)
   {
      gValues[i][TINDEX_MIN] = minmax_get (&daymin[i]);
      gValues[i][TINDEX_MAX] = minmax_get (&daymax[i]);
   }

}
