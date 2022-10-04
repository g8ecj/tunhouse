//---------------------------------------------------------------------------
// Copyright (C) 2012 Robin Gilks
//
//
//  ui.c   -   User interface - drives the LCD and scans the keyboard for user input
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


#include <cfg/debug.h>

#include <cpu/irq.h>
#include <cpu/power.h>
#include <cpu/pgm.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include <stdlib.h>

#include <algo/crc8.h>

#include <drv/timer.h>
#include <drv/ser.h>
#include <drv/lcd_hd44.h>
#include <drv/term.h>

#include <drv/kbd.h>

#include <avr/eeprom.h>

#include "measure.h"
#include "rtc.h"
#include "eeprommap.h"
#include "window.h"
#include "ui.h"


// a table of fields that are flashing
#define MAXFLASH    10
static int8_t flashing[MAXFLASH];

// mode values
#define MONITOR     0
#define SETUP       1
#define PAGEEDIT    2
#define FIELDEDIT   3
#define MANUAL      4

// turn on backlight is battery better than 12.6 volts
#define BATTERY_OK 1260

static int8_t mode = MONITOR;
static bool refreshed = false;
int16_t gBacklight;
ticks_t backlight_timer = 0;

// Timings (in mS) for various activities
#define REFRESH 300L
#define FLASHON 600L
#define FLASHOFF 300L



extern Serial serial;
static Term term;

static const char lcd_degree[8] = { 0x1c, 0x14, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00 };   /* degree - char set B doesn't have it!! */

#define DEGREE 1

// prototype functions that may not be used
int8_t get_line (int8_t field, int8_t screen);
int8_t find_next_line (int8_t field, int8_t screen, int8_t dirn);


typedef int8_t (*IncFunc_t) (int8_t field, int8_t dirn);


typedef struct PROGMEM
{
   int16_t *value;
   int16_t min;
   int16_t max;
   int16_t defval;
   uint8_t style;
   IncFunc_t get_inc;           // pointer to field increment function for this field
} Vars;

typedef struct PROGMEM
{
   int8_t field;                // global field number (relevant across all screens).
                                // -1 = no value (text only)
                                // -2 = end of array of structs
   int8_t row;                  // row of where to start text
   int8_t col;                  // column of where to start text
   PGM_P text;                  // the text!!
   int8_t vcol;                 // the column of where to display the value
   int8_t width;                // width of the field
} Screen;


// forward reference
const Vars variables[eNUMVARS];

static int8_t
null_inc (int8_t field, int8_t dirn)
{
   (void) field;
   (void) dirn;
   return 0;
}

static int8_t
heca_inc (int8_t field, int8_t dirn)
{
   (void) dirn;
   (void) field;
   return 100;
}

static int8_t
deca_inc (int8_t field, int8_t dirn)
{
   (void) dirn;
   (void) field;
   return 10;
}

#if 0       // not used in this app
static int8_t
var_inc (int8_t field, int8_t dirn)
{
   (void) dirn;
   if (*variables[field].value > 999)
      return 20;
   if (*variables[field].value > 99)
      return 5;

   return 1;
}
#endif

static int8_t
int_inc (int8_t field, int8_t dirn)
{
   (void) dirn;
   (void) field;
   return 1;
}



// the way I'm going to display the int16_t value - with decimal places, fixed width etc
enum STYLE
{
   eNORMAL,
   eDATE,
   eLARGE,
   eDECIMAL,
   eSHORT,
   eBOOLEAN,
   eTRILEAN,
   eWINDOW
};


//           value,                        min,   max, default, style,   increment function
// Note: Its only worth having (real) limits for those values that can be changed.
// *INDENT-OFF*
const Vars variables[eNUMVARS] PROGMEM = {
   {NULL, 0, 0, 0, eNORMAL, null_inc},  // dummy 1st entry
   {&gValues[SENSOR_LOW][TINDEX_MIN],     0,     0,     0,     eSHORT,   null_inc},    // low position, minimum
   {&gValues[SENSOR_LOW][TINDEX_NOW],     0,     0,     0,     eSHORT,   null_inc},    //                value now
   {&gValues[SENSOR_LOW][TINDEX_MAX],     0,     0,     0,     eSHORT,   null_inc},    //                maximuum

   {&gValues[SENSOR_HIGH][TINDEX_MIN],    0,     0,     0,     eSHORT,   null_inc},    // high position, minimum
   {&gValues[SENSOR_HIGH][TINDEX_NOW],    0,     0,     0,     eSHORT,   null_inc},    //                value now
   {&gValues[SENSOR_HIGH][TINDEX_MAX],    0,     0,     0,     eSHORT,   null_inc},    //                maximuum

   {&gValues[SENSOR_OUT][TINDEX_MIN],     0,     0,     0,     eSHORT,   null_inc},    // outside position, minimum
   {&gValues[SENSOR_OUT][TINDEX_NOW],     0,     0,     0,     eSHORT,   null_inc},    //                value now
   {&gValues[SENSOR_OUT][TINDEX_MAX],     0,     0,     0,     eSHORT,   null_inc},    //                maximuum

   {&gLimits[SENSOR_LOW][LIMIT_UP],   -2000,  3000,  2000,       eSHORT,  heca_inc},     // temperature to open
   {&gLimits[SENSOR_LOW][LIMIT_DN],   -2000,  3000,  1500,       eSHORT,  heca_inc},     //                close

   {&gLimits[SENSOR_HIGH][LIMIT_UP],  -2000,  3000,  2000,       eSHORT,  heca_inc},     // temperature to open
   {&gLimits[SENSOR_HIGH][LIMIT_DN],  -2000,  3000,  1500,       eSHORT,  heca_inc},     //                close

   {&gRadio,                              0,     1,     0,      eBOOLEAN,  int_inc},     // turn NRF radio on/off
   {&gBacklight,                          0,    60,    15,      eNORMAL,   int_inc},     // backlight timer adjuster
   {&gAdjustTime,                      -719,   719,     0,      eNORMAL,   int_inc},     // clock adjuster

   {&gHOUR,                               0,    23,    12,        eDATE,   int_inc},     // hour
   {&gMINUTE,                             0,    59,     0,        eDATE,   int_inc},     // minute
   {&gSECOND,                             0,    59,     0,        eDATE,   int_inc},     // second
   {&gDAY,                                1,    31,    15,        eDATE,   int_inc},     // day
   {&gMONTH,                              1,    12,     7,        eDATE,   int_inc},     // month
   {&gYEAR,                              12,    99,    20,        eDATE,   int_inc},     // year
 
   {&gBatCal,                         -2000, 2000,     0,        eSHORT,  deca_inc},     // battery calibration +/- 20% to 0.1%
   {&gStall[SENSOR_LOW],                  0,  500,   100,        eSHORT,  deca_inc},     // motor stall cutout current
   {&gStall[SENSOR_HIGH],                 0,  500,   100,        eSHORT,  deca_inc},     // motor stall cutout current
   {&gMotorRun,                           0,  600,    60,       eNORMAL,  deca_inc},     // motor run time

   {&gBattery,                            0,     0,     0,     eDECIMAL,  null_inc},     // battery volts

   {&gWinAuto[SENSOR_LOW],                0,     0,     0,     eTRILEAN,  null_inc},     //manual/auto
   {&gWinAuto[SENSOR_HIGH],               0,     0,     0,     eTRILEAN,  null_inc},     //manual/auto
   {&gWinState[SENSOR_LOW],               0,     0,     0,      eWINDOW,  null_inc},     //open/close etc
   {&gWinState[SENSOR_HIGH],              0,     0,     0,      eWINDOW,  null_inc},     //open/close etc
};


// strings to  go into progmem

const char datlim[]   PROGMEM  = "  -";
const char timlim[]   PROGMEM  = "  :";
const char dash[]     PROGMEM  = "-";
const char nulstr[]   PROGMEM  = "";
const char atstr[]    PROGMEM  = "@";
const char ampsstr[]  PROGMEM  = "A";
const char secsstr[]  PROGMEM  = "s";
const char percentstr[] PROGMEM  = "%";
const char radiostr[] PROGMEM  = "Radio";
const char blitestr[]  PROGMEM  = "Backlight";
const char adjuststr[] PROGMEM  = "Timesync";
const char battstr[]  PROGMEM  = "Battery";
const char calstr[]   PROGMEM  = "Batt Cal";
const char dnstallstr[] PROGMEM  = "Lower max I";
const char upstallstr[] PROGMEM  = "Upper max I";
const char motorrunstr[] PROGMEM  = "Motor Run";
const char closestr[] PROGMEM  = "Close";
const char datestr[]  PROGMEM  = "Date";
const char exstr[]    PROGMEM  = "Ex   ";
const char extstr[]   PROGMEM  = "External";
const char upstr[]    PROGMEM  = "Up   ";
const char limstr[]   PROGMEM  = "Limits";
const char lostr[]    PROGMEM  = "Lo   ";
const char lowstr[]   PROGMEM  = "Lower";
const char manstr[]   PROGMEM  = "Manual";
const char maxstr[]   PROGMEM  = "Max";
const char minstr[]   PROGMEM  = "Min   ";
const char nowstr[]   PROGMEM  = "Now";
const char openstr[]  PROGMEM  = "Open ";
const char timestr[]  PROGMEM  = "Time";
const char uppstr[]   PROGMEM  = "Upper";
const char voltstr[]  PROGMEM  = "Volts";
const char degreestr[] PROGMEM = { DEGREE, 'C', 0 };



const Screen summary[] PROGMEM = {
   {eDN_MIN,    1,    0,     lostr,     3,    5},
   {eDN_NOW,    1,    0,     nulstr,    9,    5},
   {eDN_MAX,    1,    0,     nulstr,   15,    5},

   {eUP_MIN,    0,    0,     upstr,     3,    5},
   {eUP_NOW,    0,    0,     nulstr,    9,    5},
   {eUP_MAX,    0,    0,     nulstr,   15,    5},

   {eOT_MIN,    2,    0,     exstr,     3,    5},
   {eOT_NOW,    2,    0,     nulstr,    9,    5},
   {eOT_MAX,    2,    0,     nulstr,   15,    5},

   {eHOUR,      3,    0,     timlim,    0,    2},
   {eMINUTE,    3,    3,     timlim,    3,    2},
   {eSECOND,    3,    6,     nulstr,    6,    2},
   {eDAY,       3,   11,     datlim,   11,    2},
   {eMONTH,     3,   14,     datlim,   14,    2},
   {eYEAR,      3,   17,     nulstr,   17,    2},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen lower[] PROGMEM = {
   {eWINAUTO_LO, 0,   0,     lowstr,    6,    5},
   {eWINSTATE_LO,0,   0,     nulstr,   13,    7},
   {eDN_MIN,    1,    0,     minstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {eDN_NOW,    2,    0,     nowstr,    6,    5},
   {-1,         2,   13,  degreestr,    0,    0},
   {eDN_MAX,    3,    0,     maxstr,    6,    5},
   {-1,         3,   13,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen upper[] PROGMEM = {
   {eWINAUTO_HI, 0,   0,     uppstr,    6,    5},
   {eWINSTATE_HI,0,   0,     nulstr,   13,    7},
   {eUP_MIN,    1,    0,     minstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {eUP_NOW,    2,    0,     nowstr,    6,    5},
   {-1,         2,   13,  degreestr,    0,    0},
   {eUP_MAX,    3,    0,     maxstr,    6,    5},
   {-1,         3,   13,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen external[] PROGMEM = {
   {-1,         0,    2,     extstr,    0,    0},
   {eOT_MIN,    1,    0,     minstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {eOT_NOW,    2,    0,     nowstr,    6,    5},
   {-1,         2,   13,  degreestr,    0,    0},
   {eOT_MAX,    3,    0,     maxstr,    6,    5},
   {-1,         3,   13,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen datetime[] PROGMEM = {
   {-1,         0,    2,    timestr,    0,    0},
   {eHOUR,      1,    5,     timlim,    5,    2},
   {eMINUTE,    1,    8,     timlim,    8,    2},
   {eSECOND,    1,    0,     nulstr,   11,    2},
   {-1,         2,    2,    datestr,    0,    0},
   {eDAY,       3,    5,     datlim,    5,    2},
   {eMONTH,     3,    8,     datlim,    8,    2},
   {eYEAR,      3,    0,     nulstr,   11,    2},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen battery[] PROGMEM = {
   {-1,         0,    2,    battstr,    0,    0},
   {eBATTERY,   2,   10,    voltstr,    3,    5},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen Set_Lower[] PROGMEM = {
   {-1,         0,    1,     lowstr,    0,    0},
   {-1,         0,   10,     limstr,    0,    0},
   {eDN_LIMIT_LO,1,   2,   closestr,   11,    5},
   {-1,         1,    9,      atstr,    0,    0},
   {-1,         1,   17,  degreestr,    0,    0},
   {eDN_LIMIT_HI,2,   2,    openstr,   11,    5},
   {-1,         2,    9,      atstr,    0,    0},
   {-1,         2,   17,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen Set_Upper[] PROGMEM = {
   {-1,         0,    1,     uppstr,    0,    0},
   {-1,         0,   10,     limstr,    0,    0},
   {eUP_LIMIT_LO,1,   2,   closestr,   11,    5},
   {-1,         1,    9,      atstr,    0,    0},
   {-1,         1,   17,  degreestr,    0,    0},
   {eUP_LIMIT_HI,2,   2,    openstr,   11,    5},
   {-1,         2,    9,      atstr,    0,    0},
   {-1,         2,   17,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen Set_Time[] PROGMEM = {
   {eRADIO,     0,    0,   radiostr,   11,    2},
   {eBACKLIGHT, 1,    0,   blitestr,   11,    4},
   {eADJUSTTIME,2,    0,  adjuststr,   11,    4},
   {eHOUR,      3,    0,     timlim,    0,    2},
   {eMINUTE,    3,    3,     timlim,    3,    2},
   {eSECOND,    3,    6,     nulstr,    6,    2},
   {eDAY,       3,   11,     datlim,   11,    2},
   {eMONTH,     3,   14,     datlim,   14,    2},
   {eYEAR,      3,   17,     nulstr,   17,    2},
   {-2,         0,    0,     nulstr,    0,    0}
};


const Screen Set_Battery[] PROGMEM = {
   {eBATCAL,    0,    0,     calstr,    14,    5},
   {-1,         0,   18, percentstr,     0,    0},
   {eSTALL_DN,  1,    0, dnstallstr,    14,    5},
   {-1,         1,   18,    ampsstr,     0,    0},
   {eSTALL_UP,  2,    0, upstallstr,    14,    5},
   {-1,         2,   18,    ampsstr,     0,    0},
   {eMOTORRUN,  3,    0, motorrunstr,   14,    5},
   {-1,         3,   17,     secsstr,    0,    0},
   {-2,         0,    0,     nulstr,     0,    0}
};


// *INDENT-ON*

#define NUM_INFO    6
#define NUM_SETUP   4

#define FIRSTINFO   0
#define MAXINFO     (NUM_INFO - 1)

#define FIRSTSETUP  NUM_INFO
#define MAXSETUP    (NUM_INFO + NUM_SETUP - 1)


// order here is critical - screen numbers are used to derive sensor numbers in some modes!!
static const Screen *screen_list[] =  { summary, lower, upper, external, datetime, battery, Set_Lower, Set_Upper, Set_Time, Set_Battery };


// add field to list of flashing fields
void
set_flash (int8_t field, int8_t set)
{
   int8_t i;

   for (i = 0; i < MAXFLASH; i++)
   {
      if (set)
      {
         // find a free slot or already set then set it
         if ((flashing[i] == 0) || (flashing[i] == field))
         {
            flashing[i] = field;
            break;
         }
      }
      else
      {
         // find which slot its in and clear it
         if (flashing[i] == field)
         {
            flashing[i] = 0;
            break;
         }
      }
   }
}

// return an indicator on whether this field is flashing and should currently be blanked (true) or displayed (false)
static int8_t
check_flash (int8_t field)
{
   int8_t i;
   static int8_t flash_state = true;
   static ticks_t flash_on_timer, flash_off_timer;

   // always toggle flash_state with the correct cadence then see if we need it!!
   if (flash_state)             // currently on, see if on time has expired
   {
      if (timer_clock () - flash_off_timer > ms_to_ticks (FLASHOFF))
      {
         // timer expired, set off timer and turn off (blank) field
         flash_on_timer = timer_clock ();
         flash_state = false;   // signify its now displayed
      }
   }
   else                         // currently on, turn it off
   {
      if (timer_clock () - flash_on_timer > ms_to_ticks (FLASHON))
      {
         flash_off_timer = timer_clock ();
         flash_state = true;    // signify its now blanked out
      }
   }

   for (i = 0; i < MAXFLASH; i++)
   {
      if (flashing[i] == field)
         return flash_state;
   }
   return false;
}

// display a variable or blanks of the correct length at the coordinates for this field in this screen
static void
print_field (int16_t value, int8_t field, uint8_t screen)
{
   int8_t i;
   int16_t whole, part;
   char spaces[10] = "         ";
   char tritext[4][8] = { "off ", "on  ", " auto", "manual" };
   char wintext[4][8] = { "OPENING", "CLOSING", "OPEN   ", "CLOSED " };

   const Screen *scrn = screen_list[screen];

   for (i = 0; (int8_t) pgm_read_byte (&scrn[i].field) != -2; i++)
   {
      if ((int8_t) pgm_read_byte (&scrn[i].field) == field)     // found the correct one
      {
         // set write position
         kfile_printf (&term.fd, "%c%c%c", TERM_CPC, TERM_ROW + pgm_read_byte (&scrn[i].row),
                       TERM_COL + pgm_read_byte (&scrn[i].vcol));
         // output spaces of field width to clear it in case flashing or changing
         kfile_printf (&term.fd, "%.*s", pgm_read_byte (&scrn[i].width), spaces);

         // if its currently in a blank phase of the flashing then we're done (leave as spaces)
         if (check_flash (field))
            break;

         // reset to write position
         kfile_printf (&term.fd, "%c%c%c", TERM_CPC, TERM_ROW + pgm_read_byte (&scrn[i].row),
                       TERM_COL + pgm_read_byte (&scrn[i].vcol));
         // output value based on type of field
         switch (pgm_read_byte(&variables[field].style))
         {
         case eNORMAL:
            kfile_printf (&term.fd, "%d", value);
            break;
         case eDATE:
            kfile_printf (&term.fd, "%02d", value);
            break;
         case eLARGE:
            kfile_printf (&term.fd, "%u", (uint16_t) value);
            break;
         case eDECIMAL:
            // split the value into those bits before and after the decimal point
            // if the whole part is less than 1 then we loose the sign bit so do it manually in all cases
            whole = abs (value / 100);
            part = abs (value % 100);
            kfile_printf (&term.fd, "%.*s%d.%02u", value < 0 ? 1 : 0, "-", whole, part);
            break;
         case eSHORT:
            // split the value into those bits before and after the decimal point, ONLY 1 PLACE!
            // if the whole part is less than 1 then we loose the sign bit so do it manually in all cases
            whole = abs (value / 100);
            part = abs (value % 100) / 10;
            kfile_printf (&term.fd, "%.*s%d.%1u", value < 0 ? 1 : 0, "-", whole, part);
            break;
         case eBOOLEAN:
            kfile_printf (&term.fd, "%s", tritext[value & 1]);
            break;
         case eTRILEAN:
            kfile_printf (&term.fd, "%s", tritext[value & 3]);
            break;
         case eWINDOW:
            kfile_printf (&term.fd, "%s", wintext[value & 3]);
            break;
         }
         break;
      }
   }
}


// scan screen to determine the min and max field numbers
// direction can be -1 for up, 1 for down or 0 for current field
// min & max handle the wrap round
// return the new field
// assumes fields are contiguous on a screen

static int8_t
find_next_field (int8_t field, int8_t screen, int8_t dirn)
{
   int8_t i, min = 99, max = -1;
   const Screen *scrn = screen_list[screen];

   for (i = 0; (int8_t) pgm_read_byte (&scrn[i].field) != -2; i++)
   {
      if ((int8_t) pgm_read_byte (&scrn[i].field) < 0)
         continue;
      if ((int8_t) pgm_read_byte (&scrn[i].field) > max)
         max = pgm_read_byte (&scrn[i].field);
      if ((int8_t) pgm_read_byte (&scrn[i].field) < min)
         min = pgm_read_byte (&scrn[i].field);
   }

   field += dirn;
   if (field > max)
      field = min;
   if (field < min)
      field = max;

   return field;
}

// find out what line this field is on
int8_t
get_line (int8_t field, int8_t screen)
{
   const Screen *scrn = screen_list[screen];
   int8_t i;

   // get the line this field is on
   for (i = 0; (int8_t) pgm_read_byte (&scrn[i].field) != -2; i++)
   {
      if ((int8_t) pgm_read_byte (&scrn[i].field) < 0)
         continue;
      if ((int8_t) pgm_read_byte (&scrn[i].field) == field)
      {
         return (int8_t) pgm_read_byte (&scrn[i].row);
      }
   }
   // if field not found then return something odd!!
   return -1;
}

// find the next line by scanning fields in the direction requested
int8_t
find_next_line (int8_t field, int8_t screen, int8_t dirn)
{
   int8_t startline, line = 0;
   int8_t startfield = field;

   startline = get_line (field, screen);

   while (1)
   {
      // move fields in the direction specified
      field = find_next_field (field, screen, dirn);
      line = get_line (field, screen);
      // moved to another line
      if (line != startline)
         break;
      // if only 1 line then wrapped back to where we started
      if (field == startfield)
         break;
   }

   return field;
}

// display the text and optional field for all lines on a screen
static void
print_screen (int8_t screen)
{
   int8_t i = 0, j;
   PGM_P text;
   const Screen *scrn = screen_list[screen];
   int16_t *pVar;

   while ((int8_t) pgm_read_byte (&scrn[i].field) != -2)
   {
      kfile_printf (&term.fd, "%c%c%c", TERM_CPC, TERM_ROW + pgm_read_byte (&scrn[i].row),
                    TERM_COL + pgm_read_byte (&scrn[i].col));
      text = (PGM_P) pgm_read_word (&scrn[i].text);

      for (j = 0; (const char) (pgm_read_byte (&text[j])) && j < 20; j++)
      {
         kfile_putc ((const char) pgm_read_byte (&text[j]), &term.fd);
      }

      if ((int8_t) pgm_read_byte (&scrn[i].field) != -1)
      {
         pVar = (int16_t *) pgm_read_word(&variables[pgm_read_byte (&scrn[i].field)].value);
         print_field (*pVar, pgm_read_byte (&scrn[i].field), screen);
      }
      i++;
   }
}




// scan through a few variables and check their limits to see if they should be flashing
static void
flag_warnings (void)
{
#if 0       /* not really sure about this!! */
   if (gValues[SENSOR_HIGH][TINDEX_NOW] > gLimits[SENSOR_HIGH][LIMIT_UP])
      set_flash (eUP_NOW, true);
   else
      set_flash (eUP_NOW, false);

   if (gValues[SENSOR_LOW][TINDEX_NOW] > gLimits[SENSOR_LOW][LIMIT_UP])
      set_flash (eDN_NOW, true);
   else
      set_flash (eDN_NOW, false);
#endif

}



// initialise the module!
void
ui_init (void)
{

   lcd_init ();
   lcd_display (1, 0, 0);
   lcd_remapChar (lcd_degree, DEGREE);  // put the degree symbol on character 0x01

   term_init (&term);
   kbd_init ();

   if (gBacklight == 0)
   {
      lcd_backlight (1);
      backlight_timer = timer_clock ();
   }

}


void
ui_load_defaults(void)
{
   uint8_t field;
   int16_t *pVar;

   for (field = 1; field < eNUMVARS; field++)
   {
      pVar = (int16_t *) pgm_read_word(&variables[field].value);
      *pVar = pgm_read_word(&variables[field].defval);
   }
   save_eeprom_values ();

}


// get a row of text from the terminal emulator, indicating which row it is.
// If we have all the data, return -1. On the next read we will restart at the beginning.
int8_t
ui_termrowget (uint8_t * buffer)
{
   int8_t i;
   static uint8_t row = 0;

   i = kfile_read (&term.fd, buffer, CONFIG_TERM_COLS);

   if (i != CONFIG_TERM_COLS)
   {
      row = 0;
      return -1;
   }
   else
      return row++;

}


// If the cursor is relevant (Page or Field edit modes) return true and the row and column of the curos
// otherwise return false - row and column in this case are invalid.
int8_t
ui_termcursorget (uint8_t * row, uint8_t * column)
{
   int8_t i;

   i = kfile_seek (&term.fd, 0, KSM_SEEK_CUR);

   if ((mode == PAGEEDIT) || (mode == FIELDEDIT))
   {
      *row = i / CONFIG_TERM_COLS;
      *column = i % CONFIG_TERM_COLS;
      return true;
   }
   else
   {
      *row = 0;
      *column = 0;
      return false;
   }

}

// Allow external user find out when the screen has been refreshed
bool
ui_refresh_check(void)
{
   bool ret = refreshed;
   refreshed = false;
   return ret;
}


// Allow external user find out the state of the backlight
bool
ui_backlight_check(void)
{
   return backlight_timer > 0 ? true : false;
}


// handle user interaction and screen refreshes in the UI
void
run_ui (uint8_t remote_key)
{
   static int8_t screen_number = 0, last_screen = 99, field = 0;
   static ticks_t refresh_timer;
   static int16_t saved_value;
   uint8_t sensor;
   int16_t *pVar;
   int16_t inc;
   IncFunc_t pIncFunc;

   // mark those fields that should ne flashed
   flag_warnings ();

   keymask_t key;
   key = kbd_peek ();

   if (key == 0)
   {
      // if no pushbutton then check for local serial input
      key = kfile_getc (&serial.fd);
      if ((int16_t) key == EOF)
         key = 0;
      // if alpha key (PC connected remote) then handle pseudo-long press (upper case)
      // We still just use the bit pattern of the lowest 3 bits. Candidate keys are a, b, d or i, j, l or q, r, t
      if ((key > 0x40) && (key < 0x60))
         key |= K_LONG;
      key &= (K_LONG | K_UP | K_DOWN | K_CENTRE);
   }

   if (key == 0)
   {
      // if no local key, use remote key (if any!)
      key = remote_key;
   }

   // if key pressed then ignite backlight for a short while and assume a refresh (change to a screen)
   if (key)
   {
      lcd_backlight (1);
      backlight_timer = timer_clock ();
      refreshed = true;
   }
   // If battery charging then run backlight. Keep timer running for when battery volts drop.
   else if (gBattery > BATTERY_OK)
   {
      lcd_backlight (1);
      backlight_timer = timer_clock ();
   }
   else
   {
      // if backlight timer expired (if it exists) turn off backlight.
      if ((gBacklight) && (backlight_timer) && (timer_clock () - backlight_timer > ms_to_ticks (gBacklight * 1000)))
      {
         lcd_backlight (0);
         backlight_timer = 0;
      }
   }


   // refresh whole screen regularly if no key presses
   if (timer_clock () - refresh_timer > ms_to_ticks (REFRESH))
   {
      refresh_timer = timer_clock ();
      refreshed = true;
      print_screen (screen_number);
   }

   // process keystrokes according to mode we are in.
   switch (mode)
   {
   // up/down increment/decrement the value
   // centre saves value and exits back to pageedit mode
   // long up loads default for the field
   // long down restores original value, aborts back to pageedit mode
   case FIELDEDIT:
      pVar = (int16_t *) pgm_read_word(&variables[field].value);
      pIncFunc = (PGM_VOID_P) pgm_read_word(&variables[field].get_inc);

      // refresh the value to place the cursor on the screen in the right place
      print_field (*pVar, field, screen_number);

      switch (key)
      {
      case K_CENTRE:
         // save value and exit field edit mode
         // some fields require special action
         switch (field)
         {
         case eADJUSTTIME:
         case eHOUR:
         case eMINUTE:
         case eSECOND:
         case eDAY:
         case eMONTH:
         case eYEAR:
         case eBATCAL:
         case eSTALL_DN:
         case eSTALL_UP:
            // set Unix time in seconds, save adjustment in eeprom
            set_epoch_time ();
            break;
         }
         save_eeprom_values ();

         mode = PAGEEDIT;
         set_flash (field, false);
         break;
      case K_UP:
         /// increase by increment
         inc =  pIncFunc (field, 1);
         if (*pVar + inc <= (int16_t)pgm_read_word(&variables[field].max))
            *pVar += inc;
         else                   // wrap
            *pVar = pgm_read_word(&variables[field].min);
         break;
      case K_DOWN:
         // decrease by decrement
         inc = pIncFunc (field, -1);
         if (*pVar - inc >= (int16_t)pgm_read_word(&variables[field].min))
            *pVar -= inc;
         else                   // wrap
            *pVar = pgm_read_word(&variables[field].max);
         break;
      case K_UP | K_LONG:
         // load default
         *pVar = pgm_read_word(&variables[field].defval);
         break;
      case K_DOWN | K_LONG:
         mode = PAGEEDIT;
         // abort - reload previous values
         *pVar = saved_value;
         set_flash (field, false);      // make sure flash is off
         break;
      }
      break;


// up/down moves through fields, short centre is back to monitor
// long centre enters field edit mode
   case PAGEEDIT:
      // refresh the value to place the cursor on the screen in the right place
      pVar = (int16_t *) pgm_read_word(&variables[field].value);
      print_field (*pVar, field, screen_number);
      switch (key)
      {
      case K_CENTRE:
         mode = MONITOR;
         screen_number = FIRSTINFO;
         kfile_printf (&term.fd, "%c", TERM_BLINK_OFF);
         break;
      case K_CENTRE | K_LONG:
         // enter this field to change it
         mode = FIELDEDIT;
         set_flash (field, true);
         // save the current value
         saved_value = *pVar;
         break;
      case K_UP:
         // field on previous line
         field = find_next_field (field, screen_number, -1);
         break;
      case K_DOWN:
         // field on next line
         field = find_next_field (field, screen_number, 1);
         break;
      }
      break;


// up/down moves round setup screens, centre exits back to monitor
// long centre enters field navigation mode
   case SETUP:
      switch (key)
      {
      case K_CENTRE:
         mode = MONITOR;
         screen_number = FIRSTINFO;
         break;
      case K_CENTRE | K_LONG:
         // enter edit mode
         mode = PAGEEDIT;
         // turn on cursor
         kfile_printf (&term.fd, " %c", TERM_BLINK_ON);
         // get the first field on this screen. It gets displayed as we enter edit mode
         field = find_next_field (0, screen_number, 0);
         break;
      case K_UP:
         screen_number++;
         if (screen_number > MAXSETUP)
            screen_number = FIRSTSETUP;
         break;
      case K_DOWN:
         screen_number--;
         if (screen_number < FIRSTSETUP)
            screen_number = MAXSETUP;
         break;
      }
      break;


// up/down moves round monitor screens, centre always takes back to the summary screen
//  long centre enters setup mode
   case MONITOR:
      switch (key)
      {
      case K_CENTRE:
         screen_number = FIRSTINFO;
         break;
      case K_CENTRE | K_LONG:
         mode = SETUP;
         screen_number = FIRSTSETUP;
         break;
      case K_UP:
         screen_number = (screen_number + 1) % NUM_INFO;
         break;
      case K_DOWN:
         screen_number = (screen_number - 1 + NUM_INFO) % NUM_INFO;
         break;
      case K_UP | K_LONG:
         // get sensor number from screen number, use as base for manual screen
         sensor = screen_number - 1;
         if ((sensor == SENSOR_LOW) || (sensor == SENSOR_HIGH))
            windowopen (sensor);
         break;
      case K_DOWN | K_LONG:
         sensor = screen_number - 1;
         if ((sensor == SENSOR_LOW) || (sensor == SENSOR_HIGH))
            windowclose (sensor);
         break;
      }
      break;
   }

   // refresh with clear screen first if screen number changes
   if (screen_number != last_screen)
   {
      kfile_printf (&term.fd, "%c", TERM_CLR);
      print_screen (screen_number);
      last_screen = screen_number;
   }
}
