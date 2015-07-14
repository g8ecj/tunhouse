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

#include "features.h"


// a table of fields that are flashing
#define MAXFLASH    10
static int8_t flashing[MAXFLASH];


// Timings (in mS) for various activities
#define HEADINGS 1500L
#define BACKLIGHT 5000L
#define REFRESH 300L
#define FLASHON 700L
#define FLASHOFF 300L

extern Serial serial;
static Term term;

int16_t gUSdate;

typedef int8_t (*IncFunc_t) (int8_t field, int8_t dirn);


typedef struct vars
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
   PGM_P text;            // the text!!
   int8_t vcol;                 // the column of where to display the value
   int8_t width;                // width of the field
} Screen;


// forward reference
Vars variables[eNUMVARS];

static int8_t
null_inc (int8_t field, int8_t dirn)
{
   (void) field;
   (void) dirn;
   return 0;
}

#if 0
static int8_t
deca_inc (int8_t field, int8_t dirn)
{
   (void) dirn;
   (void) field;
   return 10;
}

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
Vars variables[eNUMVARS] = {
   {NULL, 0, 0, 0, eNORMAL, null_inc},  // dummy 1st entry
   {&gValues[SENSOR_HIGH][TINDEX_MIN],    0,     0,     0,     eSHORT,   null_inc},    // high position, minimum
   {&gValues[SENSOR_HIGH][TINDEX_NOW],    0,     0,     0,     eSHORT,   null_inc},    //                value now
   {&gValues[SENSOR_HIGH][TINDEX_MAX],    0,     0,     0,     eSHORT,   null_inc},    //                maximuum

   {&gValues[SENSOR_LOW][TINDEX_MIN],     0,     0,     0,     eSHORT,   null_inc},    // low position, minimum
   {&gValues[SENSOR_LOW][TINDEX_NOW],     0,     0,     0,     eSHORT,   null_inc},    //                value now
   {&gValues[SENSOR_LOW][TINDEX_MAX],     0,     0,     0,     eSHORT,   null_inc},    //                maximuum

   {&gValues[SENSOR_OUT][TINDEX_MIN],     0,     0,     0,     eSHORT,   null_inc},    // outside position, minimum
   {&gValues[SENSOR_OUT][TINDEX_NOW],     0,     0,     0,     eSHORT,   null_inc},    //                value now
   {&gValues[SENSOR_OUT][TINDEX_MAX],     0,     0,     0,     eSHORT,   null_inc},    //                maximuum

   {&gLimits[SENSOR_HIGH][LIMIT_UP],    -20,    30,    20,      eNORMAL,   int_inc},     // temperature to open
   {&gLimits[SENSOR_HIGH][LIMIT_DN],    -20,    30,    15,      eNORMAL,   int_inc},     //                close

   {&gLimits[SENSOR_LOW][LIMIT_UP],     -20,    30,    20,      eNORMAL,   int_inc},     // temperature to open
   {&gLimits[SENSOR_LOW][LIMIT_DN],     -20,    30,    15,      eNORMAL,   int_inc},     //                close

   {&gHOUR,                               0,    23,    12,        eDATE,   int_inc},     // hour
   {&gMINUTE,                             0,    59,     0,        eDATE,   int_inc},     // minute
   {&gSECOND,                             0,    59,     0,        eDATE,   int_inc},     // second
   {&gDAY,                                1,    31,    15,        eDATE,   int_inc},     // day
   {&gMONTH,                              1,    12,     7,        eDATE,   int_inc},     // month
   {&gYEAR,                              12,    99,    15,        eDATE,   int_inc},     // year

   {&gBattery,                            0,     0,     0,     eDECIMAL,  null_inc},     // battery volts

   {&gAdjustTime,                      -719,   719,     0,      eNORMAL,   int_inc},     // clock adjuster
   {&gUSdate,                             0,     1,     0,     eBOOLEAN,   int_inc},     // date format

   {&gWinState[SENSOR_HIGH],              0,     0,     0,      eWINDOW,  null_inc},     //open/close etc
   {&gWinState[SENSOR_LOW],               0,     0,     0,      eWINDOW,  null_inc},     //open/close etc
};


Vars daymonth[2] = {
   {&gDAY,                                1,    31,    15,        eDATE,   int_inc},     // day
   {&gMONTH,                              1,    12,     7,        eDATE,   int_inc},     // month
};

// strings to  go into progmem

const char datlim[]   PROGMEM  = "  -";
const char timlim[]   PROGMEM  = "  :";
const char dash[]     PROGMEM  = "-";
const char nulstr[]   PROGMEM  = "";
const char battstr[]  PROGMEM  = "Battery";
const char canstr[]   PROGMEM  = "Centre to Cancel";
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
   {eUP_MIN,    0,    0,     upstr,     3,    5},
   {eUP_NOW,    0,    0,     nulstr,    9,    5},
   {eUP_MAX,    0,    0,     nulstr,   15,    5},

   {eDN_MIN,    1,    0,     lostr,     3,    5},
   {eDN_NOW,    1,    0,     nulstr,    9,    5},
   {eDN_MAX,    1,    0,     nulstr,   15,    5},

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

const Screen upper[] PROGMEM = {
   {-1,         0,    3,     uppstr,    0,    0},
   {eUP_MIN,    1,    0,     minstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {eUP_NOW,    2,    0,     nowstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {eUP_MAX,    3,    0,     maxstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen lower[] PROGMEM = {
   {-1,         0,    3,     lowstr,    0,    0},
   {eDN_MIN,    1,    0,     minstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {eDN_NOW,    2,    0,     nowstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {eDN_MAX,    3,    0,     maxstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen external[] PROGMEM = {
   {-1,         0,    3,     extstr,    0,    0},
   {eOT_MIN,    1,    0,     minstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {eOT_NOW,    2,    0,     nowstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {eOT_MAX,    3,    0,     maxstr,    6,    5},
   {-1,         1,   13,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen datetime[] PROGMEM = {
   {-1,         0,    0,    timestr,    0,    0},
   {eHOUR,      1,    0,     timlim,    5,    2},
   {eMINUTE,    1,    8,     timlim,    8,    2},
   {eSECOND,    1,   11,     nulstr,   11,    2},
   {-1,         2,    0,    datestr,    0,    0},
   {eDAY,       2,    0,     datlim,    5,    2},
   {eMONTH,     3,   14,     datlim,    8,    2},
   {eYEAR,      3,   17,     nulstr,   11,    2},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen battery[] PROGMEM = {
   {-1,         0,    3,    battstr,    0,    0},
   {eBATTERY,   2,   10,    voltstr,    3,    5},
   {-2,         0,    0,     nulstr,    0,    0}
};


const Screen Set_Time[] PROGMEM = {
   {eADJUSTTIME,1,    0,    timestr,    7,    4},
   {eUSDATE,    2,    0,    datestr,    7,    3},
   {eHOUR,      3,    0,     timlim,    0,    2},
   {eMINUTE,    3,    3,     timlim,    3,    2},
   {eSECOND,    3,    6,     nulstr,    6,    2},
   {eDAY,       3,   11,     datlim,   11,    2},
   {eMONTH,     3,   14,     datlim,   14,    2},
   {eYEAR,      3,   17,     nulstr,   17,    2},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen Set_Upper[] PROGMEM = {
   {-1,         0,    2,     uppstr,    0,    0},
   {-1,         1,    2,     limstr,    0,    0},
   {eUP_LIMIT_LO,2,   4,   closestr,   12,    5},
   {-1,         1,   17,  degreestr,    0,    0},
   {eUP_LIMIT_HI,3,   4,    openstr,   12,    5},
   {-1,         1,   17,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen Set_Lower[] PROGMEM = {
   {-1,         0,    2,     lowstr,    0,    0},
   {-1,         1,    2,     limstr,    0,    0},
   {eDN_LIMIT_LO,2,   4,   closestr,   12,    5},
   {-1,         1,   17,  degreestr,    0,    0},
   {eDN_LIMIT_HI,3,   4,    openstr,   12,    5},
   {-1,         1,   17,  degreestr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen Man_Upper[] PROGMEM = {
   {-1,         0,    6,     uppstr,    0,    0},
   {eWINSTATE_HI,1,   0,     manstr,   12,    8},
   {-1,         3,    0,     canstr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

const Screen Man_Lower[] PROGMEM = {
   {-1,         0,    6,     lowstr,    0,    0},
   {eWINSTATE_LO,1,   0,     manstr,   12,    8},
   {-1,         3,    0,     canstr,    0,    0},
   {-2,         0,    0,     nulstr,    0,    0}
};

// *INDENT-ON*

#define NUM_INFO    6
#define NUM_SETUP   3
#define NUM_MANUAL  2

#define FIRSTINFO   0
#define MAXINFO     (NUM_INFO - 1)

#define FIRSTSETUP  NUM_INFO
#define MAXSETUP    (NUM_INFO + NUM_SETUP - 1)

#define FIRSTMAN    (MAXINFO + NUM_SETUP)
#define MAXMAN      (FIRSTMAN + NUM_MANUAL - 1)

#define MAXSCREENS  NUM_INFO + NUM_SETUP + NUM_MANUAL

static const Screen *screen_list[] = { summary, upper, lower, external, datetime, battery, Set_Time, Set_Upper, Set_Lower, Man_Upper, Man_Lower };


static void
set_month_day (uint8_t us)
{

   if (us)
   {
      variables[eDAY] = daymonth[1];
      variables[eMONTH] = daymonth[0];
   }
   // if already in US Date mode but want EURO mode then swap back
   else
   {
      variables[eDAY] = daymonth[0];
      variables[eMONTH] = daymonth[1];
   }
}

void
get_month_day (uint8_t * month, uint8_t * day)
{

   *month = (uint8_t) * variables[eMONTH].value;
   *day = (uint8_t) * variables[eDAY].value;

}




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
   char tritext[4][5] = { "off", "on", "auto", "oops" };
   char wintext[4][8] = {"OPENING", "CLOSING", "OPEN", "CLOSED" };


   const Screen *scrn = screen_list[screen];

   for (i = 0; (int8_t) pgm_read_byte (&scrn[i].field) != -2; i++)
   {
      if ((int8_t) pgm_read_byte (&scrn[i].field) == field)       // found the correct one
      {
         kfile_printf (&term.fd, "%c%c%c", TERM_CPC, TERM_ROW + pgm_read_byte (&scrn[i].row), TERM_COL + pgm_read_byte (&scrn[i].vcol));
         kfile_printf (&term.fd, "%.*s", pgm_read_byte (&scrn[i].width), spaces);
         if (check_flash (field))
            break;
         kfile_printf (&term.fd, "%c%c%c", TERM_CPC, TERM_ROW + pgm_read_byte (&scrn[i].row), TERM_COL + pgm_read_byte (&scrn[i].vcol));
         switch (variables[field].style)
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
            kfile_printf (&term.fd, "%.*s%d.%2u", value < 0 ? 1 : 0, "-", whole, part);
            break;
         case eSHORT:
            // split the value into those bits before and after the decimal point
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
         max = scrn[i].field;
      if ((int8_t) pgm_read_byte (&scrn[i].field) < min)
         min = scrn[i].field;
   }

   field += dirn;
   if (field > max)
      field = min;
   if (field < min)
      field = max;

   return field;
}

// find out what line this field is on
static int8_t
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
static int8_t
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
   const Screen *scrn = screen_list[screen];

   kfile_printf (&term.fd, "%c", TERM_CLR);
   while (scrn[i].field != -2)
   {
      kfile_printf (&term.fd, "%c%c%c", TERM_CPC, TERM_ROW + pgm_read_byte (&scrn[i].row), TERM_COL + pgm_read_byte (&scrn[i].col));
      for (j = 0; (const char) (pgm_read_byte (&scrn[i].text[j])); i++)
         kfile_putc((const char) pgm_read_byte (&scrn[i].text[j]), &term.fd);

      if ((int8_t) pgm_read_byte (&scrn[i].field) != -1)
         print_field (*variables[scrn[i].field].value, pgm_read_byte (&scrn[i].field), screen);

      i++;
   }
}




// scan through a few variables and check their limits to see if they should be flashing
static void
flag_warnings (void)
{
   if (gValues[SENSOR_HIGH][TINDEX_NOW] > gLimits[SENSOR_HIGH][LIMIT_UP])
      set_flash (eUP_NOW, true);
   else
      set_flash (eUP_NOW, false);


}

// used externally to verify that a value is within the correct range and set it if so.
bool
check_value (enum VARS var, int16_t value)
{
   if ((value >= variables[var].min) && (value <= variables[var].max))
   {
      *variables[var].value = value;
      return false;
   }
   else
      return true;

}

// initialise the module!
void
ui_init (void)
{

   lcd_hw_init ();
   lcd_display (1, 0, 0);
//      lcd_remapChar (lcd_degree, DEGREE);        // put the degree symbol on character 0x01

   term_init (&term);
#if PUSHBUTTONS == 1
   kbd_init ();
   kbd_setRepeatMask (K_UP | K_DOWN);
#endif
   set_month_day (gUSdate);

}




// mode values
#define MONITOR     0
#define SETUP       1
#define LIMITS      2
#define PAGEEDIT    3
#define FIELDEDIT   4
#define MANUAL      5




void
run_ui (void)
{
   static int8_t screen_number = 0, field = 0, mode = MONITOR;
   static ticks_t backlight_timer, refresh_timer;
   static int16_t working_value;
   uint8_t sensor;

   flag_warnings ();

#if PUSHBUTTONS == 1
   keymask_t key;
   key = kbd_peek ();
#else
   int16_t key;
#define K_UP          'u'
#define K_DOWN        'd'
#define K_CENTRE      'c'
   key = kfile_getc (&serial.fd);
   if (key == EOF)
      key = 0;
#endif

   // if key pressed then ignite backlight for a short while
   if (key)
   {
      lcd_bl_on ();
      backlight_timer = timer_clock ();
   }
   else
   {
      if (timer_clock () - backlight_timer > ms_to_ticks (BACKLIGHT))
      {
         lcd_bl_off ();
      }
   }

   if (timer_clock () - refresh_timer > ms_to_ticks (REFRESH))
   {
      refresh_timer = timer_clock ();
      print_screen (screen_number);
   }


   switch (mode)
   {
   case FIELDEDIT:
      // refresh the value to place the cursor on the screen in the right place
      print_field (working_value, field, screen_number);

      switch (key)
      {
         int16_t inc;
      case K_CENTRE:
         // save value and exit field edit mode
         // save the working value into the real one
         *variables[field].value = working_value;
         // some fields require special action
         switch (field)
         {
         case eHOUR:
         case eMINUTE:
         case eSECOND:
         case eDAY:
         case eMONTH:
         case eYEAR:
         case eADJUSTTIME:
            // set Unix time in seconds, save adjustment in eeprom
            set_epoch_time ();
            break;
         case eUSDATE:
            set_month_day (gUSdate);
            break;
         }
         save_eeprom_values ();

         mode = PAGEEDIT;
         set_flash (field, false);
         break;
      case K_UP:
         /// increase by increment
         inc = variables[field].get_inc (field, 1);
         if (working_value + inc <= variables[field].max)
            working_value += inc;
         else                   // wrap
            working_value = variables[field].min;
         break;
      case K_DOWN:
         // decrease by increment
         inc = variables[field].get_inc (field, -1);
         if (working_value - inc >= variables[field].min)
            working_value -= inc;
         else                   // wrap
            working_value = variables[field].max;
         break;
      }
      break;


// centre enters field edit mode
// left/right together exits field navigation and moves round setup screens
   case PAGEEDIT:
      // refresh the value to place the cursor on the screen in the right place
      print_field (*variables[field].value, field, screen_number);
      switch (key)
      {
      case K_CENTRE:
         // enter this field to change it
         mode = FIELDEDIT;
         set_flash (field, true);
         // refresh the value
         working_value = *variables[field].value;
         break;
      case K_UP:
         // field on previous line
         field = find_next_line (field, screen_number, -1);
         break;
      case K_DOWN:
         // field on next line
         field = find_next_line (field, screen_number, 1);
         break;
      }
      break;


// manual open/close - only active key is cancel (centre)
   case MANUAL:
      switch (key)
      {
      case K_CENTRE:
         mode = MONITOR;
         screen_number = FIRSTINFO;
         break;
      }
      print_screen (screen_number);
      break;


// up/down moves round monitor screens
// when in setup screen then centre enters field navigation mode
   case SETUP:
      switch (key)
      {
      case K_CENTRE:
         // enter edit mode
         mode = PAGEEDIT;
         // turn on cursor
         kfile_printf (&term.fd, " %c", TERM_BLINK_ON);
         // get the first field on this screen and display it
         field = find_next_field (0, screen_number, 0);
         print_field (*variables[field].value, field, screen_number);
         break;
      case K_UP:
         screen_number++;
         if (screen_number >= MAXSETUP)
            screen_number = FIRSTSETUP;
         break;
      case K_DOWN:
         screen_number--;
         if (screen_number < FIRSTSETUP)
            screen_number = MAXSETUP;
         break;
      case K_UP | K_LONG:
         sensor = screen_number - FIRSTSETUP;
         screen_number = FIRSTMAN + sensor;
         windowopen(sensor);
         break;
      case K_DOWN | K_LONG:
         sensor = screen_number - FIRSTSETUP;
         screen_number = FIRSTMAN + sensor;
         windowclose(sensor);
         break;
      }
      print_screen (screen_number);
      break;


// up/down moves round monitor screens
// left right moves round setup screens
// when in monitor screen then centre toggles detail mode
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
      }
      print_screen (screen_number);
      break;
   }

}
