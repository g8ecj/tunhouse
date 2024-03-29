//---------------------------------------------------------------------------
// Copyright (C) 2013 Robin Gilks
//
//
//  eeprommap.c   -   All the variables that are in eeprom are declared here
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

#include "eeprommap.h"

#include <stdint.h>
#include <stdbool.h>
#include <avr/eeprom.h>

#include "measure.h"
#include "rtc.h"
#include "window.h"
#include "ui.h"


// these variables are all in one place so that if another is added, we don't 
// loose the existing value provided new stuff is *ALWAYS* added to the end


// motor run time
int16_t EEMEM eeMotorRun;
// values for upper and lower limits on the sensors that can control windows
int16_t EEMEM eeLimits[NUMSENSORS][NUMLIMIT];
// configured number of seconds per day to adjust clock for slow/fast 16MHz crystal
int16_t EEMEM eeAdjustTime;
// date and time stored when set and every hour so clock isn't too far out after a reset
DT_t EEMEM eeDateTime;
// timeout for the backlight
int16_t EEMEM eeBacklight;
// whether we are using the NRF radio or not
int16_t EEMEM eeRadio;
// battery calibration
int16_t EEMEM eeBatCal;
// stall current of the motors
int16_t EEMEM eeStall[NUMSENSORS];


void
load_eeprom_values (void)
{

   eeprom_read_block ((void *) &gLimits, (const void *) &eeLimits, sizeof (gLimits));
   eeprom_read_block ((void *) &gAdjustTime, (const void *) &eeAdjustTime, sizeof (gAdjustTime));
   eeprom_read_block ((void *) &gBacklight, (const void *) &eeBacklight, sizeof (gBacklight));
   eeprom_read_block ((void *) &gRadio, (const void *) &eeRadio, sizeof (gRadio));
   eeprom_read_block ((void *) &gBatCal, (const void *) &eeBatCal, sizeof (gBatCal));
   eeprom_read_block ((void *) &gStall, (const void *) &eeStall, sizeof (gStall));
   eeprom_read_block ((void *) &gMotorRun, (const void *) &eeMotorRun, sizeof (gMotorRun));


}

void
save_eeprom_values (void)
{
   eeprom_write_block ((const void *) &gAdjustTime, (void *) &eeAdjustTime, sizeof (gAdjustTime));
   eeprom_write_block ((const void *) &gLimits, (void *) &eeLimits, sizeof (gLimits));
   eeprom_write_block ((const void *) &gBacklight, (void *) &eeBacklight, sizeof (gBacklight));
   eeprom_write_block ((const void *) &gRadio, (void *) &eeRadio, sizeof (gRadio));
   eeprom_write_block ((const void *) &gBatCal, (void *) &eeBatCal, sizeof (gBatCal));
   eeprom_write_block ((const void *) &gStall, (void *) &eeStall, sizeof (gStall));
   eeprom_write_block ((const void *) &gMotorRun, (void *) &eeMotorRun, sizeof (gMotorRun));

}
