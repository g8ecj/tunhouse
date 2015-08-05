//---------------------------------------------------------------------------
// Copyright (C) 2012 Robin Gilks
//
//
//  ui.h   -   User interface - drives the LCD and scans the keyboard for user input
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

#ifndef _UI_H
#define _UI_H


// all the fields displayed
enum VARS
{
   eDN_MIN = 1,
   eDN_NOW,
   eDN_MAX,

   eUP_MIN,
   eUP_NOW,
   eUP_MAX,

   eOT_MIN,
   eOT_NOW,
   eOT_MAX,

   eDN_LIMIT_HI,
   eDN_LIMIT_LO,

   eUP_LIMIT_HI,
   eUP_LIMIT_LO,

   eADJUSTTIME,
   eUSDATE,

   eHOUR,
   eMINUTE,
   eSECOND,
   eDAY,
   eMONTH,
   eYEAR,

   eBATTERY,

   eWINSTATE_LO,
   eWINSTATE_HI,

   eNUMVARS
};

extern int16_t gUSdate;


void ui_init (void);
void run_ui (uint8_t remote_key);
void set_flash (int8_t field, int8_t set);
bool check_value (enum VARS var, int16_t value);
void get_month_day (uint8_t * month, uint8_t * day);
int8_t ui_termrowget(uint8_t * buffer);
int8_t ui_termcursorget(uint8_t * row, uint8_t * column);
#endif
