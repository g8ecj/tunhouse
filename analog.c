/**
 * \file
 * <!--
 * This file is the speed controller for the walnut washer project.
 * It handles the rotary encoders, the LEDs, the push buttons and the H-bridge drivers.
 * It also handles charging the batteries from a solar panel.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Copyright 2015 Robin Gilks
 *
 * -->
 *
 * \author Robin Gilks <g8ecj@gilks.org>
 *
 * \brief Walnut Washer
 *
 * This is a minimalist project, it just gets the speed settings from a bunch of rotary
 * encoders and sets the H-bridges via PWM control. It can optionally save a sequence of up to
 * 500 speed changes.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <avr/io.h>

#include <cfg/macros.h>

#include "analog.h"

// external resistor scaling to measure up to ~20 volts (using E12 resistor values)
#define V_SCALE  ((15.0 + 5.6) / 5.6)


// read an analog input 8 times to improve accuracy
uint16_t
analog_read (uint8_t chan)
{
   uint8_t i;
   uint16_t ADC_temp = 0;

   ADMUX = chan | BV (REFS0);   // internal AREF of AVCC (5V) and ADCx

   ADCSRA = BV (ADEN) | BV (ADPS2) | BV (ADPS1);        // set ADC prescaler to 16MHz / 64 = 250kHz    

   ADCSRA |= BV (ADSC);         // do single conversion to clear out any noise
   while (!(ADCSRA & BV (ADIF)));       // wait for conversion done, ADIF flag active

   for (i = 0; i < 8; i++)      // do the ADC conversion 8 times for better accuracy 
   {
      ADCSRA |= BV (ADSC);
      while (!(ADCSRA & BV (ADIF)));

      ADC_temp += ADC;          // accumulate result (8 samples) for later averaging
   }

   ADC_temp = ADC_temp >> 3;    // average the 8 samples

   ADCSRA &= ~BV (ADEN);        // disable the ADC

   // 5 volts, scaled across 1023 values, external potential divider
   // scaled up by 100 to give 10mV resolution in an integer
   return (uint16_t) ((float) ADC_temp * 5.0 / 1023.0 * V_SCALE * 100);
}
