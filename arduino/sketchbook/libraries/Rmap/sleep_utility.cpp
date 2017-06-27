/**********************************************************************
Copyright (C) 2017  Marco Baldinetti <m.baldinetti@digiteco.it>
authors:
Marco Baldinetti <m.baldinetti@digiteco.it>

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
**********************************************************************/

#include <Arduino.h>
#include "sleep_utility.h"

void wdt_init(uint8_t timer) {
	// Don't replace "WDTCSR = timer" with "WDTCSR = ((timer & 0x08) << 2) | (timer & 0x07)"
	// because the assignement in WTCSR MUST terminate in under 4 cycle of clock
	// and in the case it is replaced this doesn't happen.
	timer = ((timer & 0x08) << 2) | (timer & 0x07);

	// clear various "reset" flags
	MCUSR = 0;
	// allow changes, disable reset
	WDTCSR = (1 << WDCE) | (1 << WDE);
	// set interrupt mode and an interval
	WDTCSR = timer | (1 << WDIE);
	wdt_reset();
}

/*! \fn void power_down(void)
 *  \brief Enter in power down state if DEBOUNCING_POWER_DOWN_TIME_MS was elapsed from awakened_event_occurred_time_ms.
 *  \return void.
 */
void power_down(uint32_t *awakened_event_occurred_time_ms) {
  if (millis() - *awakened_event_occurred_time_ms > DEBOUNCING_POWER_DOWN_TIME_MS) {
    adc_disable();
    noInterrupts ();
    sleep_enable();

    // turn off brown-out enable in software
    MCUCR = bit (BODS) | bit (BODSE);
    MCUCR = bit (BODS);
    interrupts ();  // guarantees next instruction executed
    sleep_cpu();
    sleep_disable();
    *awakened_event_occurred_time_ms = millis();
  }
}
