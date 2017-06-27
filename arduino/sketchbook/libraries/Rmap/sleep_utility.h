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

#ifndef _SLEEP_UTILITY_H
#define _SLEEP_UTILITY_H

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <hardware_config.h>

#define adc_disable()							    (ADCSRA = 0)
#define adc_enable()								  (ADCSRA = 0b10000000)

#define WDT_TIMER_MAX_VALUE					  (3000)	//!< m.c.m of 1, 2, 4, 8, 15, 30, 60, 120, 250, 500

void wdt_init(uint8_t);
void power_down(uint32_t *);

#if (WDT_TIMER == WDTO_15MS)
#define WDT_OFFSET								    (15)
#define WDT_ERROR_MAX                 (0)
#elif (WDT_TIMER == WDTO_30MS)
#define WDT_OFFSET								    (30)
#define WDT_ERROR_MAX                 (0)
#elif (WDT_TIMER == WDTO_60MS)
#define WDT_OFFSET								    (60)
#define WDT_ERROR_MAX                 (0)
#elif (WDT_TIMER == WDTO_120MS)
#define WDT_OFFSET								    (120)
#define WDT_ERROR_MAX                 (0)
#elif (WDT_TIMER == WDTO_250MS)
#define WDT_OFFSET								    (250)
#define WDT_ERROR_MAX                 (0)
#elif (WDT_TIMER == WDTO_500MS)
#define WDT_OFFSET								    (500)
#define WDT_ERROR_MAX                 (0)
#elif (WDT_TIMER == WDTO_1S)
#define WDT_OFFSET								    (1)
#define WDT_ERROR_MAX                 (0.04)
#elif (WDT_TIMER == WDTO_2S)
#define WDT_OFFSET								    (2)
#define WDT_ERROR_MAX                 (0.07)
#elif (WDT_TIMER == WDTO_4S)
#define WDT_OFFSET								    (4)
#define WDT_ERROR_MAX                 (0.14)
#elif (WDT_TIMER == WDTO_8S)
#define WDT_OFFSET								    (8)
#define WDT_ERROR_MAX                 (0.2)
#endif

#define executeWdtTaskEach(timer)				(timer % WDT_OFFSET == 0 ? (wdt_timer.value % timer == 0 ? 1 : 0) : 0)
#define executeRtcTaskEach(timer)				(timer % RTC_TIMER == 0 ? (rtc_timer.value % timer == 0 ? 1 : 0) : 0)

#endif
