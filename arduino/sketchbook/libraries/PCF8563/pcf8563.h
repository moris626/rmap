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

#ifndef _PCF8563_H
#define _PCF8563_H

#include "Arduino.h"
#include <Wire.h>

#define PCF8563_READ_ADDRESS                (0xA3 >> 1)
#define PCF8563_WRITE_ADDRESS               (0xA2)

#define PCF8563_CONTROL_STATUS_1_ADDRESS    (0x00)
#define PCF8563_CONTROL_STATUS_2_ADDRESS    (0x01)  // BIT 4: TI_TP, BIT 3: AF, BIT 2: TF, BIT 1: AIE, BIT 0: TIE
#define PCF8563_VL_SECOND_ADDRESS           (0x02)  // BIT 0-6: seconds [0-59], BIT 7: VL
#define PCF8563_MINUTE_ADDRESS              (0x03)  // BIT 0-6: minutes [0-59]
#define PCF8563_HOUR_ADDRESS                (0x04)  // BIT 0-5: hours [0-23]
#define PCF8563_DAY_ADDRESS                 (0x05)  // BIT 0-5: days [1-31]
#define PCF8563_WEEKDAY_ADDRESS             (0x06)  // BIT 0-2: weekdays [0-6] start from Sunday
#define PCF8563_CENTURY_MONTHS_ADDRESS      (0x07)  // BIT 0-4: months [1-12], BIT 7: century [0-1]
#define PCF8563_YEAR_ADDRESS                (0x08)  // BIT 0-7: year [0-99]
#define PCF8563_MINUTE_ALARM_ADDRESS        (0x09)  // BIT 0-6: minutes alarm [0-59], BIT 7: AE_M (0: enable, 1: disable)
#define PCF8563_HOUR_ALARM_ADDRESS          (0x0A)  // BIT 0-5: hours alarm [0-23], BIT 7: AE_H (0: enable, 1: disable)
#define PCF8563_DAY_ALARM_ADDRESS           (0x0B)  // BIT 0-5: days alarm [1-31], BIT 7: AE_D (0: enable, 1: disable)
#define PCF8563_WEEKDAY_ALARM_ADDRESS       (0x0C)  // BIT 0-2: weekdays alarm [0-6], BIT 7: AE_W (0: enable, 1: disable)
#define PCF8563_CLKOUT_CONTROL_ADDRESS      (0x0D)
#define PCF8563_TIMER_CONTROL_ADDRESS       (0x0E)
#define PCF8563_TIMER_ADDRESS               (0x0F)

#define PCF8563_SECOND_MASK                 (0b01111111)
#define PCF8563_MINUTE_MASK                 (0b01111111)
#define PCF8563_HOUR_MASK                   (0b00111111)
#define PCF8563_DAY_MASK                    (0b00111111)
#define PCF8563_WEEKDAY_MASK                (0b00000111)
#define PCF8563_CENTURY_MASK                (0b10000000)
#define PCF8563_MONTH_MASK                  (0b00011111)
#define PCF8563_ALARM_MINUTE_MASK           (0b01111111)
#define PCF8563_ALARM_HOUR_MASK             (0b00111111)
#define PCF8563_ALARM_DAY_MASK              (0b00111111)
#define PCF8563_ALARM_WEEKDAY_MASK          (0b00000111)

#define PCF8563_ALARM_ENABLE_BIT            (0b10000000)  // BIT 7: AE_M, AE_H, AE_D, AE_W (0: enable, 1: disable)

#define PCF8563_CONTROL_STATUS_2_TI_TP_BIT  (0b00010000)  // BIT 4: Pulse generator (0: disable, 1: enable)
#define PCF8563_CONTROL_STATUS_2_AF_BIT     (0b00001000)  // BIT 3: Alarm Flag (0: inactive, 1: active)
#define PCF8563_CONTROL_STATUS_2_TF_BIT     (0b00000100)  // BIT 2: Timer Flag (0: inactive, 1: active)
#define PCF8563_CONTROL_STATUS_2_AIE_BIT    (0b00000010)  // BIT 1: Alarm Interrupt Enable (0: disable, 1: enable)
#define PCF8563_CONTROL_STATUS_2_TIE_BIT    (0b00000001)  // BIT 0: Timer Interrupt Enable (0: disable, 1: enable)

#define PCF8563_TIMER_CONTROL_TE_BIT        (0b10000000)  // BIT 7: Timer Enable (0: disable, 1: enable)
#define PCF8563_TIMER_CONTROL_TD_BIT        (0b00000011)  // BIT 0-1: 00: 4096 KHz, 01: 64 Hz, 10: 1 Hz, 11: 1/60 Hz

#define PCF8563_CLKOUT_CONTROL_FE_BIT       (0b10000000)  // BIT 7: Clkout Enable (0: disable, 1: enable)
#define PCF8563_CLKOUT_CONTROL_FD_BIT       (0b00000011)  // BIT 0-1: Frequency clock: 00: 32768 KHz, 01: 1024 Hz, 10: 32 Hz, 11: 1 Hz

#define PCF8563_CONTROL_STATUS_2_LENGTH     (1)
#define PCF8563_TIMER_CONTROL_LENGTH        (1)
#define PCF8563_CLKOUT_CONTROL_LENGTH       (1)
#define PCF8563_DATE_LENGTH                 (4)
#define PCF8563_TIME_LENGTH                 (3)
#define PCF8563_ALARM_LENGTH                (4)

#define PCF8563_ALARM_DISABLE               (255)

#define PCF8563_TIMER_FREQUENCY_4096_KHZ    (0b00000000)
#define PCF8563_TIMER_FREQUENCY_64_HZ       (0b00000001)
#define PCF8563_TIMER_FREQUENCY_1_HZ        (0b00000010)
#define PCF8563_TIMER_FREQUENCY_1_60_HZ     (0b00000011)

#define PCF8563_TIMER_FREQUENCY_SECONDS     (PCF8563_TIMER_FREQUENCY_1_HZ)
#define PCF8563_TIMER_FREQUENCY_MINUTES     (PCF8563_TIMER_FREQUENCY_1_60_HZ)

#define PCF8563_CLKOUT_FREQUENCY_32768_KHZ  (0b00000000)
#define PCF8563_CLKOUT_FREQUENCY_1024_HZ    (0b00000001)
#define PCF8563_CLKOUT_FREQUENCY_32_HZ      (0b00000010)
#define PCF8563_CLKOUT_FREQUENCY_1_HZ       (0b00000011)

#define PCF8563_CLKOUT_FREQUENCY_SECONDS    (PCF8563_CLKOUT_FREQUENCY_1_HZ)

namespace Pcf8563 {
  bool reset(void);
  bool getControlStatus2(uint8_t *control_status_2);
  bool setControlStatus2(uint8_t control_status_2);
  bool getTimerControl(uint8_t *timer_control);
  bool setTimerControl(uint8_t timer_control);
  bool enableClockout(void);
  bool disableClockout(void);
  bool setClockoutFrequency(uint8_t frequency);
  bool isClockoutActive(void);
  bool enableAlarm(void);
  bool disableAlarm(void);
  bool resetAlarm(void);
  bool isAlarmActive(void);
  bool getAlarm(uint8_t *hours, uint8_t *minutes, uint8_t *day, uint8_t *weekday);
  bool setAlarm(uint8_t hours, uint8_t minutes, uint8_t day = PCF8563_ALARM_DISABLE, uint8_t weekday = PCF8563_ALARM_DISABLE);
  bool enableTimer(void);
  bool disableTimer(void);
  bool resetTimer(void);
  bool isTimerActive(void);
  bool setTimer(uint8_t frequency, uint8_t timer);
  bool getDate(uint8_t *day, uint8_t *month, uint8_t *year, uint8_t *weekday = NULL, uint8_t *century = NULL);
  bool setDate(uint8_t day, uint8_t month, uint8_t year, uint8_t weekday, uint8_t century = 0);
  bool getTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);
  bool setTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
  int16_t getDaysFromTwoDate (int16_t d1, int16_t m1, int16_t y1, int16_t d2, int16_t m2, int16_t y2);
  uint32_t getTime();
};

#endif
