/**********************************************************************
 * Copyright (C) 2017  Marco Baldinetti <m.baldinetti@digiteco.it>
 * authors:
 * Marco Baldinetti <m.baldinetti@digiteco.it>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **********************************************************************/

#ifndef _STIMA_MODULE_H
#define _STIMA_MODULE_H

#define CONFIGURATION_CURRENT         (0)
#define CONFIGURATION_DEFAULT         (1)
#define CONFIGURATION_EEPROM_ADDRESS  (0)

#define STIMA_MODULE_TYPE_SAMPLE_ETH  (1)
#define STIMA_MODULE_TYPE_REPORT_ETH  (2)
#define STIMA_MODULE_TYPE_SAMPLE_GSM  (3)
#define STIMA_MODULE_TYPE_REPORT_GSM  (4)
#define STIMA_MODULE_TYPE_RAIN        (5)
#define STIMA_MODULE_TYPE_TH          (6)

#define STIMA_MODULE_NAME_SAMPLE_ETH  ("sample-eth")
#define STIMA_MODULE_NAME_REPORT_ETH  ("report-eth")
#define STIMA_MODULE_NAME_SAMPLE_GSM  ("sample-gsm")
#define STIMA_MODULE_NAME_REPORT_GSM  ("report-gsm")
#define STIMA_MODULE_NAME_RAIN        ("i2c-rain")
#define STIMA_MODULE_NAME_TH          ("i2c-th")

#endif
