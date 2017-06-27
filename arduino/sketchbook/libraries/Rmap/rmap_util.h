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

#ifndef _RMAP_UTIL_H
#define _RMAP_UTIL_H

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "stima_module.h"

void getStimaNameByType(char *, uint8_t);
void stringToArray(uint8_t *, char *, const char *, uint8_t);

#define macStringToArray(mac, string)     (stringToArray(mac, string, ":", 16))
#define ipStringToArray(ip, string)       (stringToArray(ip, string, ".", 10))

#endif
