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

#ifndef _SDCARD_UTILITY_H
#define _SDCARD_UTILITY_H

#include <SdFat.h>
#include <Time.h>
#include <sdcard_config.h>

bool sdcard_init(SdFat *SD, uint8_t chip_select);
bool sdcard_open_file(SdFat *SD, File *file, const char *file_name, uint8_t param);
void sdcard_make_filename(time_t time, char *file_name);

#endif
