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

#include "rmap_util.h"

void getStimaNameByType(char *name, uint8_t type) {
  switch (type) {
    case STIMA_MODULE_TYPE_SAMPLE_ETH:
      strcpy(name, STIMA_MODULE_NAME_SAMPLE_ETH);
    break;

    case STIMA_MODULE_TYPE_REPORT_ETH:
      strcpy(name, STIMA_MODULE_NAME_REPORT_ETH);
    break;

    case STIMA_MODULE_TYPE_SAMPLE_GSM:
      strcpy(name, STIMA_MODULE_NAME_SAMPLE_GSM);
    break;

    case STIMA_MODULE_TYPE_REPORT_GSM:
      strcpy(name, STIMA_MODULE_NAME_REPORT_GSM);
    break;

    case STIMA_MODULE_TYPE_RAIN:
      strcpy(name, STIMA_MODULE_NAME_RAIN);
    break;

    case STIMA_MODULE_TYPE_TH:
      strcpy(name, STIMA_MODULE_NAME_TH);
    break;
  }
}

void stringToArray(uint8_t *array, char *string, const char *delimiter, uint8_t base) {
  uint8_t i=0;
  char *token;
  token = strtok(string, delimiter);

  while (token != NULL) {
    array[i++] = (uint8_t)strtol(token, NULL, base);
    token = strtok(NULL, delimiter);
   }
}
