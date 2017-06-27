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

#ifndef _NTP_H
#define _NTP_H

#define NTP_SERVER_PORT               (123)
#define NTP_PACKET_LENGTH             (48)
#define NTP_RECEIVE_TIMESTAMP_OFFSET  (40)
#define NTP_1_HOUR_SECONDS            (3600UL)
#define NTP_70_YEARS_SECONDS          (2208988800UL)

#define NTP_TIMEZONE                  (1) // Europe

#include <stdint.h>
#include <string.h>
#include <EthernetUdp2.h>

namespace Ntp {
  bool sendRequest(EthernetUDP *, const char *);
  bool getResponse(EthernetUDP *);
  uint32_t getTime();
};

#endif
