/*
Copyright (C) 2014  Paolo Paruno <p.patruno@iperbole.bologna.it>
authors:
Paolo Paruno <p.patruno@iperbole.bologna.it>

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
*/


///////////////////////////////////////////////////////////////////////
// Compile time configuration

#define FIRMWARE FIRMRADIORF24
//#define RF24SLEEP

/////////////////////////

///////////////////////////////////////////////////////////////////////
// activate if you have the Hobby Components I2C LCD module (HCARDU0023)
// LCD Version 1 - Marked YwRobot Arduino LCM1602 IIC V1
#define LCD
// activate if you have relays connected to some pins
#define ATTUATORE
///////////////////////////////////////////////////////////////////////


#include "common.h"

