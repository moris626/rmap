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

#ifndef _RMAP_UTILITY_H
#define _RMAP_UTILITY_H

#include <Time.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "stima_module.h"
#include <mqtt_config.h>
#include <sensors_config.h>

#if (USE_JSON)
#include <json_config.h>
#include <ArduinoJson.h>
#endif

#if (USE_JSON)
uint8_t jsonToMqtt(const char *json, const char *mqtt_sensor, char topic[][MQTT_SENSOR_TOPIC_LENGTH], char message[][MQTT_MESSAGE_LENGTH], tmElements_t *sensor_reading_time);
time_t getDateFromMessage(char *message);
#endif

void mqttToSd(const char *topic, const char *message, char *sd);
void sdToMqtt(const char *sd, char *topic, char *message);
void getFullTopic(char *full_topic, const char *root_topic, const char *sensor_topic);

void getStimaNameByType(char *, uint8_t);
void stringToArray(uint8_t *, char *, const char *, uint8_t);

#define macStringToArray(mac, string)     (stringToArray(mac, string, ":", 16))
#define ipStringToArray(ip, string)       (stringToArray(ip, string, ".", 10))
#define dateStringToArray(date, string)   (stringToArray(date, string, "-", 10))
#define timeStringToArray(time, string)   (stringToArray(time, string, ":", 10))

#endif
