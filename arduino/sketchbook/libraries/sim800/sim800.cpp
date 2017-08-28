
/*
Copyright (C) 2015  Paolo Paruno <p.patruno@iperbole.bologna.it>
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

#include <debug_config.h>

#define SERIAL_TRACE_LEVEL SIM800_SERIAL_TRACE_LEVEL

#include "sim800Client.h"

bool SIM800::isOn() {
   return (state & SIM800_STATE_ON);
}

bool SIM800::isInitialized() {
   return (state & SIM800_STATE_INITIALIZED);
}

bool SIM800::isSetted() {
   return (state & SIM800_STATE_SETTED);
}

bool SIM800::isRegistered() {
   return (state & SIM800_STATE_REGISTERED);
}

bool SIM800::isHttpInitialized() {
   return (state & SIM800_STATE_HTTP_INITIALIZED);
}

// return true when switch on
sim800_status_t SIM800::switchOn() {
   bool is_switching_on = true;
   sim800_status_t at_command_status;

   at_command_status = switchModem(is_switching_on);

   if (at_command_status != SIM800_BUSY) {
      SERIAL_INFO("SIM800 switching ON... [ %s ] [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), isOn() ? ON_STRING : OFF_STRING);
   }

   return at_command_status;
}

// return true when switch off
sim800_status_t SIM800::switchOff(uint8_t power_off_method) {
   bool is_switching_on = false;
   sim800_status_t at_command_status;

   if (power_off_method == SIM800_POWER_OFF_BY_SWITCH) {
      at_command_status = switchModem(is_switching_on);
   }
   else if (power_off_method == SIM800_POWER_OFF_BY_AT_COMMAND) {
      at_command_status = softwareSwitchOff();
   }

   if (at_command_status != SIM800_BUSY) {
      SERIAL_INFO("SIM800 switching OFF... [ %s ] [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), isOn() ? ON_STRING : OFF_STRING);
   }

   return at_command_status;
}

sim800_status_t SIM800::switchModem(bool is_switching_on) {
   static uint8_t retry;
   static sim800_power_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static bool is_error;
   static sim800_status_t sim800_status;
   sim800_status_t at_command_status;

   switch (sim800_power_state) {

      case SIM800_POWER_INIT:
         retry = 0;
         is_error = false;
         sim800_status = SIM800_BUSY;
         sim800_power_state = SIM800_POWER_SET_PIN_LOW;
         break;

      // pin low for SIM800_POWER_ON_OFF_SWITCH_DELAY_MS milliseconds
      case SIM800_POWER_SET_PIN_LOW:
         digitalWrite(on_off_pin, LOW);
         delay_ms = SIM800_POWER_ON_OFF_SWITCH_DELAY_MS;
         start_time_ms = millis();
         state_after_wait = SIM800_POWER_SET_PIN_HIGH;
         sim800_power_state = SIM800_POWER_WAIT_STATE;
         break;

      // pin high for SIM800_POWER_ON_OFF_DONE_DELAY_MS milliseconds
      case SIM800_POWER_SET_PIN_HIGH:
         digitalWrite(on_off_pin, HIGH);
         delay_ms = SIM800_POWER_ON_OFF_DONE_DELAY_MS;
         start_time_ms = millis();
         state_after_wait = SIM800_POWER_CHECK_STATUS;
         sim800_power_state = SIM800_POWER_WAIT_STATE;
         break;

      case SIM800_POWER_CHECK_STATUS:
         at_command_status = sendAt();

         // success: switching ON and is ON
         if (is_switching_on && (at_command_status == SIM800_OK)) {
            state |= SIM800_STATE_ON;
            sim800_power_state = SIM800_POWER_END;
         }
         //success: switching OFF and is OFF
         else if (!is_switching_on && (at_command_status == SIM800_ERROR)) {
            state &= ~SIM800_STATE_INITIALIZED;
            state &= ~SIM800_STATE_ON;
            sim800_power_state = SIM800_POWER_END;
         }
         // fail: switching ON and is OFF
         else if (is_switching_on && (at_command_status == SIM800_ERROR)) {
            is_error = true;
            state &= ~SIM800_STATE_INITIALIZED;
            state &= ~SIM800_STATE_ON;
            sim800_power_state = SIM800_POWER_END;
         }
         // fail: switching OFF and is ON
         else if (!is_switching_on && (at_command_status == SIM800_OK)) {
            is_error = true;
            state |= SIM800_STATE_ON;
            sim800_power_state = SIM800_POWER_END;
         }
         // wait...
         break;

      case SIM800_POWER_END:
         // success
         if (!is_error) {
            sim800_status = SIM800_OK;
            sim800_power_state = SIM800_POWER_INIT;
         }
         // retry
         else if ((++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = false;
            sim800_status = SIM800_BUSY;
            sim800_power_state = SIM800_POWER_SET_PIN_LOW;
         }
         // fail
         else {
            sim800_status = SIM800_ERROR;
            sim800_power_state = SIM800_POWER_INIT;
         }
         break;

      case SIM800_POWER_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            sim800_power_state = state_after_wait;
         }
         break;
   }

   return sim800_status;
}


// You must free the result if result is non-NULL.
// char *str_replace( const char *orig, const char *rep, const char *with) {
//    char *result; // the return string
//    const char *ins;    // the next insert point
//    char *tmp;    // varies
//    int len_rep;  // length of rep
//    int len_with; // length of with
//    int len_front; // distance between rep and end of last rep
//    int count;    // number of replacements
//
//    if (!orig)
//    return NULL;
//    if (!rep)
//    rep = "";
//    len_rep = strlen(rep);
//    if (!with)
//    with = "";
//    len_with = strlen(with);
//
//    ins = orig;
//    for (count = 0; (tmp = strstr(ins, rep)); ++count) {
//       ins = tmp + len_rep;
//    }
//
//    // first time through the loop, all the variable are set correctly
//    // from here on,
//    //    tmp points to the end of the result string
//    //    ins points to the next occurrence of rep in orig
//    //    orig points to the remainder of orig after "end of rep"
//    tmp = result = (char*)malloc(strlen(orig) + (len_with - len_rep) * count + 1);
//
//    if (!result)
//    return NULL;
//
//    while (count--) {
//       ins = strstr(orig, rep);
//       len_front = ins - orig;
//       tmp = strncpy(tmp, orig, len_front) + len_front;
//       tmp = strcpy(tmp, with) + len_with;
//       orig += len_front + len_rep; // move to next "end of rep"
//    }
//    strcpy(tmp, orig);
//    return result;
// }


SIM800::SIM800() {}

uint8_t SIM800::receive(char *rx_buffer, const char *at_ok_string, const char *at_error_string) {
   memset(rx_buffer, 0, SIM800_BUFFER_LENGTH);
   uint8_t rx_buffer_length = 0;
   bool is_responce_ok = false;
   bool is_responce_error = false;
   char *write_ptr = rx_buffer;
   char readed_char;

   uint32_t start_time_ms = millis();
   while ((millis() - start_time_ms <= 5) && (rx_buffer_length < (SIM800_BUFFER_LENGTH - 1)) && !is_responce_ok && !is_responce_error) {
      if (modem->available()) {
         start_time_ms = millis();

         readed_char = modem->read();

         if (readed_char == '\r' || readed_char == '\n') {
            if (rx_buffer_length > 2) {
               *write_ptr++ = ' ';
               rx_buffer_length++;
            }
         }
         else {
            *write_ptr++ = readed_char;
            rx_buffer_length++;

            *write_ptr = '\0';

            if (at_ok_string) {
               is_responce_ok = found(rx_buffer, at_ok_string);
            }

            if (at_error_string && !is_responce_ok){
               is_responce_error = found(rx_buffer, at_error_string);
            }
         }
      }
   }

   if (rx_buffer[rx_buffer_length-2] == ' ') {
      rx_buffer_length -= 2;
      rx_buffer[rx_buffer_length] = '\0';
   }

   if (is_responce_error) {
      rx_buffer_length = 0;
   }

   if (rx_buffer_length) {
      SERIAL_TRACE("--> %s\r\n", rx_buffer);
   }

   return rx_buffer_length;
}

// receive until timeout_ms or number of uint8_ts
// bool SIM800xxreceivelen(char *buf, uint32_t timeout_ms, unsigned int datalen) {
//    uint8_t count = 0;
//    uint32_t timeIsOut;
//    char *rec;
//    rec=buf;
//    *rec=0;
//
//    timeIsOut = millis() + timeout_ms;
//    while (timeIsOut > millis() && count < datalen) {
//       #ifdef ENABLEWDT
//       wdt_reset();
//       #endif
//       if (modem->available())
//       //if (Serial1.available())
//       {
//          count++;
//          *rec++ = modem->read();
//          //*rec++ = Serial1.read();
//          *rec = 0;           // terminate the string
//       }
//    }
//
//    IF_SDEBUG(Serial.println(F("#sim800:RECEIVED:")));
//    IF_SDEBUG(Serial.println(buf));
//
//    if (count == datalen){
//       IF_SDEBUG(Serial.println(F("#sim800:->ok")));
//    }else{
//       IF_SDEBUG(Serial.println(F("#sim800:->not ok")));
//    }
//
//    return count == datalen;
// }

void SIM800::cleanInput () {
   while (modem->available()) {
      modem->read();
   }
}

sim800_status_t SIM800::sendAtCommand(const char *command, char *buffer, const char *at_ok_string, const char *at_error_string, uint32_t timeout_ms) {
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static sim800_at_state_t state_after_wait;
   static sim800_status_t sim800_status;
   static bool is_error;
   static uint8_t rx_data_length;

   switch (sim800_at_state) {

      case SIM800_AT_INIT:
         memset(buffer, 0, SIM800_BUFFER_LENGTH);
         rx_data_length = 0;
         is_error = false;
         sim800_status = SIM800_BUSY;

         delay_ms = SIM800_AT_DELAY_MS;
         start_time_ms = millis();
         state_after_wait = SIM800_AT_SEND;
         sim800_at_state = SIM800_AT_WAIT_STATE;
         break;

      case SIM800_AT_SEND:
         cleanInput();
         start_time_ms = 0;
         strncpy(buffer, command, SIM800_BUFFER_LENGTH);
         send(buffer);
         SERIAL_TRACE("SIM800\r\n--> %s", buffer);
         sim800_at_state = SIM800_AT_RECEIVE;
         break;

      case SIM800_AT_RECEIVE:
         if (start_time_ms == 0) {
            start_time_ms = millis();
         }

         rx_data_length = receive(buffer, at_ok_string, at_error_string);

         // success
         if (rx_data_length) {
            sim800_at_state = SIM800_AT_END;
         }
         // timeout elapsed
         else if (millis() - start_time_ms > timeout_ms) {
            is_error = true;
            sim800_at_state = SIM800_AT_END;
         }
         // wait...
         break;

      case SIM800_AT_END:
         sim800_status = (is_error ? SIM800_ERROR : SIM800_OK);
         sim800_at_state = SIM800_AT_INIT;
         break;

      case SIM800_AT_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            sim800_at_state = state_after_wait;
         }
         break;
   }

   return sim800_status;
}

bool SIM800::init(uint8_t _on_off_pin, uint8_t _reset_pin) {
   on_off_pin = _on_off_pin;
   reset_pin = _reset_pin;

   pinMode(on_off_pin, OUTPUT);
   digitalWrite(on_off_pin, HIGH);
   SERIAL_DEBUG("SIM800 on/off pin [ %u ]\r\n", on_off_pin);

   if (reset_pin != 0xFF) {
      pinMode(reset_pin, OUTPUT);
      digitalWrite(reset_pin, HIGH);
      SERIAL_DEBUG("SIM800 reset pin [ %u ]\r\n", reset_pin);
   }

   state = SIM800_STATE_NONE;
   sim800_at_state = SIM800_AT_INIT;
   sim800_power_state = SIM800_POWER_INIT;
   sim800_setup_state = SIM800_SETUP_INIT;
   sim800_connection_start_state = SIM800_CONNECTION_START_INIT;
   sim800_connection_state = SIM800_CONNECTION_INIT;
   sim800_connection_stop_state = SIM800_CONNECTION_STOP_INIT;
   sim800_exit_transparent_mode_state = SIM800_EXIT_TRANSPARENT_MODE_INIT;

   modem->begin(SIM800_SERIAL_PORT_BAUD_RATE);

   return true;
}

void SIM800::setSerial(HardwareSerial *serial) {
   modem = serial;
}

// this is required to reset modem to default configuration and autobaud mode
// bool SIM800::init_fixbaud() {
//
//    char buf[SIM800_BUFFER_LENGTH];
//    char command[SIM800_BUFFER_LENGTH];
//
//    IF_SDEBUG(Serial.println(F("#sim800:initializing modem fixbaud ...")));
//
//    // try different fixed baud rate
//    long int baudrate []={1200,2400,4800,9600,19200,38400,57600,115200};
//
//    for (uint8_t i=0; (i<(sizeof(baudrate) / sizeof(long int))); i++) {
//       #ifdef ENABLEWDT
//       wdt_reset();
//       #endif
//       cleanInput();
//       IF_SDEBUG(Serial.print(F("#sim800:TRY BAUDRATE:")));
//       IF_SDEBUG(Serial.println(baudrate[i]));
//       modem->begin(baudrate[i]);
//       //Serial1.begin(baudrate[i]);
//
//       if (sendAtCommand("", buf)){
//          IF_SDEBUG(Serial.println(F("#sim800:baudrate found")));
//          sprintf(command,"+IPR=%ld",baudrate[i]);
//          if (sendAtCommand(command, buf)){
//             IF_SDEBUG(Serial.println(F("#sim800:inizialize done")));
//             if (!sendAtCommand("&F", buf)) return false;
//             // this is the default
//             //if (!sendAtCommand("+IPR=0", buf)) return false;
//             if (!sendAtCommand("&W", buf)) return false;    // save
//
//             // here a reset is required !
//
//             // TODO for now I take the baudrate of TA as my baudrate
//
//             // switch off the modem
//             // if (!sendAtCommand("+CPOWD=1", buf)) return false;
//
//             // this require hardware patch on microduino
//             // resetModem();
//
//             // redo everythings in autobaud
//             // return initAutobaud(modemPort, on_off_pin,reset_pin);
//             //
//
//             // for now I go forward, but the next boot it will be autobaud
//             state |= SIM800_STATE_INITIALIZED;
//             return true;
//          }
//       }
//    }
//    state &= ~SIM800_STATE_INITIALIZED;
//    IF_SDEBUG(Serial.println(F("#sim800:inizialize failed")));
//    #ifdef ENABLEWDT
//    wdt_reset();
//    #endif
//    return false;
// }

sim800_status_t SIM800::sendAt() {
   char buffer[SIM800_BUFFER_LENGTH];
   return sendAtCommand("AT\r\n", buffer);
}

sim800_status_t SIM800::initAutobaud() {
   sim800_status_t at_command_status;

   // sync autobaud
   at_command_status = sendAt();

   if (at_command_status == SIM800_OK) {
      state |= SIM800_STATE_INITIALIZED;
   }

   if (at_command_status != SIM800_BUSY) {
      SERIAL_DEBUG("SIM800 autobaud... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, FAIL_STRING));
   }

   return at_command_status;
}

sim800_status_t SIM800::getGsn(char *imei) {
   char buffer[SIM800_BUFFER_LENGTH];
   sim800_status_t at_command_status;

   if (!isInitialized())
      return SIM800_ERROR;

   at_command_status = sendAtCommand("AT+GSN\r\n", buffer);

   if (at_command_status == SIM800_OK) {
      if (sscanf(buffer, "%s", imei) != 1) {
         at_command_status = SIM800_ERROR;
      }
   }

   if (at_command_status != SIM800_BUSY) {
      SERIAL_DEBUG("SIM800 IMEI [ %s ] [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), imei);
   }

   return at_command_status;
}

sim800_status_t SIM800::getCreg(uint8_t *n, uint8_t *stat) {
   char buffer[SIM800_BUFFER_LENGTH];
   sim800_status_t at_command_status;

   if (!isInitialized())
      return SIM800_ERROR;

   at_command_status = sendAtCommand("AT+CREG?\r\n", buffer);

   if (at_command_status == SIM800_OK) {
      if (sscanf(buffer, "+CREG: %hhu,%hhu", n, stat) != 2) {
         *n = 0;
         *stat = 0;
         at_command_status = SIM800_ERROR;
      }
   }

   if (at_command_status != SIM800_BUSY) {
      SERIAL_DEBUG("SIM800 CREG [ %s ] [ %hhu,%hhu ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), *n, *stat);
   }

   return at_command_status;
}

sim800_status_t SIM800::getCsq(uint8_t *rssi, uint8_t *ber) {
   char buffer[SIM800_BUFFER_LENGTH];
   sim800_status_t at_command_status;

   *rssi = SIM800_RSSI_UNKNOWN;
   *ber = SIM800_BER_UNKNOWN;

   if (!isInitialized())
      return SIM800_ERROR;

   at_command_status = sendAtCommand("AT+CSQ\r\n", buffer);

   if (at_command_status == SIM800_OK) {
      if (sscanf(buffer, "+CSQ: %hhu,%hhu", rssi, ber) != 2) {
         at_command_status = SIM800_ERROR;
      }
   }

   if (at_command_status != SIM800_BUSY) {
      SERIAL_DEBUG("SIM800 CSQ [ %s ] [ %hhu,%hhu ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), *rssi, *ber);
   }

   return at_command_status;
}

sim800_status_t SIM800::getCgatt(bool *is_attached) {
   char buffer[SIM800_BUFFER_LENGTH];
   sim800_status_t at_command_status;

   uint8_t is_gprs_attached = 0;

   if (!isInitialized())
      return SIM800_ERROR;

   at_command_status = sendAtCommand("AT+CGATT?\r\n", buffer);

   if (at_command_status == SIM800_OK) {
      if (sscanf(buffer, "+CGATT: %hhu", &is_gprs_attached) != 1) {
         at_command_status = SIM800_ERROR;
      }
      *is_attached = (bool) is_gprs_attached;
   }

   if (at_command_status != SIM800_BUSY) {
      SERIAL_DEBUG("SIM800 CGATT [ %s ] [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), *is_attached ? YES_STRING : NO_STRING);
   }

   return at_command_status;
}

sim800_status_t SIM800::getCifsr(char *ip) {
   char buffer[SIM800_BUFFER_LENGTH];
   sim800_status_t at_command_status;

   if (!isInitialized())
      return SIM800_ERROR;

   at_command_status = sendAtCommand("AT+CIFSR\r\n", buffer);

   if (at_command_status == SIM800_OK) {
      if (sscanf(buffer, "%s", ip) != 1) {
         at_command_status = SIM800_ERROR;
         strcpy(ip, "0.0.0.0");
      }
   }

   return at_command_status;
}

sim800_status_t SIM800::sendCpowd() {
   char buffer[SIM800_BUFFER_LENGTH];
   sim800_status_t at_command_status;

   at_command_status = sendAtCommand("AT+CPOWD=1\r\n", buffer, AT_NORMAL_POWER_DOWN_STRING, NULL, SIM800_AT_DEFAULT_TIMEOUT_MS);

   if (at_command_status == SIM800_OK || at_command_status == SIM800_ERROR) {
      state &= ~SIM800_STATE_INITIALIZED;
      state &= ~SIM800_STATE_ON;
   }

   return at_command_status;
}

sim800_status_t SIM800::getCclk(tmElements_t *tm) {
   char buffer[SIM800_BUFFER_LENGTH];
   sim800_status_t at_command_status;

   if (!isInitialized())
      return SIM800_ERROR;

   at_command_status = sendAtCommand("AT+CCLK?\r\n", buffer);

   if (at_command_status == SIM800_OK) {
      // if (sscanf(buffer, "%s", imei) != 1) {
      //    at_command_status = SIM800_ERROR;
      // }
   }

   if (at_command_status != SIM800_BUSY) {
      SERIAL_DEBUG("SIM800 CLTS [ %s ] [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), buffer);
   }

   return at_command_status;
}

sim800_status_t SIM800::exitTransparentMode() {
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static sim800_exit_transparent_mode_state_t state_after_wait;
   static sim800_status_t sim800_status;
   static sim800_status_t at_command_status;
   static bool is_error;
   static char buffer[SIM800_BUFFER_LENGTH];

   switch (sim800_exit_transparent_mode_state) {

      case SIM800_EXIT_TRANSPARENT_MODE_INIT:
         is_error = false;
         sim800_status = SIM800_BUSY;

         // wait 1 second
         delay_ms = SIM800_WAIT_FOR_EXIT_TRANSPARENT_MODE_DELAY_MS;
         start_time_ms = millis();
         state_after_wait = SIM800_EXIT_TRANSPARENT_MODE_SEND_ESCAPE_SEQUENCE;
         sim800_exit_transparent_mode_state = SIM800_EXIT_TRANSPARENT_MODE_WAIT_STATE;
         break;

      case SIM800_EXIT_TRANSPARENT_MODE_SEND_ESCAPE_SEQUENCE:
         at_command_status = sendAtCommand("+++", buffer);

         // success
         if (at_command_status == SIM800_OK) {
            // wait 1 second
            delay_ms = SIM800_WAIT_FOR_EXIT_TRANSPARENT_MODE_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_EXIT_TRANSPARENT_MODE_END;
            sim800_exit_transparent_mode_state = SIM800_EXIT_TRANSPARENT_MODE_WAIT_STATE;
         }
         else if (at_command_status == SIM800_ERROR) {
            is_error = true;
            sim800_exit_transparent_mode_state = SIM800_EXIT_TRANSPARENT_MODE_END;
         }
         // wait...
         break;

      case SIM800_EXIT_TRANSPARENT_MODE_END:
         sim800_status = (is_error ? SIM800_ERROR : SIM800_OK);
         sim800_exit_transparent_mode_state = SIM800_EXIT_TRANSPARENT_MODE_INIT;
         SERIAL_DEBUG("SIM800 switch to command mode [ %s ]\r\n", sim800_status == SIM800_OK ? OK_STRING : FAIL_STRING);
         break;

      case SIM800_EXIT_TRANSPARENT_MODE_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            sim800_exit_transparent_mode_state = state_after_wait;
         }
         break;
   }

   return sim800_status;
}

sim800_status_t SIM800::connection(const char *tipo, const char *server, const int port) {
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static sim800_connection_state_t state_after_wait;
   static sim800_status_t sim800_status;
   static sim800_status_t at_command_status;
   static bool is_error;
   static uint8_t rx_data_length;
   static char buffer[SIM800_BUFFER_LENGTH];
   static char buffer2[SIM800_BUFFER_LENGTH];

   switch (sim800_connection_state) {

      case SIM800_CONNECTION_INIT:
         rx_data_length = 0;
         is_error = false;
         sim800_status = SIM800_BUSY;

         if (isInitialized()) {
            snprintf(buffer2, SIM800_BUFFER_LENGTH, "AT+CIPSTART=\"%s\",\"%s\",\"%i\"\r\n", tipo, server, port);
            sim800_connection_state = SIM800_CONNECTION_OPEN;
         }
         else {
            is_error = true;
            sim800_connection_state = SIM800_CONNECTION_END;
         }

         break;

      case SIM800_CONNECTION_OPEN:
         at_command_status = sendAtCommand(buffer2, buffer, AT_OK_STRING, AT_ERROR_STRING, SIM800_CIPSTART_RESPONSE_TIME_MAX_MS);

         // success
         if (at_command_status == SIM800_OK) {
            sim800_connection_state = SIM800_CONNECTION_CHECK_STATUS;
         }
         // fail
         else if (at_command_status == SIM800_ERROR) {
            is_error = true;
            sim800_connection_state = SIM800_CONNECTION_END;
         }
         // wait...
         break;

      case SIM800_CONNECTION_CHECK_STATUS:
         if (start_time_ms == 0) {
            start_time_ms = millis();
         }

         rx_data_length = receive(buffer, NULL, NULL);

         // fail
         if (rx_data_length && found(buffer, AT_CONNECT_FAIL_STRING)) {
            SERIAL_ERROR("SIM800 %s status... [ %s ] [ %s ]\r\n", tipo, ERROR_STRING, buffer);
            is_error = true;
            sim800_connection_state = SIM800_CONNECTION_END;
         }
         // success
         else if (rx_data_length && found(buffer, AT_CONNECT_OK_STRING)) {
            SERIAL_INFO("SIM800 %s status... [ %s ] [ %s ]\r\n", tipo, OK_STRING, buffer);
            sim800_connection_state = SIM800_CONNECTION_END;
         }
         // timeout fail
         else if (millis() - start_time_ms > SIM800_CIPSTART_RESPONSE_TIME_MAX_MS) {
            SERIAL_ERROR("SIM800 %s status... [ %s ]\r\n", tipo, ERROR_STRING);
            is_error = true;
            sim800_connection_state = SIM800_CONNECTION_END;
         }
         // wait...
         break;

      case SIM800_CONNECTION_END:
         sim800_status = (is_error ? SIM800_ERROR : SIM800_OK);
         sim800_connection_state = SIM800_CONNECTION_INIT;
         break;

      case SIM800_CONNECTION_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            sim800_connection_state = state_after_wait;
         }
         break;
   }

   return sim800_status;
}

sim800_status_t SIM800::setup() {
   static uint8_t retry;
   static sim800_setup_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static char buffer[SIM800_BUFFER_LENGTH];
   static sim800_status_t sim800_status;
   static bool is_error;
   static bool is_registered;
   sim800_status_t at_command_status;
   uint8_t n;
   uint8_t stat;
   uint8_t rssi;
   uint8_t ber;

   switch (sim800_setup_state) {

      case SIM800_SETUP_INIT:
         retry = 0;
         is_error = false;
         sim800_status = SIM800_BUSY;

         // is baud set?
         if (isInitialized()) {
            sim800_setup_state = SIM800_SETUP_RESET;
         }
         else {
            is_error = true;
            sim800_setup_state = SIM800_SETUP_END;
         }
         break;

      case SIM800_SETUP_RESET:
         // reset to factory default
         at_command_status = sendAtCommand("AT&F\r\n", buffer);

         // success
         if (at_command_status == SIM800_OK) {
            delay_ms = SIM800_WAIT_FOR_SETUP_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_SETUP_ECHO_MODE;
            sim800_setup_state = SIM800_SETUP_WAIT_STATE;
         }
         // fail
         else if (at_command_status == SIM800_ERROR) {
            is_error = true;
            sim800_setup_state = SIM800_SETUP_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 reset to factory default [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }
         break;

      case SIM800_SETUP_ECHO_MODE:
         // echo mode off
         at_command_status = sendAtCommand("ATE0\r\n", buffer);

         // success
         if (at_command_status == SIM800_OK) {
            sim800_setup_state = SIM800_SETUP_WAIT_NETWORK;
         }
         // fail
         else if (at_command_status == SIM800_ERROR) {
            is_error = true;
            sim800_setup_state = SIM800_SETUP_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 echo mode off [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }

         // wait...
         break;

      case SIM800_SETUP_WAIT_NETWORK:
         is_registered = false;

         at_command_status = getNetworkStatus(&n, &stat);

         // success
         if (at_command_status == SIM800_OK) {
            switch (stat) {
               case 0:
                  is_registered = false;
                  SERIAL_INFO("SIM800 NOT registered... [ %s ]\r\n", ERROR_STRING);
                  break;

               case 1:
                  is_registered = true;
                  SERIAL_INFO("SIM800 network registered... [ %s ]\r\n", OK_STRING);
                  break;

               case 2:
                  is_registered = false;
                  SERIAL_INFO("SIM800 searching network...\r\n");
                  break;

               case 3:
                  is_registered = false;
                  SERIAL_INFO("SIM800 network registration denied... [ %s ]\r\n", ERROR_STRING);
                  break;

               case 4:
                  is_registered = false;
                  SERIAL_INFO("SIM800 unknown network...\r\n");
                  break;

               case 5:
                  is_registered = true;
                  SERIAL_INFO("SIM800 network registered (roaming)... [ %s ]\r\n", OK_STRING);
                  break;
            }
         }

         // success
         if (at_command_status == SIM800_OK && is_registered) {
            retry = 0;
            sim800_setup_state = SIM800_SETUP_GET_SIGNAL_QUALITY;
         }
         // retry
         else if (at_command_status == SIM800_OK && !is_registered && (++retry) < SIM800_WAIT_FOR_NETWORK_RETRY_COUNT_MAX) {
            delay_ms = SIM800_WAIT_FOR_NETWORK_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_SETUP_WAIT_NETWORK;
            sim800_setup_state = SIM800_SETUP_WAIT_STATE;
         }
         // fail
         else if (at_command_status == SIM800_ERROR || retry >= SIM800_WAIT_FOR_NETWORK_RETRY_COUNT_MAX) {
            retry = 0;
            is_error = true;
            sim800_setup_state = SIM800_SETUP_END;
         }
         // wait
         break;

      case SIM800_SETUP_GET_SIGNAL_QUALITY:
         at_command_status = getSignalQuality(&rssi, &ber);

         // success or fail: dont care
         if (at_command_status == SIM800_OK || at_command_status == SIM800_ERROR) {
            sim800_setup_state = SIM800_SETUP_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 signal [ %s ] [ rssi %hhu, ber %hhu ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), rssi, ber);
         }

         // wait
         break;

      case SIM800_SETUP_END:
         if (is_error) {
            sim800_status = SIM800_ERROR;
            state &= ~SIM800_STATE_SETTED;
         }
         else {
            sim800_status = SIM800_OK;
            state |= SIM800_STATE_SETTED;
         }

         sim800_setup_state = SIM800_SETUP_INIT;
         SERIAL_INFO("SIM800 setup... [ %s ]\r\n", printStatus(sim800_status, OK_STRING, FAIL_STRING));
         break;

      case SIM800_SETUP_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            sim800_setup_state = state_after_wait;
         }
         break;
   }

   return sim800_status;
}

sim800_status_t SIM800::startConnection(const char *apn, const char *username, const char *password) {
   static uint8_t retry;
   static sim800_connection_start_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static char ip[SIM800_IP_LENGTH];
   static char buffer[SIM800_BUFFER_LENGTH];
   static char buffer2[SIM800_BUFFER_LENGTH];
   static sim800_status_t sim800_status;
   static bool is_error;
   static bool is_attached;
   sim800_status_t at_command_status;

   switch (sim800_connection_start_state) {

      case SIM800_CONNECTION_START_INIT:
         retry = 0;
         is_error = false;
         sim800_status = SIM800_BUSY;

         if (isSetted()) {
            sim800_connection_start_state = SIM800_CONNECTION_START_CHECK_GPRS;
         }
         else {
            is_error = true;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }
         break;

      case SIM800_CONNECTION_START_CHECK_GPRS:
         is_attached = false;
         at_command_status = isGprsAttached(&is_attached);

         // success
         if (at_command_status == SIM800_OK && is_attached) {
            retry = 0;
            sim800_connection_start_state = SIM800_CONNECTION_START_SINGLE_IP;
         }
         // gprs not attached
         else if (at_command_status == SIM800_OK && !is_attached) {
            retry = 0;
            delay_ms = SIM800_WAIT_FOR_ATTACH_GPRS_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_ATTACH_GPRS;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_CHECK_GPRS;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // fail
         else if (at_command_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            retry = 0;
            is_error = true;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 GPRS attach... [ %s ] [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), is_attached ? YES_STRING : NO_STRING);
         }

         // wait
         break;

      case SIM800_CONNECTION_START_ATTACH_GPRS:
         // attach GPRS
         at_command_status = sendAtCommand("AT+CGATT=1\r\n", buffer, AT_OK_STRING, AT_ERROR_STRING, SIM800_CGATT_RESPONSE_TIME_MAX_MS);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_start_state = SIM800_CONNECTION_START_SINGLE_IP;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_ATTACH_GPRS;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // fail
         else if (at_command_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = true;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 attach GPRS... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, FAIL_STRING));
         }

         // wait...
         break;

      case SIM800_CONNECTION_START_SINGLE_IP:
         at_command_status = sendAtCommand("AT+CIPMUX=0\r\n", buffer);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_start_state = SIM800_CONNECTION_START_TRANSPARENT_MODE;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_SINGLE_IP;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // fail
         else if (at_command_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = true;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 single IP mode... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }

         // wait...
         break;

      case SIM800_CONNECTION_START_TRANSPARENT_MODE:
         at_command_status = sendAtCommand("AT+CIPMODE=1\r\n", buffer);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_start_state = SIM800_CONNECTION_START_TRANSPARENT_MODE_CONFIG;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_TRANSPARENT_MODE;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // fail
         else if (at_command_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = true;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 switch to data mode... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }

         // wait...
         break;

      // AT+CIPCCFG: (NmRetry:3-8),(WaitTm:2-10),(SendSz:1-1460),(esc:0,1) ,(Rxmode:0,1), (RxSize:50-1460),(Rxtimer:20-1000)
      // NmRetry:    Number of retries to be made for an IP packet.
      // WaitTm:     Number of 200ms intervals to wait for serial input before sending the packet
      // SendSz:     Size in uint8_ts of data block to be received from serial port before sending.
      // Esc:        Whether turn on the escape sequence, default is TRUE.
      // Rxmode:     Whether to set time interval during output data from serial port.
      // RxSize:     Output data length for each time, default value is 1460.
      // Rxtimer:    Time interval (ms) to wait for serial port to output data again. Default value: 50ms
      case SIM800_CONNECTION_START_TRANSPARENT_MODE_CONFIG:
         at_command_status = sendAtCommand("AT+CIPCCFG=8,2,1024,1,0,1460,50\r\n", buffer);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_start_state = SIM800_CONNECTION_START_APN_USERNAME_PASSWORD;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_TRANSPARENT_MODE_CONFIG;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // fail
         else if (at_command_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = true;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 transparent mode... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }

         // wait...
         break;

      case SIM800_CONNECTION_START_APN_USERNAME_PASSWORD:
         snprintf(buffer2, SIM800_BUFFER_LENGTH, "AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", apn, username, password);
         at_command_status = sendAtCommand(buffer2, buffer);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_start_state = SIM800_CONNECTION_START_CONNECT;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_APN_USERNAME_PASSWORD;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // fail
         else if (at_command_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = true;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 set APN, username and password... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }

         // wait...
         break;

      case SIM800_CONNECTION_START_CONNECT:
         at_command_status = sendAtCommand("AT+CIICR\r\n", buffer, AT_OK_STRING, AT_ERROR_STRING, SIM800_CIICR_RESPONSE_TIME_MAX_MS);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_GET_IP;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_CONNECT;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // fail
         else if (at_command_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = true;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 setting up connection... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }

         // wait...
         break;

      case SIM800_CONNECTION_START_GET_IP:
         at_command_status = getIp(ip);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_START_GET_IP;
            sim800_connection_start_state = SIM800_CONNECTION_START_WAIT_STATE;
         }
         // fail
         else if (sim800_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            retry = 0;
            is_error = true;
            sim800_connection_start_state = SIM800_CONNECTION_START_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 IP... [ %s ] [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING), ip);
         }

         // wait...
         break;

      case SIM800_CONNECTION_START_END:
         sim800_status = (is_error ? SIM800_ERROR : SIM800_OK);
         sim800_connection_start_state = SIM800_CONNECTION_START_INIT;
         SERIAL_INFO("SIM800 start connection... [ %s ]\r\n", printStatus(sim800_status, OK_STRING, ERROR_STRING));
         break;

      case SIM800_CONNECTION_START_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            sim800_connection_start_state = state_after_wait;
         }
         break;
   }

   return sim800_status;
}

sim800_status_t SIM800::stopConnection() {
   static uint8_t retry;
   static sim800_connection_stop_state_t state_after_wait;
   static uint32_t delay_ms;
   static uint32_t start_time_ms;
   static char buffer[SIM800_BUFFER_LENGTH];
   static sim800_status_t sim800_status;
   static bool is_error;
   sim800_status_t at_command_status;

   switch (sim800_connection_stop_state) {

      case SIM800_CONNECTION_STOP_INIT:
         retry = 0;
         is_error = false;
         sim800_status = SIM800_BUSY;

         sim800_connection_stop_state = SIM800_CONNECTION_STOP_SWITCH_TO_COMMAND_MODE;
         break;

      case SIM800_CONNECTION_STOP_SWITCH_TO_COMMAND_MODE:
         at_command_status = exitTransparentMode();

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_CLOSE;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_STOP_SWITCH_TO_COMMAND_MODE;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_WAIT_STATE;
         }
         // fail
         else if (sim800_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            retry = 0;
            is_error = true;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_END;
         }
         // wait
         break;

      case SIM800_CONNECTION_STOP_CLOSE:
         at_command_status = sendAtCommand("AT+CIPCLOSE=0\r\n", buffer, AT_CIPCLOSE_OK_STRING, AT_CIPCLOSE_ERROR_STRING, SIM800_AT_DEFAULT_TIMEOUT_MS);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_CLOSE_PDP;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_STOP_CLOSE;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_WAIT_STATE;
         }
         // fail
         else if (sim800_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = true;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 stop connection... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }

         // wait
         break;

      case SIM800_CONNECTION_STOP_CLOSE_PDP:
         at_command_status = sendAtCommand("AT+CIPSHUT\r\n", buffer, AT_CIPSHUT_OK_STRING, AT_CIPSHUT_ERROR_STRING, SIM800_CIPSHUT_RESPONSE_TIME_MAX_MS);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_DETACH_GPRS;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_STOP_DETACH_GPRS;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_WAIT_STATE;
         }
         // fail
         else if (sim800_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = true;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 PDP close... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }

         // wait
         break;

      case SIM800_CONNECTION_STOP_DETACH_GPRS:
         at_command_status = sendAtCommand("AT+CGATT=0\r\n", buffer, AT_OK_STRING, AT_ERROR_STRING, SIM800_CGATT_RESPONSE_TIME_MAX_MS);

         // success
         if (at_command_status == SIM800_OK) {
            retry = 0;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_END;
         }
         // retry
         else if (at_command_status == SIM800_ERROR && (++retry) < SIM800_GENERIC_RETRY_COUNT_MAX) {
            delay_ms = SIM800_GENERIC_WAIT_DELAY_MS;
            start_time_ms = millis();
            state_after_wait = SIM800_CONNECTION_STOP_DETACH_GPRS;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_WAIT_STATE;
         }
         // fail
         else if (sim800_status == SIM800_ERROR || retry >= SIM800_GENERIC_RETRY_COUNT_MAX) {
            is_error = true;
            sim800_connection_stop_state = SIM800_CONNECTION_STOP_END;
         }

         if (at_command_status != SIM800_BUSY) {
            SERIAL_INFO("SIM800 detach GPRS... [ %s ]\r\n", printStatus(at_command_status, OK_STRING, ERROR_STRING));
         }

         // wait...
         break;

      case SIM800_CONNECTION_STOP_END:
         sim800_status = (is_error ? SIM800_ERROR : SIM800_OK);
         sim800_connection_stop_state = SIM800_CONNECTION_STOP_INIT;
         SERIAL_INFO("SIM800 stop connection... [ %s ]\r\n", printStatus(sim800_status, OK_STRING, FAIL_STRING));
         break;

      case SIM800_CONNECTION_STOP_WAIT_STATE:
         if (millis() - start_time_ms > delay_ms) {
            sim800_connection_stop_state = state_after_wait;
         }
         break;
   }

   return sim800_status;
}

// bool SIM800::startNetwork(const char *apn, const char *user, const char *pwd ) {
//    char buf[SIM800_BUFFER_LENGTH];
//    char bufcommand[SIM800_COMMAND_BUFFER_LENGTH];
//
//    IF_SDEBUG(Serial.println(F("#sim800:start network ...")));
//
//    if(!isInitialized()) return false;
//
//    // restart network if already connected
//    if (isRegistered()) stopNetwork();
//
//    if (!sendAtCommand("+CIPMUX=0", buf)) return false; //IP Single Connection
//
//    if (!isRegistered()){
//       if (sendAtCommand("+SAPBR=3,1,\"Contype\",\"GPRS\"", buf)){
//          sprintf(bufcommand,"+SAPBR=3,1,\"APN\",\"%s\"",apn );
//          if (sendAtCommand(bufcommand, buf)){
//             sprintf(bufcommand,"+SAPBR=3,1,\"USER\",\"%s\"",user );
//             if (sendAtCommand(bufcommand, buf)){
//                sprintf(bufcommand,"+SAPBR=3,1,\"PWD\",\"%s\"",pwd );
//                if (sendAtCommand(bufcommand, buf)){
//                   // 85 sec timeout_ms!
//                   if(sendAtCommand("+SAPBR=1,1", buf, AT_OK_STRING, AT_ERROR_STRING, 85000)){
//                      if(sendAtCommand("+SAPBR=2,1", buf)){
//                         state |= SIM800_STATE_REGISTERED;
//                      }
//                   }
//                }
//             }
//          }
//       }
//    }
//
//    // if(isRegistered() {
//    //    Serial.println(F("start network done"));
//    // }else{
//    //    Serial.println(F("start network failed"));
//    // }
//
//    return isRegistered();
//
// }

// // return false if wrong response or already disconnected
// bool SIM800::stopNetwork() {
//    char buf[SIM800_BUFFER_LENGTH];
//    bool status=false;
//
//    IF_SDEBUG(Serial.println(F("#sim800:stop network ...")));
//
//    //if (isRegistered()){
//    // force stop ever!!
//    status = sendAtCommand("+SAPBR=0,1", buf , AT_OK_STRING, AT_ERROR_STRING, 65000);
//    if (status) {
//       state &= ~SIM800_STATE_REGISTERED;
//    }
//    //}
//
//    IF_SDEBUG(Serial.println(F("#sim800:stop network done")));
//    return status;
// }
//
//
//
// bool SIM800::httpGET(const char* server, int port, const char* path, char* result, int resultlength)
// {
//    char bufcommand[SIM800_COMMAND_BUFFER_LENGTH];
//    char buf[SIM800_BUFFER_LENGTH];
//    char* newpath;
//
//    IF_SDEBUG(Serial.println(F("#sim800:start httpget ...")));
//
//    if(!isInitialized()) return false;
//    if(!isRegistered()) return false;
//
//    if (isHttpInitialized()){
//       sendAtCommand("+HTTPTERM", buf);
//       state &= ~SIM800_STATE_HTTP_INITIALIZED;
//    }
//
//    if(!sendAtCommand("+HTTPINIT", buf)){
//       sendAtCommand("+HTTPTERM", buf);
//       if(!sendAtCommand("+HTTPINIT", buf)) return false;
//       state |= SIM800_STATE_HTTP_INITIALIZED;
//    }
//
//    if(!sendAtCommand("+HTTPPARA=\"CID\",1", buf)) return false;
//    newpath = str_replace(path,"\"", "%22");
//    sprintf(bufcommand, "+HTTPPARA=\"URL\",\"%s%s\"",server,newpath );
//    free(newpath);
//    if(!sendAtCommand(bufcommand, buf)) return false;
//    if(!sendAtCommand("+HTTPACTION=0", buf)) return false;
//    //receive(buf,5000,"\n",NULL);             // here we receive some spourious \r\n; do not wait for it
//    receive(buf,20000,"+HTTPACTION",NULL);     // timeout_ms for response 20 sec
//    receive(buf,5000,"\n",NULL);
//    int method;
//    int status;
//    int datalen;
//    int token_count = sscanf(buf,":%i,%i,%i",&method,&status,&datalen);
//    if ( token_count == 3 ){
//       IF_SDEBUG(Serial.print(F("#sim800:method: ")));
//       IF_SDEBUG(Serial.println(method));
//       IF_SDEBUG(Serial.print(F("#sim800:status: ")));
//       IF_SDEBUG(Serial.println(status));
//       IF_SDEBUG(Serial.print(F("#sim800:datalen: ")));
//       IF_SDEBUG(Serial.println(datalen));
//    }else{
//       IF_SDEBUG(Serial.println(F("#sim800:ERROR httpaction")));
//       IF_SDEBUG(Serial.print(F("#sim800:token count: ")));
//       IF_SDEBUG(Serial.println(token_count));
//       IF_SDEBUG(Serial.println(buf));
//       return false;
//    }
//    if (status != 200) return false;
//    send("AT+HTTPREAD\r\n");
//    if(!receive(buf,5000,"\r\n",NULL)) return false;  // get +HTTPREAD: n
//    if(!receive(buf,5000,"\r\n",NULL)) return false;   // gel null line
//
//    if (datalen+1 > resultlength){
//       IF_SDEBUG(Serial.println(F("#sim800:ERROR no buffer space for http response")));
//       return false;
//    }
//    if(!receivelen(result,5000,datalen)) return false;
//    if(!receive(buf,5000,"OK\r\n",NULL)) return false;
//    if(!sendAtCommand("+HTTPTERM", buf)) return false;
//    state &= ~SIM800_STATE_HTTP_INITIALIZED;
//
//    return true;
// }
//
// bool SIM800::checkNetwork() {
//    char buf[SIM800_BUFFER_LENGTH];
//    bool retstatus;
//
//    if(!isInitialized()) return false;
//    if(!isRegistered()) return false;
//
//    IF_SDEBUG(Serial.println(F("#sim800:checking network ...")));
//    if ((retstatus=sendAtCommand("+SAPBR=2,1", buf))){
//       // here the secon number=1 means all is OK
//       if (found(buf,"+SAPBR: 1,1")) {
//          state |= SIM800_STATE_REGISTERED;
//       }else{
//          IF_SDEBUG(Serial.println(F("#sim800:ERROR network status")));
//          IF_SDEBUG(Serial.println(buf));
//          state &= ~SIM800_STATE_REGISTERED;
//          retstatus=false;
//       }
//
//    }else {
//       state &= ~SIM800_STATE_REGISTERED;
//    }
//
//    IF_SDEBUG(Serial.println(F("#sim800:cheking network done")));
//    return retstatus;
// }

// todo to be used
/*
* Reads a token from the given string. Token is seperated by the
* given delimiter.
*/
/*

char *SIM800::readToken(char *str, char *buf, char delimiter) {
uint8_t c = 0;
while (true) {
c = *str++;
if ((c == delimiter) || (c == '\0')) {
break;
}
else if (c != ' ') {
*buf++ = c;
}
}
*buf = '\0';
return str;
}
*/

// bool SIM800::resetModem() {
//    digitalWrite(reset_pin, LOW);
//    delay(300);
//    digitalWrite(reset_pin, HIGH);
//    delay(3000);
//    #ifdef ENABLEWDT
//    wdt_reset();
//    #endif
//    return true;
// }

time_t SIM800::RTCget()   // Aquire data from buffer and convert to time_t
{
   tmElements_t tm;

   if (RTCread(tm) != 0){
      return 0UL;
   }
   return(makeTime(tm));
}

uint8_t  SIM800::RTCset(time_t t)
{
   tmElements_t tm;

   if (t!= 0UL){
      breakTime(t, tm);
      return RTCwrite(tm);
   }
   return 1;
}

uint8_t SIM800::RTCread( tmElements_t &tm){

   char buf[SIM800_BUFFER_LENGTH];

   if (sendAtCommand("AT+CCLK?",buf)){

      //"yy/MM/dd,hh:mm:ss±zz"
      //int token_count = sscanf(buf,"+CCLK: \"%02hu/%02hu/%02hu,%02hu:%02hu:%02hu+00\"\r\n",&tm.Year,&tm.Month,&tm.Day,&tm.Hour,&tm.Minute,&tm.Second);
      //int token_count = sscanf(buf,"+CCLK: \"%02SCNd8/%02SCNd8/%02SCNd8,%02SCNd8:%02SCNd8:%02SCNd8+00\"\r\n",&tm.Year,&tm.Month,&tm.Day,&tm.Hour,&tm.Minute,&tm.Second);
      //int token_count = sscanf(buf,"+CCLK: \"%02"SCNd8"/%02"SCNd8"/%02"SCNd8",%02"SCNd8":%02"SCNd8":%02"SCNd8"+00\"\r\n",&tm.Year,&tm.Month,&tm.Day,&tm.Hour,&tm.Minute,&tm.Second);

      int token_count = sscanf(buf,"+CCLK: \"%02hhd/%02hhd/%02hhd,%02hhd:%02hhd:%02hhd+00\"\r\n",&tm.Year,&tm.Month,&tm.Day,&tm.Hour,&tm.Minute,&tm.Second);

      //tm.Wday
      if (token_count == 6){
         if (tm.Year == 1) return 1;         // never set
         tm.Year = y2kYearToTm(tm.Year);
         return 0;
      }
   }
   return 1;                            // error in atcommand
}

uint8_t SIM800::RTCwrite(tmElements_t &tm){

   //sendAtCommand("+CCLK=\"yy/MM/dd,hh:mm:ss±zz\"",buf);
   //sendAtCommand("+CCLK=\"15/04/27,16:12:00+00\"",buf);

   char buf[SIM800_BUFFER_LENGTH];
   char command[SIM800_BUFFER_LENGTH];

   sprintf(command,"+CCLK=\"%02i/%02i/%02i,%02i:%02i:%02i+00\"",tmYearToY2k(tm.Year),tm.Month,tm.Day,tm.Hour,tm.Minute,tm.Second);
   //tm.Wday

   if (sendAtCommand(command,buf)){
      return 0;
   }
   return 1;

}

// bool SIM800::TCPstop()
// {
//    char buf[SIM800_BUFFER_LENGTH];
//
//    state &= ~SIM800_STATE_REGISTERED;
//
//    if (!sendAtCommand("+CIPSHUT", buf,"SHUT OK", AT_ERROR_STRING, 65000)) return false;
//
//    // If attached to GPRS detach GPRS
//    if(!sendAtCommand("+CGATT?", buf, "+CGATT: 0", "+CGATT: 1", 5000)) {
//       if(!sendAtCommand("+CGATT=0", buf)) return false;
//    }
//
//    return true;
// }

// *****************************************************************************
// etherclient compatibility
// *****************************************************************************

sim800Client::sim800Client() {}

int sim800Client::connect(IPAddress ip, int port) {
   char server[16];
   sprintf(server, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
   return connection("TCP", server, port);
}

int sim800Client::connect(const char *server, int port) {
   return (int) connection("TCP", server, port);
}

uint8_t sim800Client::connected() {
   return isHttpInitialized();
}

int sim800Client::available() {
   return modem->available();
}

int sim800Client::read() {
   return modem->read();
}

int sim800Client::readBytes(char *buffer, size_t size) {
  return modem->readBytes(buffer, size);
}

int sim800Client::readBytes(uint8_t *buffer, size_t size) {
  return modem->readBytes(buffer, size);
}

void sim800Client::setTimeout(uint32_t timeout_ms) {
   modem->setTimeout(timeout_ms);
}

size_t sim800Client::write(uint8_t buffer) {
   return modem->write(buffer);
}

size_t sim800Client::write(const uint8_t *buffer, size_t size) {
   return modem->write(buffer, size);
}

bool sim800Client::transparent()
{
   char buf[SIM800_BUFFER_LENGTH];

   // IF_SDEBUG(Serial.println(F("#sim800:going to transparent mode")));
   return sendAtCommand("O", buf, "CONNECT", AT_ERROR_STRING, 5000);

}

void sim800Client::flush() {
}

void sim800Client::stop() {
}
