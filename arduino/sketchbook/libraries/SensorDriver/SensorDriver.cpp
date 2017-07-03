#include <debug_config.h>

#define SERIAL_TRACE_LEVEL SENSOR_DRIVER_SERIAL_TRACE_LEVEL

#include "SensorDriver.h"

//------------------------------------------------------------------------------
// SensorDriver
//------------------------------------------------------------------------------

SensorDriver::SensorDriver(const char* driver, const char* type, bool *is_setted, bool *is_prepared) {
  _driver = driver;
  _type = type;
  _is_setted = is_setted;
  *is_setted = false;
  _is_prepared = is_prepared;
  *_is_prepared = false;
  _retry = 0;
}

SensorDriver *SensorDriver::create(const char* driver, const char* type, bool *is_setted, bool *is_prepared) {
  if (strlen(driver) == 0 || strlen(type) == 0) {
    SERIAL_ERROR("SensorDriver %s-%s create... [ FAIL ]\r\n--> driver or type is null.\r\n", driver, type);
    return NULL;
  }

  #if (USE_SENSOR_ADT)
  else if (strcmp(type, SENSOR_TYPE_ADT) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_HIH)
  else if (strcmp(type, SENSOR_TYPE_HIH) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_HYT)
  else if (strcmp(type, SENSOR_TYPE_HYT) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_HI7)
  else if (strcmp(type, SENSOR_TYPE_HI7) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_BMP)
  else if (strcmp(type, SENSOR_TYPE_BMP) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_DW1)
  else if (strcmp(type, SENSOR_TYPE_DW1) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_TBS || USE_SENSOR_TBR)
  else if (strcmp(type, SENSOR_TYPE_TBS) == 0 || strcmp(type, SENSOR_TYPE_TBR) == 0)
    return new SensorDriverRain(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_STH || USE_SENSOR_ITH || USE_SENSOR_MTH || USE_SENSOR_NTH || USE_SENSOR_XTH)
  else if (strcmp(type, SENSOR_TYPE_STH) == 0 || strcmp(type, SENSOR_TYPE_ITH) == 0 || strcmp(type, SENSOR_TYPE_MTH) == 0 || strcmp(type, SENSOR_TYPE_NTH) == 0 || strcmp(type, SENSOR_TYPE_XTH) == 0)
    return new SensorDriverTh(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_SSD)
  else if (strcmp(type, SENSOR_TYPE_SSD) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_ISD)
  else if (strcmp(type, SENSOR_TYPE_ISD) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_MSD)
  else if (strcmp(type, SENSOR_TYPE_MSD) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_NSD)
  else if (strcmp(type, SENSOR_TYPE_NSD) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  #if (USE_SENSOR_XSD)
  else if (strcmp(type, SENSOR_TYPE_XSD) == 0)
    return new SensorDriverHyt271(driver, type, is_setted, is_prepared);
  #endif

  else {
    SERIAL_ERROR("SensorDriver %s-%s create... [ FAIL ]\r\n--> driver or type not found.\r\n", driver, type);
    return NULL;
  }
}

void SensorDriver::setup(const uint8_t address, const uint8_t node) {
  _address = address;
  _node = node;
  _start_time_ms = 0;
}

const char *SensorDriver::getDriver() {
  return _driver;
}

const char *SensorDriver::getType() {
  return _type;
}

uint8_t SensorDriver::getAddress() {
  return _address;
}

uint8_t SensorDriver::getNode() {
  return _node;
}

uint32_t SensorDriver::getDelay() {
  return _delay_ms;
}

uint32_t SensorDriver::getStartTime() {
  return _start_time_ms;
}

bool SensorDriver::isPrepared() {
  return *_is_prepared;
}

void SensorDriver::resetPrepared() {
  *_is_prepared = false;
  _retry = 0;
}

bool SensorDriver::isEnd() {
  return _is_end;
}

bool SensorDriver::isSuccess() {
  return _is_success;
}

bool SensorDriver::isReaded() {
  return _is_readed;
}

void SensorDriver::createAndSetup(const char* driver, const char* type, bool *is_setted, bool *is_prepared, uint8_t address, SensorDriver *sensors[], uint8_t *sensors_count) {
  sensors[*sensors_count] = SensorDriver::create(driver, type, is_setted, is_prepared);
  if (sensors[*sensors_count]) {
    sensors[*sensors_count]->setup(address);
    (*sensors_count)++;
  }
}

#if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
void SensorDriver::printInfo(const char* driver, const char* type, const uint8_t address, const uint8_t node) {
  SERIAL_DEBUG("SensorDriver %s-%s", driver, type);

  if (address)
    SERIAL_DEBUG(" 0x%x (%d)", address, address);

  if (node)
    SERIAL_DEBUG(" on node %d", node);
}
#endif

//------------------------------------------------------------------------------
// HYT271
//------------------------------------------------------------------------------

#if (USE_SENSOR_HYT)
void SensorDriverHyt271::setup(const uint8_t address, const uint8_t node) {
  SensorDriver::setup(address, node);

  #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
  SensorDriver::printInfo(_driver, _type, _address, _node);
  #endif

  SERIAL_DEBUG(" setup... [ OK ]\r\n");
}

void SensorDriverHyt271::prepare() {
  #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
  SensorDriver::printInfo(_driver, _type, _address, _node);
  #endif

  if (!*_is_prepared) {
    *_is_prepared = true;
    _delay_ms = Hyt271::initRead(_address);
    SERIAL_DEBUG(" prepare... [ OK ]\r\n");
  }
  else {
    _delay_ms = 0;
    SERIAL_DEBUG(" prepare... [ YES ]\r\n");
  }

  _start_time_ms = millis();
  _get_state = INIT;
}

void SensorDriverHyt271::get(int32_t *values, uint8_t length) {
  static float humidity;
  static float temperature;

  switch (_get_state) {
    case INIT:
      _is_success = true;

      values[0] = UINT16_MAX;
      values[1] = UINT16_MAX;

      humidity = 0;
      temperature = 0;

      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = false;
      _get_state = READ;
      break;

    case READ:
      _is_success = Hyt271::read(_address, &humidity, &temperature);

      #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
      SensorDriver::printInfo(_driver, _type, _address, _node);

      if (_is_success)
        SERIAL_DEBUG(" get... [ OK ]\r\n");
      else SERIAL_DEBUG(" get... [ FAIL ]\r\n");

      #endif

      if (_is_success && length >= 1) {
        values[0] = humidity;

        if (values[0] >= SENSOR_DRIVER_HYT271_HUMIDITY_MIN && values[0] <= SENSOR_DRIVER_HYT271_HUMIDITY_MAX && _is_success)
          SERIAL_DEBUG("--> humidity: %u\r\n", values[0]);
        else {
          _is_success = false;
          values[0] = UINT16_MAX;
          SERIAL_DEBUG("--> humidity: ---\r\n");
        }
      }

      if (_is_success && length >= 2) {
        values[1] = SENSOR_DRIVER_C_TO_K + temperature * 100;

        if (values[1] >= SENSOR_DRIVER_HYT271_TEMPERATURE_MIN && values[1] <= SENSOR_DRIVER_HYT271_TEMPERATURE_MAX && _is_success)
          SERIAL_DEBUG("--> temperature: %u\r\n", values[1]);
        else {
          _is_success = false;
          values[1] = UINT16_MAX;
          SERIAL_DEBUG("--> temperature: ---\r\n");
        }
      }

      _start_time_ms = millis();

      #if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
      _delay_ms = 5;
      #else
      _delay_ms = 0;
      #endif

      _is_end = true;
      _is_readed = false;
      _get_state = END;
      break;

    case END:
      _delay_ms = 0;
      _is_end = true;
      _is_readed = true;
      _get_state = INIT;
      break;
  }
}

#if (USE_JSON)
void SensorDriverHyt271::getJson(int32_t *values, uint8_t length, char *json_buffer, size_t json_buffer_length) {
  StaticJsonBuffer<JSON_BUFFER_LENGTH> buffer;
  JsonObject &json = buffer.createObject();

  SensorDriverHyt271::get(values, length);

  if (_is_end && _is_success && values[0] != UINT16_MAX && !_is_readed && length >= 1)
    json["B13003"] = values[0];
  else json["B13003"] = (char*)NULL;

  if (_is_end && _is_success && values[1] != UINT16_MAX && !_is_readed && length >= 2)
    json["B12101"] = values[1];
  else json["B12101"] = (char*)NULL;

  json.printTo(json_buffer, json_buffer_length);
}
#endif

#endif

//------------------------------------------------------------------------------
// I2C-Rain
// TBS: oneshot
// TBR: oneshot
//------------------------------------------------------------------------------

#if (USE_SENSOR_TBS || USE_SENSOR_TBR)
void SensorDriverRain::setup(const uint8_t address, const uint8_t node) {
  SensorDriver::setup(address, node);

  #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
  SensorDriver::printInfo(_driver, _type, _address, _node);
  #endif

  SERIAL_DEBUG(" setup... [ OK ]\r\n");
}

void SensorDriverRain::prepare() {
  #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
  SensorDriver::printInfo(_driver, _type, _address, _node);
  #endif

  if (!*_is_prepared) {
    Wire.beginTransmission(_address);
    Wire.write(I2C_COMMAND_ID);

    if (strcmp(_type, SENSOR_TYPE_TBS) == 0 || strcmp(_type, SENSOR_TYPE_TBR) == 0) {
      Wire.write(I2C_RAIN_COMMAND_ONESHOT_START_STOP);
      _delay_ms = 0;
    }
    else {
      _delay_ms = 0;
      SERIAL_DEBUG(" prepare... [ FAIL ]\r\n");
      return;
    }

    if (Wire.endTransmission()) {
      SERIAL_DEBUG(" prepare... [ FAIL ]\r\n");
      return;
    }

    *_is_prepared = true;

    SERIAL_DEBUG(" prepare... [ OK ]\r\n");
  }
  else {
    SERIAL_DEBUG(" prepare... [ YES ]\r\n");
    _delay_ms = 0;
  }

  _start_time_ms = millis();
  _get_state = INIT;
}

void SensorDriverRain::get(int32_t *values, uint8_t length) {
  static uint8_t rain_data[I2C_RAIN_TIPS_LENGTH];
  uint8_t data_length;

  switch (_get_state) {
    case INIT:
      memset(values, UINT16_MAX, length);
      _is_success = true;
      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = false;

      if (_is_success && length >= 1)
        _get_state = SET_RAIN_ADDRESS;
      // else _get_state = END;
      break;

    case SET_RAIN_ADDRESS:
      Wire.beginTransmission(_address);

      if (strcmp(_type, SENSOR_TYPE_TBS) == 0 || strcmp(_type, SENSOR_TYPE_TBR) == 0) {
        Wire.write(I2C_RAIN_TIPS_ADDRESS);
        Wire.write(I2C_RAIN_TIPS_LENGTH);
      }
      else _is_success = false;

      if (Wire.endTransmission()) {
        _is_success = false;
        // _get_state = END;
      }

      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = false;

      if (_is_success)
        _get_state = READ_RAIN;
      // else _get_state = END;
      break;

    case READ_RAIN:
      if (strcmp(_type, SENSOR_TYPE_TBS) == 0 || strcmp(_type, SENSOR_TYPE_TBR) == 0)
        data_length = I2C_RAIN_TIPS_LENGTH;
      else _is_success = false;

      Wire.requestFrom(_address, data_length);

      if (Wire.available() < data_length)
        _is_success = false;

      if (_is_success) {
        for (uint8_t i=0; i<data_length; i++)
          rain_data[i] = Wire.read();
      }

      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = false;

      if (_is_success)
        _get_state = END;
      // else _get_state = END;
      break;

    case END:
      #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
      SensorDriver::printInfo(_driver, _type, _address, _node);

      if (_is_success)
        SERIAL_DEBUG(" get... [ OK ]\r\n");
      else SERIAL_DEBUG(" get... [ FAIL ]");
      #endif

      if (length >= 1) {
        values[0] = (uint16_t)(rain_data[1] << 8) | (rain_data[0]);

        if (_is_success && values[0] >= SENSOR_DRIVER_RAIN_MIN && values[0] <= SENSOR_DRIVER_RAIN_MAX)
          SERIAL_DEBUG("--> rain tips: %u\r\n", values[0]);
        else {
          _is_success = false;
          values[0] = UINT16_MAX;
          SERIAL_DEBUG("--> rain tips: ---\r\n");
        }
      }

      _start_time_ms = millis();
      _delay_ms = 20;
      _is_end = true;
      _is_readed = false;
      break;
    }
}

#if (USE_JSON)
void SensorDriverRain::getJson(int32_t *values, uint8_t length, char *json_buffer, size_t json_buffer_length) {
  StaticJsonBuffer<JSON_BUFFER_LENGTH> buffer;
  JsonObject &json = buffer.createObject();

  SensorDriverRain::get(values, length);

  if (_is_end && _is_success && values[0] != UINT16_MAX && !_is_readed && length >= 1)
    json["B13011"] = values[0];
  else json["B13011"] = (char*)NULL;

  json.printTo(json_buffer, json_buffer_length);
}
#endif

#endif

//------------------------------------------------------------------------------
// I2C-TH
// STH: oneshot
// ITH: continuous istantaneous
// MTH: continuous average
// NTH: continuous min
// XTH: continuous max
//------------------------------------------------------------------------------

#if (USE_SENSOR_STH || USE_SENSOR_ITH || USE_SENSOR_MTH || USE_SENSOR_NTH || USE_SENSOR_XTH)
void SensorDriverTh::setup(const uint8_t address, const uint8_t node) {
  SensorDriver::setup(address, node);

  #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
  SensorDriver::printInfo(_driver, _type, _address, _node);
  #endif

  if (!*_is_setted) {
    Wire.beginTransmission(_address);
    Wire.write(I2C_COMMAND_ID);

    if (strcmp(_type, SENSOR_TYPE_ITH) == 0 || strcmp(_type, SENSOR_TYPE_MTH) == 0 || strcmp(_type, SENSOR_TYPE_NTH) == 0 || strcmp(_type, SENSOR_TYPE_XTH) == 0)
      Wire.write(I2C_TH_COMMAND_CONTINUOUS_START);
    else {
      SERIAL_DEBUG(" setup... [ FAIL ]\r\n");
      return;
    }

    if (Wire.endTransmission()) {
      SERIAL_DEBUG(" setup... [ FAIL ]\r\n");
      return;
    }

    *_is_setted = true;

    SERIAL_DEBUG(" setup... [ OK ]\r\n");
  }
  else {
    SERIAL_DEBUG(" setup... [ YES ]\r\n");
  }
}

void SensorDriverTh::prepare() {
  #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
  SensorDriver::printInfo(_driver, _type, _address, _node);
  #endif

  if (!*_is_prepared) {
    Wire.beginTransmission(_address);
    Wire.write(I2C_COMMAND_ID);

    if (strcmp(_type, SENSOR_TYPE_STH) == 0) {
      Wire.write(I2C_TH_COMMAND_ONESHOT_START_STOP);
      _delay_ms = 150;
    }
    else if (strcmp(_type, SENSOR_TYPE_ITH) == 0 || strcmp(_type, SENSOR_TYPE_MTH) == 0 || strcmp(_type, SENSOR_TYPE_NTH) == 0 || strcmp(_type, SENSOR_TYPE_XTH) == 0) {
      Wire.write(I2C_TH_COMMAND_CONTINUOUS_START_STOP);
      _delay_ms = 0;
    }
    else {
      SERIAL_DEBUG(" prepare... [ FAIL ]\r\n");
      return;
    }

    if (Wire.endTransmission()) {
      SERIAL_DEBUG(" prepare... [ FAIL ]\r\n");
      return;
    }

    *_is_prepared = true;

    SERIAL_DEBUG(" prepare... [ OK ]\r\n");
  }
  else {
    SERIAL_DEBUG(" prepare... [ YES ]\r\n");
    _delay_ms = 0;
  }

  _start_time_ms = millis();
  _get_state = INIT;
}

void SensorDriverTh::get(int32_t *values, uint8_t length) {
  static uint8_t temperature_data[I2C_TH_TEMPERATURE_DATA_MAX_LENGTH];
  static uint8_t humidity_data[I2C_TH_HUMIDITY_DATA_MAX_LENGTH];
  uint8_t data_length;

  switch (_get_state) {
    case INIT:
      memset(values, UINT16_MAX, length);
      _is_success = true;
      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = false;

      if (_is_success && length >= 1)
        _get_state = SET_TEMPERATURE_ADDRESS;
      // else _get_state = END;
      break;

    case SET_TEMPERATURE_ADDRESS:
      // SERIAL_INFO("aa");
      Wire.beginTransmission(_address);

      if (strcmp(_type, SENSOR_TYPE_STH) == 0) {
        Wire.write(I2C_TH_TEMPERATURE_SAMPLE_ADDRESS);
        Wire.write(I2C_TH_TEMPERATURE_SAMPLE_LENGTH);
      }
      else if (strcmp(_type, SENSOR_TYPE_ITH) == 0) {
        Wire.write(I2C_TH_TEMPERATURE_MED60_ADDRESS);
        Wire.write(I2C_TH_TEMPERATURE_MED60_LENGTH);
      }
      else if (strcmp(_type, SENSOR_TYPE_MTH) == 0) {
        Wire.write(I2C_TH_TEMPERATURE_MED_ADDRESS);
        Wire.write(I2C_TH_TEMPERATURE_MED_LENGTH);
      }
      else if (strcmp(_type, SENSOR_TYPE_NTH) == 0) {
        Wire.write(I2C_TH_TEMPERATURE_MIN_ADDRESS);
        Wire.write(I2C_TH_TEMPERATURE_MIN_LENGTH);
      }
      else if (strcmp(_type, SENSOR_TYPE_XTH) == 0) {
        Wire.write(I2C_TH_TEMPERATURE_MAX_ADDRESS);
        Wire.write(I2C_TH_TEMPERATURE_MAX_LENGTH);
      }
      else _is_success = false;

      if (Wire.endTransmission())
        _is_success = false;

      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = false;

      // SERIAL_INFO("TA %u ",_is_success);

      if (_is_success)
        _get_state = READ_TEMPERATURE;
      // else _get_state = END;
      break;

    case READ_TEMPERATURE:

      if (strcmp(_type, SENSOR_TYPE_STH) == 0)
        data_length = I2C_TH_TEMPERATURE_SAMPLE_LENGTH;
      else if (strcmp(_type, SENSOR_TYPE_ITH) == 0)
        data_length = I2C_TH_TEMPERATURE_MED60_LENGTH;
      else if (strcmp(_type, SENSOR_TYPE_MTH) == 0)
        data_length = I2C_TH_TEMPERATURE_MED_LENGTH;
      else if (strcmp(_type, SENSOR_TYPE_NTH) == 0)
        data_length = I2C_TH_TEMPERATURE_MIN_LENGTH;
      else if (strcmp(_type, SENSOR_TYPE_XTH) == 0)
        data_length = I2C_TH_TEMPERATURE_MAX_LENGTH;
      else _is_success = false;

      Wire.requestFrom(_address, data_length);

      if (Wire.available() < data_length)
        _is_success = false;

      if (_is_success) {
        for (uint8_t i=0; i<data_length; i++)
          temperature_data[i] = Wire.read();
      }

      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = false;

      // SERIAL_INFO("RT %u ",_is_success);

      if (_is_success && length >= 2)
        _get_state = SET_HUMIDITY_ADDRESS;
      else if (_is_success && length >= 1)
        _get_state = END;
      // else _get_state = END;
      break;

    case SET_HUMIDITY_ADDRESS:
      Wire.beginTransmission(_address);

      if (strcmp(_type, SENSOR_TYPE_STH) == 0) {
        Wire.write(I2C_TH_HUMIDITY_SAMPLE_ADDRESS);
        Wire.write(I2C_TH_HUMIDITY_SAMPLE_LENGTH);
      }
      else if (strcmp(_type, SENSOR_TYPE_ITH) == 0) {
        Wire.write(I2C_TH_HUMIDITY_MED60_ADDRESS);
        Wire.write(I2C_TH_HUMIDITY_MED60_LENGTH);
      }
      else if (strcmp(_type, SENSOR_TYPE_MTH) == 0) {
        Wire.write(I2C_TH_HUMIDITY_MED_ADDRESS);
        Wire.write(I2C_TH_HUMIDITY_MED_LENGTH);
      }
      else if (strcmp(_type, SENSOR_TYPE_NTH) == 0) {
        Wire.write(I2C_TH_HUMIDITY_MIN_ADDRESS);
        Wire.write(I2C_TH_HUMIDITY_MIN_LENGTH);
      }
      else if (strcmp(_type, SENSOR_TYPE_XTH) == 0) {
        Wire.write(I2C_TH_HUMIDITY_MAX_ADDRESS);
        Wire.write(I2C_TH_HUMIDITY_MAX_LENGTH);
      }
      else _is_success = false;

      if (Wire.endTransmission())
        _is_success = false;

      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = false;

      // SERIAL_INFO("HA %u ",_is_success);

      if (_is_success)
        _get_state = READ_HUMIDITY;
      // else _get_state = END;
      break;

    case READ_HUMIDITY:
      if (strcmp(_type, SENSOR_TYPE_STH) == 0)
        data_length = I2C_TH_HUMIDITY_SAMPLE_LENGTH;
      else if (strcmp(_type, SENSOR_TYPE_ITH) == 0)
        data_length = I2C_TH_HUMIDITY_MED60_LENGTH;
      else if (strcmp(_type, SENSOR_TYPE_MTH) == 0)
        data_length = I2C_TH_HUMIDITY_MED_LENGTH;
      else if (strcmp(_type, SENSOR_TYPE_NTH) == 0)
        data_length = I2C_TH_HUMIDITY_MIN_LENGTH;
      else if (strcmp(_type, SENSOR_TYPE_XTH) == 0)
        data_length = I2C_TH_HUMIDITY_MAX_LENGTH;
      else _is_success = false;

      Wire.requestFrom(_address, data_length);

      if (Wire.available() < data_length)
        _is_success = false;

      if (_is_success) {
        for (uint8_t i=0; i<data_length; i++)
          humidity_data[i] = Wire.read();
      }

      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = false;

      // SERIAL_INFO("RH %u ",_is_success);

      if (_is_success)
        _get_state = END;
      // else _get_state = END;
      break;

    case END:
      #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
      SensorDriver::printInfo(_driver, _type, _address, _node);

      if (_is_success)
        SERIAL_DEBUG(" get... [ OK ]\r\n");
      else SERIAL_DEBUG(" get... [ FAIL ]\r\n");
      #endif

      if (length >= 1) {
        values[0] = (uint16_t)(temperature_data[1] << 8) | (temperature_data[0]);
        // SERIAL_INFO("%s %u ", _type, values[0]);

        if (_is_success && values[0] >= SENSOR_DRIVER_TEMPERATURE_MIN && values[0] <= SENSOR_DRIVER_TEMPERATURE_MAX)
          SERIAL_DEBUG("--> temperature: %u\r\n", values[0]);
        else {
          _is_success = false;
          values[0] = UINT16_MAX;
          SERIAL_DEBUG("--> temperature: ---\r\n");
        }
      }

      if (length >= 2) {
        values[1] = (uint16_t)(humidity_data[1] << 8) | (humidity_data[0]);

        if (_is_success && values[1] >= SENSOR_DRIVER_HUMIDITY_MIN && values[1] <= SENSOR_DRIVER_HUMIDITY_MAX)
          SERIAL_DEBUG("--> humidity: %u\r\n", values[1]);
        else {
          _is_success = false;
          values[1] = UINT16_MAX;
          SERIAL_DEBUG("--> humidity: ---\r\n");
        }
      }

      // SERIAL_INFO("R %u %s\r\n",_is_success, _type);

      _start_time_ms = millis();
      _delay_ms = 0;
      _is_end = true;
      _is_readed = false;
      break;
  }
}

#if (USE_JSON)
void SensorDriverTh::getJson(int32_t *values, uint8_t length, char *json_buffer, size_t json_buffer_length) {
  StaticJsonBuffer<JSON_BUFFER_LENGTH> buffer;
  JsonObject &json = buffer.createObject();

  SensorDriverTh::get(values, length);

  if (_is_end && _is_success && values[0] != UINT16_MAX && !_is_readed && length >= 1)
    json["B12101"] = values[0];
  else json["B12101"] = (char*)NULL;

  if (_is_end && _is_success && values[1] != UINT16_MAX && !_is_readed && length >= 2)
    json["B13003"] = values[0];
  else json["B13003"] = (char*)NULL;

  json.printTo(json_buffer, json_buffer_length);
}
#endif

#endif
