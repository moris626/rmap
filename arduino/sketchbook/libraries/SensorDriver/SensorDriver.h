/*
SensorDriver.h - Library for read sensor.
Created by Paolo Patruno , November 30, 2013.
Released into the GPL licenze.
*/

#ifndef SensorDriver_h
#define SensorDriver_h

#include <debug.h>
#include "SensorDriverSensors.h"
#include <sensors_config.h>
#include <Arduino.h>
#include <Wire.h>

#define SENSOR_DRIVER_ERROR       (1)
#define SENSOR_DRIVER_SUCCESS     (0)

#define SENSOR_DRIVER_C_TO_K      (27315l)

#define MAX_DELAY_FOR_READ_MS     (60000)   // this the value for http and report

#if (USE_JSON)
#include <ArduinoJson.h>
#define JSON_BUFFER_LENGTH        (50)
#endif

class SensorDriver {
public:
  static SensorDriver *create(const char* driver, const char* type, bool *is_setted, bool *is_prepared);
  static void createAndSetup(const char* driver, const char* type, bool *is_setted, bool *is_prepared, uint8_t address, SensorDriver *sensors[], uint8_t *sensors_count);
  SensorDriver(const char* driver, const char* type, bool *is_setted, bool *is_prepared);
  virtual void setup(const uint8_t address, const uint8_t node = 0);
  virtual void prepare();
  virtual void get(int32_t *values, uint8_t length);

  #if (USE_JSON)
  virtual void getJson(int32_t *values, uint8_t length, char *json_buffer, size_t json_buffer_length = JSON_BUFFER_LENGTH);
  #endif

  // virtual ~SensorDriver();
  const char *getDriver();
  const char *getType();
  uint8_t getAddress();
  uint8_t getNode();
  uint32_t getStartTime();
  uint32_t getDelay();
  int32_t getValues();
  bool isPrepared();
  void resetPrepared();
  bool isEnd();
  bool isSuccess();
  bool isReaded();
protected:
  const char* _driver;
  const char* _type;
  uint8_t _address;
  int8_t _node;
  uint32_t _delay_ms;
  uint32_t _start_time_ms;
  int32_t values[];
  bool *_is_setted;
  bool *_is_prepared;
  bool _is_end;
  bool _is_success;
  bool _is_readed;

  #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
  static void printInfo(const char* driver, const char* type, const uint8_t address = 0, const uint8_t node = 0);
  #endif
};

#if (USE_SENSOR_HYT)
#include "hyt271.h"
#define SENSOR_DRIVER_HYT271_HUMIDITY_MIN       (HYT271_HUMIDITY_MIN)
#define SENSOR_DRIVER_HYT271_HUMIDITY_MAX       (HYT271_HUMIDITY_MAX)
#define SENSOR_DRIVER_HYT271_TEMPERATURE_MIN    (SENSOR_DRIVER_C_TO_K + (HYT271_TEMPERATURE_MIN * 100))
#define SENSOR_DRIVER_HYT271_TEMPERATURE_MAX    (SENSOR_DRIVER_C_TO_K + (HYT271_TEMPERATURE_MAX * 100))
class SensorDriverHyt271 : public SensorDriver {
public:
  SensorDriverHyt271(const char* driver, const char* type, bool *is_setted, bool *is_prepared) : SensorDriver(driver, type, is_setted, is_prepared) {
    #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
    SensorDriver::printInfo(driver, type);
    SERIAL_DEBUG(" create... [ OK ]\r\n");
    #endif
  };
  void setup(const uint8_t address, const uint8_t node = 0);
  void prepare();
  void get(int32_t *values, uint8_t length);

  #if (USE_JSON)
  void getJson(int32_t *values, uint8_t length, char *json_buffer, size_t json_buffer_length = JSON_BUFFER_LENGTH);
  #endif

protected:
  enum {
    INIT,
    READ,
    END
  } _get_state;
};
#endif

#if (USE_SENSOR_TBS || USE_SENSOR_TBR)
#include "registers-rain.h"
#define SENSOR_DRIVER_RAIN_MIN      (0)
#define SENSOR_DRIVER_RAIN_MAX      (300)
class SensorDriverRain : public SensorDriver {
public:
  SensorDriverRain(const char* driver, const char* type, bool *is_setted, bool *is_prepared) : SensorDriver(driver, type, is_setted, is_prepared) {
    #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
    SensorDriver::printInfo(driver, type);
    SERIAL_DEBUG(" create... [ OK ]\r\n");
    #endif
  };
  void setup(const uint8_t address, const uint8_t node = 0);
  void prepare();
  void get(int32_t *values, uint8_t length);

  #if (USE_JSON)
  void getJson(int32_t *values, uint8_t length, char *json_buffer, size_t json_buffer_length = JSON_BUFFER_LENGTH);
  #endif

protected:
  enum {
    INIT,
    SET_RAIN_ADDRESS,
    READ_RAIN,
    READ,
    END
  } _get_state;
};
#endif

#if (USE_SENSOR_STH || USE_SENSOR_ITH || USE_SENSOR_MTH || USE_SENSOR_NTH || USE_SENSOR_XTH)
#include "registers-th.h"
#define SENSOR_DRIVER_TEMPERATURE_MIN       (SENSOR_DRIVER_C_TO_K + (-50 * 100))
#define SENSOR_DRIVER_TEMPERATURE_MAX       (SENSOR_DRIVER_C_TO_K + (130 * 100))
#define SENSOR_DRIVER_HUMIDITY_MIN          (0)
#define SENSOR_DRIVER_HUMIDITY_MAX          (100)
class SensorDriverTh : public SensorDriver {
public:
  SensorDriverTh(const char* driver, const char* type, bool *is_setted, bool *is_prepared) : SensorDriver(driver, type, is_setted, is_prepared) {
    #if (SERIAL_TRACE_LEVEL == SERIAL_TRACE_LEVEL_DEBUG)
    SensorDriver::printInfo(driver, type);
    SERIAL_DEBUG(" create... [ OK ]\r\n");
    #endif
  };
  void setup(const uint8_t address, const uint8_t node = 0);
  void prepare();
  void get(int32_t *values, uint8_t length);

  #if (USE_JSON)
  void getJson(int32_t *values, uint8_t length, char *json_buffer, size_t json_buffer_length = JSON_BUFFER_LENGTH);
  #endif

protected:
  enum {
    INIT,
    SET_TEMPERATURE_ADDRESS,
    READ_TEMPERATURE,
    SET_HUMIDITY_ADDRESS,
    READ_HUMIDITY,
    READ,
    END
  } _get_state;
};
#endif

#endif
