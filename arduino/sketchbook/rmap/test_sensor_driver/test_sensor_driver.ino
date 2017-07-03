#include <debug_config.h>

#define SERIAL_TRACE_LEVEL SERIAL_TRACE_LEVEL_INFO

#include <Wire.h>
#include <SensorDriver.h>
// #include <ArduinoJson.h>
#include <typedef.h>

#define SENSORS_RETRY_COUNT_MAX     (5)
#define I2C_BUS_CLOCK               (50000L)
#define CLOCK_OFFSET                (10)

uint8_t sensors_count = 0;
SensorDriver *sensors[5];

bool is_first_run = true;
bool is = false;

#define SENSORS_RETRY_DELAY_MS (50)
#define VALUES_TO_READ_FROM_SENSOR_COUNT (2)

value_t temperature;
value_t humidity;
rain_t rain;

typedef enum {
  INIT_SENSOR,
  PREPARE_SENSOR,
  IS_SENSOR_PREPARED,
  GET_SENSOR,
  IS_SENSOR_GETTED,
  READ_SENSOR,
  END_SENSOR_READING,
  END_TASK,
  END_SENSOR,
  WAIT_STATE
} sensor_state_t;

sensor_state_t sensor_state;

bool is_sensors_th_prepared;
bool is_sensors_th_setted;
bool is_sensors_rain_prepared;
bool is_sensors_rain_setted;
bool is_sensors_hyt_prepared;
bool is_sensors_hyt_setted;

uint32_t acquiring_sensors_delay_ms;

void init_sensors () {
  sensors_count = 0;

  SERIAL_INFO("Sensors...\r\n");

  #if (USE_SENSOR_HYT)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_HYT, &is_sensors_hyt_setted, &is_sensors_hyt_prepared, 0x28, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_HYT);
  #endif

  #if (USE_SENSOR_TBS)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_TBS, &is_sensors_rain_setted, &is_sensors_rain_prepared, 0x21, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_TBS);
  #endif

  #if (USE_SENSOR_TBR)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_TBR, &is_sensors_rain_setted, &is_sensors_rain_prepared, 0x21, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_TBR);
  #endif

  #if (USE_SENSOR_ITH)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_ITH, &is_sensors_th_setted, &is_sensors_th_prepared, 0x23, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_ITH);
  #endif

  #if (USE_SENSOR_MTH)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_MTH, &is_sensors_th_setted, &is_sensors_th_prepared, 0x23, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_MTH);
  #endif

  #if (USE_SENSOR_NTH)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_NTH, &is_sensors_th_setted, &is_sensors_th_prepared, 0x23, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_NTH);
  #endif

  #if (USE_SENSOR_XTH)
  SensorDriver::createAndSetup(SENSOR_DRIVER_I2C, SENSOR_TYPE_XTH, &is_sensors_th_setted, &is_sensors_th_prepared, 0x23, sensors, &sensors_count);
  SERIAL_INFO("--> %u: %s-%s\r\n", sensors_count, SENSOR_DRIVER_I2C, SENSOR_TYPE_XTH);
  #endif

  SERIAL_INFO("\r\n");
}

void sensors_reading_task () {
  static uint8_t med60_count;
  static uint8_t min_count;
  static uint8_t med_count;
  static uint8_t max_count;
  static uint8_t rain_count;

  static uint8_t i;
  static uint8_t retry;
  static sensor_state_t state_after_wait;
  static uint32_t delay_ms;
  static uint32_t start_time_ms;
  static int32_t values_readed_from_sensor[2];

  switch (sensor_state) {
    case INIT_SENSOR:
      for (i=0; i<sensors_count; i++)
        sensors[i]->resetPrepared();

      i = 0;
      retry = 0;
      memset(&temperature, UINT16_MAX, sizeof(value_t));
      memset(&humidity, UINT16_MAX, sizeof(value_t));
      memset(&rain, UINT16_MAX, sizeof(rain_t));
      state_after_wait = INIT_SENSOR;
      sensor_state = PREPARE_SENSOR;
      break;

    case PREPARE_SENSOR:
      sensors[i]->prepare();
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();
      state_after_wait = IS_SENSOR_PREPARED;
      sensor_state = WAIT_STATE;
      break;

    case IS_SENSOR_PREPARED:
      // success
      if (sensors[i]->isPrepared()) {
        sensor_state = GET_SENSOR;
      }
      // retry
      else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = PREPARE_SENSOR;
        sensor_state = WAIT_STATE;
      }
      // fail
      else sensor_state = END_SENSOR_READING;
      break;

    case GET_SENSOR:
      sensors[i]->get(values_readed_from_sensor, VALUES_TO_READ_FROM_SENSOR_COUNT);
      delay_ms = sensors[i]->getDelay();
      start_time_ms = sensors[i]->getStartTime();
      state_after_wait = IS_SENSOR_GETTED;
      sensor_state = WAIT_STATE;
      break;

    case IS_SENSOR_GETTED:
      // success and end
      if (sensors[i]->isEnd() && !sensors[i]->isReaded() && sensors[i]->isSuccess()) {
        sensor_state = READ_SENSOR;
      }
      // success and not end
      else if (sensors[i]->isSuccess()) {
        sensor_state = GET_SENSOR;
      }
      // retry
      else if ((++retry) < SENSORS_RETRY_COUNT_MAX) {
        delay_ms = SENSORS_RETRY_DELAY_MS;
        start_time_ms = millis();
        state_after_wait = GET_SENSOR;
        sensor_state = WAIT_STATE;
      }
      // fail
      else sensor_state = END_SENSOR_READING;
      break;

    case READ_SENSOR:
      #if (USE_SENSOR_ITH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_ITH) == 0) {
        temperature.med60 = values_readed_from_sensor[0];
        humidity.med60 = values_readed_from_sensor[1];
        med60_count = retry;
      }
      #endif

      #if (USE_SENSOR_MTH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_MTH) == 0) {
        temperature.med = values_readed_from_sensor[0];
        humidity.med = values_readed_from_sensor[1];
        med_count = retry;
      }
      #endif

      #if (USE_SENSOR_NTH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_NTH) == 0) {
        temperature.min = values_readed_from_sensor[0];
        humidity.min = values_readed_from_sensor[1];
        min_count = retry;
      }
      #endif

      #if (USE_SENSOR_XTH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_XTH) == 0) {
        temperature.max = values_readed_from_sensor[0];
        humidity.max = values_readed_from_sensor[1];
        max_count = retry;
      }
      #endif

      #if (USE_SENSOR_TBS || USE_SENSOR_TBR)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_TBS) == 0 || strcmp(sensors[i]->getType(), SENSOR_TYPE_TBR) == 0) {
        rain.tips_count = values_readed_from_sensor[0];
        rain_count = retry;
      }
      #endif
      sensor_state = END_SENSOR_READING;
      break;

    case END_SENSOR_READING:
      #if (USE_SENSOR_ITH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_ITH) == 0) {
        med60_count = retry;
      }
      #endif

      #if (USE_SENSOR_MTH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_MTH) == 0) {
        med_count = retry;
      }
      #endif

      #if (USE_SENSOR_NTH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_NTH) == 0) {
        min_count = retry;
      }
      #endif

      #if (USE_SENSOR_XTH)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_XTH) == 0) {
        max_count = retry;
      }
      #endif

      #if (USE_SENSOR_TBS || USE_SENSOR_TBR)
      if (strcmp(sensors[i]->getType(), SENSOR_TYPE_TBS) == 0 || strcmp(sensors[i]->getType(), SENSOR_TYPE_TBR) == 0) {
        rain_count = retry;
      }
      #endif

      // SERIAL_INFO("%u ", retry);
      // next sensor
      if ((++i) < sensors_count) {
        retry = 0;
        delay_ms = 100;
        start_time_ms = millis();
        state_after_wait = PREPARE_SENSOR;
        sensor_state = WAIT_STATE;
        // sensor_state = PREPARE_SENSOR;
      }
      // end
      else {
        if (is_first_run) {
          SERIAL_INFO("x TIME\t- T-IST\tH-IST\t- T-MIN\tH-MIN\t- T-MED\tH-MED\t- T-MAX\tH-MAX\t- R-TPS\r\n");
          is_first_run = false;
        }
        else {
          #if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_INFO)
          SERIAL_INFO("x %u\t", millis()/1000);

          if (temperature.med60 != UINT16_MAX)
            SERIAL_INFO("%u %u\t", med60_count, temperature.med60);
          else SERIAL_INFO("%u -----\t", med60_count);

          if (humidity.med60 != UINT16_MAX)
            SERIAL_INFO("%u\t", humidity.med60);
          else SERIAL_INFO("-----\t");

          if (temperature.min != UINT16_MAX)
            SERIAL_INFO("%u %u\t", min_count, temperature.min);
          else SERIAL_INFO("%u -----\t", min_count);

          if (humidity.min != UINT16_MAX)
            SERIAL_INFO("%u\t", humidity.min);
          else SERIAL_INFO("-----\t");

          if (temperature.med != UINT16_MAX)
            SERIAL_INFO("%u %u\t", med_count, temperature.med);
          else SERIAL_INFO("%u -----\t", med_count);

          if (humidity.med != UINT16_MAX)
            SERIAL_INFO("%u\t", humidity.med);
          else SERIAL_INFO("-----\t");

          if (temperature.max != UINT16_MAX)
            SERIAL_INFO("%u %u\t", max_count, temperature.max);
          else SERIAL_INFO("%u -----\t", max_count);

          if (humidity.max != UINT16_MAX)
            SERIAL_INFO("%u\t", humidity.max);
          else SERIAL_INFO("-----\t");

          if (rain.tips_count != UINT16_MAX)
            SERIAL_INFO("%u %u\r\n", rain_count, rain.tips_count);
          else SERIAL_INFO("%u -----\r\n", rain_count);
          #endif
        }

        #if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_INFO)
        delay_ms = 10;
        start_time_ms = millis();
        state_after_wait = END_TASK;
        sensor_state = WAIT_STATE;
        #else
        sensor_state = END_TASK;
        #endif
      }
      break;

    case END_TASK:
      sensor_state = END_SENSOR;
      break;

    case END_SENSOR:
      break;

    case WAIT_STATE:
      if (millis() - start_time_ms > delay_ms) {
        sensor_state = state_after_wait;
      }
      break;
  }
}

void setup() {
  TRACE_BEGIN(230400);
  Wire.begin();
  Wire.setClock(I2C_BUS_CLOCK);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  init_sensors();
  acquiring_sensors_delay_ms = 0;
  sensor_state = INIT_SENSOR;
}

void loop() {
  sensors_reading_task();

  if (millis() - acquiring_sensors_delay_ms >= 5000) {
    acquiring_sensors_delay_ms = millis();
    sensor_state = INIT_SENSOR;
  }
}
