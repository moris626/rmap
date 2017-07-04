#ifndef _TYPEDEF_H
#define _TYPEDEF_H

typedef struct {
  char driver[5];
  char type[5];
  uint8_t address;
  uint8_t node;
  char mqtt_path[MQTT_SENSOR_PATH_LENGTH];
} sensor_t;

/*!
  \typedef
  \struct rain_t
  Rain data.
*/
typedef struct {
  uint16_t tips_count;  //!< rain gauge tips counter
} rain_t;

/*!
  \typedef
  \struct values_t
  Values for report
*/
typedef struct {
  uint16_t sample; // last sample
  uint16_t med60;  // last observation
  uint16_t med;    // average values of observations
  uint16_t max;    // max values of observations
  uint16_t min;    // minium values of observations
  uint16_t sigma;  // standard deviation of observations
} value_t;

#endif
