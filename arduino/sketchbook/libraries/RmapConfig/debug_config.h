#ifndef _DEBUG_CONFIG_H
#define _DEBUG_CONFIG_H

// Serial debug level definitions
#define SERIAL_TRACE_LEVEL_OFF            (0)
#define SERIAL_TRACE_LEVEL_ERROR          (1)
#define SERIAL_TRACE_LEVEL_WARNING        (2)
#define SERIAL_TRACE_LEVEL_INFO           (3)
#define SERIAL_TRACE_LEVEL_DEBUG          (4)

// LCD debug level definitions
#define LCD_TRACE_LEVEL_OFF               (0)
#define LCD_TRACE_LEVEL_ERROR             (1)
#define LCD_TRACE_LEVEL_WARNING           (2)
#define LCD_TRACE_LEVEL_INFO              (3)
#define LCD_TRACE_LEVEL_DEBUG             (4)

// SensorDriver
#define SENSOR_DRIVER_SERIAL_TRACE_LEVEL  (SERIAL_TRACE_LEVEL_OFF)

// I2C-TH
#define I2C_TH_SERIAL_TRACE_LEVEL         (SERIAL_TRACE_LEVEL_INFO)

// I2C-RAIN
#define I2C_RAIN_SERIAL_TRACE_LEVEL       (SERIAL_TRACE_LEVEL_INFO)

// rmap
#define RMAP_SERIAL_TRACE_LEVEL           (SERIAL_TRACE_LEVEL_INFO)

#endif
