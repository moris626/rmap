#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#define USE_JSON                    (true)   // use ajson library for json response

#define USE_SENSOR_ADT              (false)   // ADT7420
#define USE_SENSOR_HIH              (false)   // HIH6100
#define USE_SENSOR_HYT              (false)   // HYT271
#define USE_SENSOR_HI7              (false)   // SI7021
#define USE_SENSOR_BMP              (false)   // Bmp085
#define USE_SENSOR_DW1              (false)   // DW1
#define USE_SENSOR_TBS              (true)   // Tipping bucket rain gauge
#define USE_SENSOR_TBR              (false)   // Tipping bucket rain gauge
#define USE_SENSOR_STH              (false)   // Temperature and humidity oneshot
#define USE_SENSOR_ITH              (true)   // Temperature and humidity continuous istantaneous
#define USE_SENSOR_NTH              (true)   // Temperature and humidity continuous minium
#define USE_SENSOR_MTH              (true)   // Temperature and humidity continuous average
#define USE_SENSOR_XTH              (true)   // Temperature and humidity continuous maximum
#define USE_SENSOR_SSD              (false)   // SSD011 oneshot
#define USE_SENSOR_ISD              (false)   // SSD011 report istantaneous
#define USE_SENSOR_NSD              (false)   // SSD011 report minium
#define USE_SENSOR_MSD              (false)   // SSD011 report average
#define USE_SENSOR_XSD              (false)   // SSD011 report maximum
#define USE_SENSOR_SMI              (false)   // MICS4514 oneshot
#define USE_SENSOR_IMI              (false)   // MICS4514 report istantaneous
#define USE_SENSOR_NMI              (false)   // MICS4514 report minium
#define USE_SENSOR_MMI              (false)   // MICS4514 report average
#define USE_SENSOR_XMI              (false)   // MICS4514 report maximum
#define USE_SENSOR_RF24             (false)   // Radio RF24

#define RAIN_FOR_TIP                (2)

// sampling every 3-15 seconds --> watchdog timer (SAMPLE_SECONDS in relative modules)
// observations with processing every 1-10 minutes (minutes for processing sampling)
// report every 5-60 minutes (> OBSERVATIONS_MINUTES)
#define OBSERVATIONS_MINUTES        (1) // every 1-10 minutes (minutes for processing samples)
#define REPORT_MINUTES              (1) // every 5-60 minutes (minutes for report. > n * OBSERVATIONS_MINUTES)

#if ((REPORT_MINUTES % OBSERVATIONS_MINUTES) != 0)
#error OBSERVATIONS_MINUTES must be multiple of REPORT_MINUTES !!!
#endif

#endif
