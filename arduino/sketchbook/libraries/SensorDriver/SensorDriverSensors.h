#ifndef SensorDriverSensors_h
#define SensorDriverSensors_h

#define SENSOR_DRIVER_I2C         ("I2C")

#define SENSOR_TYPE_ADT           ("ADT")   // ADT7420
#define SENSOR_TYPE_HIH           ("HIH")   // HIH6100
#define SENSOR_TYPE_HYT           ("HYT")   // HYT271
#define SENSOR_TYPE_HI7           ("HI7")   // SI7021
#define SENSOR_TYPE_BMP           ("BMP")   // Bmp085
#define SENSOR_TYPE_DW1           ("DW1")   // DW1
#define SENSOR_TYPE_TBS           ("TBS")   // Tipping bucket rain gauge
#define SENSOR_TYPE_TBR           ("TBR")   // Tipping bucket rain gauge
#define SENSOR_TYPE_STH           ("STH")   // Temperature and humidity oneshot
#define SENSOR_TYPE_ITH           ("ITH")   // Temperature and humidity oneshot report istantaneous
#define SENSOR_TYPE_MTH           ("MTH")   // Temperature and humidity oneshot report average
#define SENSOR_TYPE_NTH           ("NTH")   // Temperature and humidity oneshot report minium
#define SENSOR_TYPE_XTH           ("XTH")   // Temperature and humidity oneshot report maximum
#define SENSOR_TYPE_SSD           ("SSD")   // SSD011 oneshot
#define SENSOR_TYPE_ISD           ("ISD")   // SSD011 report istantaneous
#define SENSOR_TYPE_MSD           ("MSD")   // SSD011 report average
#define SENSOR_TYPE_NSD           ("NSD")   // SSD011 report minium
#define SENSOR_TYPE_XSD           ("XSD")   // SSD011 report maximum
#define SENSOR_TYPE_SMI           ("SMI")   // MICS4514 oneshot
#define SENSOR_TYPE_IMI           ("IMI")   // MICS4514 report istantaneous
#define SENSOR_TYPE_MMI           ("MMI")   // MICS4514 report average
#define SENSOR_TYPE_NMI           ("NMI")   // MICS4514 report minium
#define SENSOR_TYPE_XMI           ("XMI")   // MICS4514 report maximum
#define SENSOR_TYPE_RF24          ("RF24")  // Radio RF24

// add secondary to primary parameters to send in multiparameter sensors
//#define SECONDARYPARAMETER

//those ms after a prepare the measure will be too old to be considered valid

// use RF24Network library for radio transport
//#define RADIORF24

// use AES library for radio transport
//#define AES

// retry number for multimaster I2C configuration
//#define NTRY 3

// include TMP driver
//#define TMPDRIVER

// include ADT driver
// #define ADTDRIVER

// include HIH driver
// #define HIHDRIVER

// include HYT driver
//#define USE_SENSOR_HYT

// include BMP driver
// #define BMPDRIVER
//#define BMP085_DEBUG 1

// include DAVIS WIND driver
// #define DAVISWIND1

// include tipping bucket rain gauge driver
//#define TIPPINGBUCKETRAINGAUGE

// include TH temperature/humidity driver SAMPLE MODE
//#define TEMPERATUREHUMIDITY_ONESHOT

// include TH temperature/humidity driver REPORT MODE
//#define TEMPERATUREHUMIDITY_REPORT

// include sds011 pm 2.5 and pm 10 driver SAMPLE MODE
//#define SDS011_ONESHOT

// include sds011 pm 2.5 and pm 10 driver REPORT MODE
// #define SDS011_REPORT

// include mics4514 CO and NO2pm driver SAMPLE MODE
// #define MICS4514_ONESHOT

// include mics4514 CO and NO2pm driver REPORT MODE
//#define MICS4514_REPORT


// #if defined (TEMPERATUREHUMIDITY_ONESHOT)
// #if defined (TEMPERATUREHUMIDITY_REPORT)
// CANNOT DEFINE TEMPERATUREHUMIDITY_ONESHOT AND TEMPERATUREHUMIDITY_REPORT TOGETHER
// #endif
// #endif
//
// #if defined (SDS011_ONESHOT)
// #if defined (SDS011_REPORT)
// CANNOT DEFINE SDS011_ONESHOT AND SDS011_REPORT TOGETHER
// #endif
// #endif
//
// #if defined (MICS4514_ONESHOT)
// #if defined (MICS4514_REPORT)
// CANNOT DEFINE MICS4514_ONESHOT AND MICS4514_REPORT TOGETHER
// #endif
// #endif

#if defined (TIPPINGBUCKETRAINGAUGE)
 // how many rain for one tick of the rain gauge (Hg/m^2)
 // 0.2 Kg/m^2 for tips
 #define RAINFORTIP 2
#endif

#endif
