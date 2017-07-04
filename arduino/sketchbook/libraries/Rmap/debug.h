#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdarg.h>
#include <stdio.h>

// Debug level definitions
// #define SERIAL_TRACE_LEVEL_OFF          (0)
// #define SERIAL_TRACE_LEVEL_ERROR        (1)
// #define SERIAL_TRACE_LEVEL_WARNING      (2)
// #define SERIAL_TRACE_LEVEL_INFO         (3)
// #define SERIAL_TRACE_LEVEL_DEBUG        (4)

#define INT8    (1)
#define INT16   (2)
#define INT32   (3)
#define UINT8   (4)
#define UINT16  (5)
#define UINT32  (6)
#define FLOAT   (7)

#define TRACE_PRINTF_BUFFER_LENGTH        (256)

char *trace_printf(char *ptr, const char *fmt, ...);
char *trace_printf_array(void *data, int16_t length, uint8_t type, const char *fmt, ...);

// Default Debug level
#ifndef SERIAL_TRACE_LEVEL
   #define SERIAL_TRACE_LEVEL SERIAL_TRACE_LEVEL_OFF
#endif

// Debug output redirection
#if (SERIAL_TRACE_LEVEL > SERIAL_TRACE_LEVEL_OFF)
  #ifndef _SERIAL_PRINT
  #define _SERIAL_PRINT(...) Serial.print(trace_printf(__VA_ARGS__))
  #endif
  #ifndef _SERIAL_PRINT_ARRAY
  #define _SERIAL_PRINT_ARRAY(...) Serial.print(trace_printf_array(__VA_ARGS__))
  #endif
  #ifndef TRACE_BEGIN
  #define TRACE_BEGIN(...) Serial.begin(__VA_ARGS__)
  #endif
#else
  #ifndef TRACE_BEGIN
    #define TRACE_BEGIN(...)
  #endif
#endif

// Debugging macros
#if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_ERROR)
  #define SERIAL_ERROR(...) _SERIAL_PRINT(NULL, __VA_ARGS__)
  #define SERIAL_ERROR_ARRAY(...) _SERIAL_PRINT_ARRAY(__VA_ARGS__)
#else
  #define SERIAL_ERROR(...)
  #define SERIAL_ERROR_ARRAY(...)
#endif

#if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_WARNING)
  #define SERIAL_WARNING(...) _SERIAL_PRINT(NULL, __VA_ARGS__)
  #define SERIAL_WARNING_ARRAY(...) _SERIAL_PRINT_ARRAY(__VA_ARGS__)
#else
  #define SERIAL_WARNING(...)
  #define SERIAL_WARNING_ARRAY(...)
#endif

#if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_INFO)
  #define SERIAL_INFO(...) _SERIAL_PRINT(NULL, __VA_ARGS__)
  #define SERIAL_INFO_ARRAY(...) _SERIAL_PRINT_ARRAY(__VA_ARGS__)
#else
  #define SERIAL_INFO(...)
  #define SERIAL_INFO_ARRAY(...)
#endif

#if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_DEBUG)
  #define SERIAL_DEBUG(...) _SERIAL_PRINT(NULL, __VA_ARGS__)
  #define SERIAL_DEBUG_ARRAY(...) _SERIAL_PRINT_ARRAY(__VA_ARGS__)
#else
  #define SERIAL_DEBUG(...)
  #define SERIAL_DEBUG_ARRAY(...)
#endif

#if (SERIAL_TRACE_LEVEL >= SERIAL_TRACE_LEVEL_TRACE)
  #define SERIAL_TRACE(...) _SERIAL_PRINT(NULL, __VA_ARGS__)
  #define SERIAL_TRACE_ARRAY(...) _SERIAL_PRINT_ARRAY(__VA_ARGS__)
#else
  #define SERIAL_TRACE(...)
  #define SERIAL_TRACE_ARRAY(...)
#endif

#endif
