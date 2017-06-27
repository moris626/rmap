#include "debug.h"

char serial_buffer_print[TRACE_PRINTF_BUFFER_LENGTH];
int16_t serial_buffer_print_written_char;

char *trace_printf(char *ptr, const char *fmt, ...) {
  va_list args;
  va_start (args, fmt);

  if (ptr == NULL)
    ptr = serial_buffer_print;

  serial_buffer_print_written_char = vsnprintf(ptr, TRACE_PRINTF_BUFFER_LENGTH, fmt, args);
  va_end (args);
  return ptr;
}

char *trace_printf_array(void *data, int16_t length, uint8_t type, const char *fmt, ...) {
  char *serial_buffer_ptr = serial_buffer_print;

  for (int i=0; i<length; i++) {
    switch (type) {
      case INT8:
        trace_printf(serial_buffer_ptr, fmt, ((int8_t *)(data))[i]);
      break;
      case INT16:
        trace_printf(serial_buffer_ptr, fmt, ((int16_t *)(data))[i]);
      break;
      case INT32:
        trace_printf(serial_buffer_ptr, fmt, ((int32_t *)(data))[i]);
      break;
      case UINT8:
        trace_printf(serial_buffer_ptr, fmt, ((uint8_t *)(data))[i]);
      break;
      case UINT16:
        trace_printf(serial_buffer_ptr, fmt, ((uint16_t *)(data))[i]);
      break;
      case UINT32:
        trace_printf(serial_buffer_ptr, fmt, ((uint32_t *)(data))[i]);
      break;
      case FLOAT:
        trace_printf(serial_buffer_ptr, fmt, ((float *)(data))[i]);
      break;
    }
    serial_buffer_ptr += serial_buffer_print_written_char;
  }

  // delete last character
  *(serial_buffer_ptr-1) = 0;

  return serial_buffer_print;
}
