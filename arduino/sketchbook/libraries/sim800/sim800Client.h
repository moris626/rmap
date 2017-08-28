#ifndef Client_h
#define Client_h

#include "sim800.h"

class sim800Client : public SIM800 {

 public:
  sim800Client();
  int connect(IPAddress ip, int port);
  int connect(const char *host, int port);
  uint8_t connected();
  int available();
  int read();
  int readBytes(char *buffer, size_t size);
  int readBytes(uint8_t *buffer, size_t size);
  void setTimeout(uint32_t timeout);
  size_t write(uint8_t);
  size_t write(const uint8_t *buffer, size_t size);
  void flush();
  void stop();
  bool transparentescape();
  bool transparent();
};

#endif
