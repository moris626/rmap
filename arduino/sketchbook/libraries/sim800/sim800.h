
#ifndef sim800_h
#define sim800_h

#include "Arduino.h"
#include <Time.h>
#include "IPAddress.h"

#include <debug.h>

#define SIM800_BUFFER_LENGTH                             (100)
#define SIM800_COMMAND_BUFFER_LENGTH                     (120)

#define SIM800_SERIAL_PORT                               (Serial1)
#define SIM800_SERIAL_PORT_BAUD_RATE                     (115200)

#define SIM800_STATE_NONE                                (0b00000000)
#define SIM800_STATE_ON                                  (0b00000001)
#define SIM800_STATE_INITIALIZED                         (0b00000010)
#define SIM800_STATE_SETTED                              (0b00000100)
#define SIM800_STATE_REGISTERED                          (0b00001000)
#define SIM800_STATE_HTTP_INITIALIZED                    (0b00010000)

#define AT_OK_STRING                                     ("OK")
#define AT_ERROR_STRING                                  ("ERROR")
#define AT_CONNECT_OK_STRING                             ("CONNECT")
#define AT_CONNECT_FAIL_STRING                           ("CONNECT FAIL")
#define AT_NORMAL_POWER_DOWN_STRING                      ("NORMAL POWER DOWN")
#define AT_CIPCLOSE_OK_STRING                            ("CLOSE OK")
#define AT_CIPCLOSE_ERROR_STRING                         ("ERROR")
#define AT_CIPSHUT_OK_STRING                             ("SHUT OK")
#define AT_CIPSHUT_ERROR_STRING                          ("ERROR")

#define SIM800_AT_DEFAULT_TIMEOUT_MS                     (1000)
#define SIM800_AT_DELAY_MS                               (200)    // delay between at command

#define SIM800_GENERIC_RETRY_COUNT_MAX                   (3)
#define SIM800_GENERIC_WAIT_DELAY_MS                     (3000)

#define SIM800_WAIT_FOR_NETWORK_DELAY_MS                 (5000)
#define SIM800_WAIT_FOR_NETWORK_RETRY_COUNT_MAX          (2)

#define SIM800_WAIT_FOR_SETUP_DELAY_MS                   (5000)
#define SIM800_WAIT_FOR_AUTOBAUD_DELAY_MS                (3000)

#define SIM800_WAIT_FOR_ATTACH_GPRS_DELAY_MS             (5000)

#define SIM800_POWER_ON_OFF_SWITCH_DELAY_MS              (1200)   // switch pulled down for on or off module
#define SIM800_POWER_ON_OFF_DONE_DELAY_MS                (2000)   // delay for ready after power on or power off
#define SIM800_POWER_ON_TO_OFF_DELAY_MS                  (10000)  // min ms to power off after power on

#define SIM800_WAIT_FOR_EXIT_TRANSPARENT_MODE_DELAY_MS   (1000)

#define SIM800_POWER_OFF_BY_SWITCH                       (0x01)
#define SIM800_POWER_OFF_BY_AT_COMMAND                   (0x02)

#define SIM800_IP_LENGTH                                 (16)
#define SIM800_IMEI_LENGTH                               (20)

#define SIM800_RSSI_MIN                                  (0)
#define SIM800_RSSI_MAX                                  (31)
#define SIM800_RSSI_UNKNOWN                              (99)

#define SIM800_BER_MIN                                   (0)
#define SIM800_BER_MAX                                   (7)
#define SIM800_BER_UNKNOWN                               (99)

#define SIM800_CGATT_RESPONSE_TIME_MAX_MS                (10000)
#define SIM800_CIICR_RESPONSE_TIME_MAX_MS                (85000)
#define SIM800_CIPSTART_RESPONSE_TIME_MAX_MS             (160000)
#define SIM800_CIPSHUT_RESPONSE_TIME_MAX_MS              (65000)

#define found(str, check)                                (strstr(str, check))
#define send(data)                                       (modem->print(data))

#define printStatus(status, ok, error)                   (status == SIM800_OK ? ok : error)

#define getImei(imei)                                    (getGsn(imei))
#define getIp(ip)                                        (getCifsr(ip))
#define getNetworkStatus(n, stat)                        (getCreg(n, stat))
#define getSignalQuality(rssi, ber)                      (getCsq(rssi, ber))
#define isGprsAttached(is_attached)                      (getCgatt(is_attached))
#define softwareSwitchOff()                              (sendCpowd())

typedef enum {
   SIM800_POWER_INIT,
   SIM800_POWER_SET_PIN_LOW,
   SIM800_POWER_SET_PIN_HIGH,
   SIM800_POWER_CHECK_STATUS,
   SIM800_POWER_END,
   SIM800_POWER_WAIT_STATE
} sim800_power_state_t;

typedef enum {
   SIM800_SETUP_INIT,
   SIM800_SETUP_RESET,
   SIM800_SETUP_ECHO_MODE,
   SIM800_SETUP_GET_SIGNAL_QUALITY,
   SIM800_SETUP_WAIT_NETWORK,
   SIM800_SETUP_END,
   SIM800_SETUP_WAIT_STATE
} sim800_setup_state_t;

typedef enum {
   SIM800_CONNECTION_START_INIT,
   SIM800_CONNECTION_START_CHECK_GPRS,
   SIM800_CONNECTION_START_ATTACH_GPRS,
   SIM800_CONNECTION_START_SINGLE_IP,
   SIM800_CONNECTION_START_TRANSPARENT_MODE,
   SIM800_CONNECTION_START_TRANSPARENT_MODE_CONFIG,
   SIM800_CONNECTION_START_APN_USERNAME_PASSWORD,
   SIM800_CONNECTION_START_CONNECT,
   SIM800_CONNECTION_START_GET_IP,
   SIM800_CONNECTION_START_END,
   SIM800_CONNECTION_START_WAIT_STATE
} sim800_connection_start_state_t;

typedef enum {
   SIM800_CONNECTION_INIT,
   SIM800_CONNECTION_OPEN,
   SIM800_CONNECTION_CHECK_STATUS,
   SIM800_CONNECTION_END,
   SIM800_CONNECTION_WAIT_STATE
} sim800_connection_state_t;

typedef enum {
   SIM800_CONNECTION_STOP_INIT,
   SIM800_CONNECTION_STOP_SWITCH_TO_COMMAND_MODE,
   SIM800_CONNECTION_STOP_CLOSE,
   SIM800_CONNECTION_STOP_CLOSE_PDP,
   SIM800_CONNECTION_STOP_DETACH_GPRS,
   SIM800_CONNECTION_STOP_END,
   SIM800_CONNECTION_STOP_WAIT_STATE
} sim800_connection_stop_state_t;

typedef enum {
   SIM800_EXIT_TRANSPARENT_MODE_INIT,
   SIM800_EXIT_TRANSPARENT_MODE_SEND_ESCAPE_SEQUENCE,
   SIM800_EXIT_TRANSPARENT_MODE_END,
   SIM800_EXIT_TRANSPARENT_MODE_WAIT_STATE
} sim800_exit_transparent_mode_state_t;

typedef enum {
   SIM800_AT_INIT,
   SIM800_AT_SEND,
   SIM800_AT_RECEIVE,
   SIM800_AT_END,
   SIM800_AT_WAIT_STATE
} sim800_at_state_t;

typedef enum {
   SIM800_BUSY,      // 0 busy: doing something
   SIM800_OK,        // 1 operation complete with no error
   SIM800_ERROR      // 2 operation abort due to error
} sim800_status_t;

class SIM800 {
 public:
  SIM800();

  bool isOn();
  bool isInitialized();
  bool isSetted();
  bool isRegistered();
  bool isHttpInitialized();

  void setSerial(HardwareSerial *serial);
  bool init(uint8_t _on_off_pin, uint8_t _reset_pin = 0xFF);

  sim800_status_t getGsn(char *imei);
  sim800_status_t getCreg(uint8_t *n, uint8_t *stat);
  sim800_status_t getCsq(uint8_t *rssi, uint8_t *ber);
  sim800_status_t getCgatt(bool *is_attached);
  sim800_status_t getCifsr(char *ip);
  sim800_status_t sendCpowd();
  sim800_status_t getCclk(tmElements_t *tm);
  sim800_status_t sendCclk(tmElements_t tm);

  sim800_status_t switchOn();
  sim800_status_t switchOff(uint8_t power_off_method = SIM800_POWER_OFF_BY_SWITCH);
  sim800_status_t sendAt();
  sim800_status_t initAutobaud();
  sim800_status_t setup();
  sim800_status_t startConnection(const char *apn, const char *username, const char *password);
  sim800_status_t connection(const char *tipo, const char *server, const int port);
  sim800_status_t stopConnection();
  sim800_status_t exitTransparentMode();

  void cleanInput();
  uint8_t receive(char *rx_buffer, const char *at_ok_string = AT_OK_STRING, const char *at_error_string = AT_ERROR_STRING);
  sim800_status_t sendAtCommand(const char *command, char *buf, const char *at_ok_string = AT_OK_STRING, const char *at_error_string = AT_ERROR_STRING, uint32_t timeout_ms = SIM800_AT_DEFAULT_TIMEOUT_MS);
  time_t  RTCget();
  uint8_t RTCread(tmElements_t &tm);

  uint8_t RTCset(time_t t);
  uint8_t RTCwrite(tmElements_t &tm);

  uint8_t state;

  // bool init_fixbaud();
  // bool startNetwork(const char *apn, const char *user, const char *pwd );
  // bool stopNetwork();
  // bool checkNetwork();
  // bool httpGET(const char* server, int port, const char* path, char* result, int resultlength);
  // bool resetModem();
  // bool receivelen(char *buf, uint32_t timeout_ms, unsigned int datalen);

 protected:
    HardwareSerial *modem = &Serial1;

 private:
  sim800_status_t switchModem(bool is_switching_on);

  uint8_t on_off_pin;
  uint8_t reset_pin;

  sim800_power_state_t sim800_power_state;
  sim800_setup_state_t sim800_setup_state;
  sim800_connection_start_state_t sim800_connection_start_state;
  sim800_connection_state_t sim800_connection_state;
  sim800_at_state_t sim800_at_state;
  sim800_connection_stop_state_t sim800_connection_stop_state;
  sim800_exit_transparent_mode_state_t sim800_exit_transparent_mode_state;
};

#endif
