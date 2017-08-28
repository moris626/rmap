#ifndef _MQTT_CONFIG_H
#define _MQTT_CONFIG_H

#define MQTT_ROOT_TOPIC_LENGTH         (50)
#define MQTT_SUBSCRIBE_TOPIC_LENGTH    (50)
#define MQTT_SENSOR_TOPIC_LENGTH       (30)
#define MQTT_MESSAGE_LENGTH            (50)
#define MQTT_SERVER_LENGTH             (30)
#define MQTT_USERNAME_LENGTH           (30)
#define MQTT_PASSWORD_LENGTH           (30)

#define MQTT_TIMEOUT_MS                (5000)

#define MQTT_DEFAULT_SERVER            ("rmap.cc")
#define MQTT_DEFAULT_PORT              (1883)
#define MQTT_DEFAULT_ROOT_TOPIC        ("test/digiteco/1132822,4450078/fixed/")
#define MQTT_DEFAULT_SUBSCRIBE_TOPIC   ("test/digiteco/1132822,4450078/rx")
#define MQTT_DEFAULT_USERNAME          ("digiteco")
#define MQTT_DEFAULT_PASSWORD          ("dgt6943")

#if (MQTT_ROOT_TOPIC_LENGTH + MQTT_SENSOR_TOPIC_LENGTH > 100)
#error MQTT root/sensor topic is too big!
#endif

#endif
