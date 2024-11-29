#include "Arduino.h"
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "connection_settings.h"
#include <ArduinoOTA.h>

extern WiFiClient client;
extern Adafruit_MQTT_Client mqtt;
extern Adafruit_MQTT_Publish Compressor_Control;
extern Adafruit_MQTT_Subscribe Compressor_time_remaining;
extern Adafruit_MQTT_Subscribe air_dryer_controller_status;
extern Adafruit_MQTT_Publish air_dryer_override;
extern Adafruit_MQTT_Subscribe *subscription;
extern ArduinoOTAClass ArduinoOTA;

extern unsigned long tLastMqttKeepAlive;
extern uint32_t keepalive_interval_ms;

void initWifi(void);
void initMqtt(void);
void initOta(void);
void MQTT_connect(void);
void mqttKeepAlive(void);
void checkWifiConnection(void);