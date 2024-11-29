#include "wifi_mqtt.h"
#include "connection_settings.h"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish Compressor_Control(&mqtt, AIO_USERNAME "/feeds/Compressor_Control");
Adafruit_MQTT_Subscribe Compressor_time_remaining(&mqtt, AIO_USERNAME "/feeds/Compressor_time_remaining");
Adafruit_MQTT_Subscribe air_dryer_controller_status(&mqtt, AIO_USERNAME "/feeds/air_dryer_controller_status");
Adafruit_MQTT_Publish air_dryer_override(&mqtt, AIO_USERNAME "/feeds/air_dryer_override");
Adafruit_MQTT_Subscribe *subscription;

unsigned long tLastMqttKeepAlive = 0;
uint32_t keepalive_interval_ms = 60000;

void initWifi(void) {
    Serial.print("Connecting to ");
    Serial.println(WLAN_SSID);
    WiFi.setHostname("esp32-airknob");
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    WiFi.setSleep(WIFI_PS_NONE);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected");
    Serial.println("IP address: "); Serial.println(WiFi.localIP());
}

void initMqtt(void) {
    mqtt.subscribe(&Compressor_time_remaining);
    mqtt.subscribe(&air_dryer_controller_status);
    //mqtt.will("/feeds/air_dryer_controller_status", "Air dryer controller disconnected");
    MQTT_connect();
}

void initOta(void) {
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("ArduinoOTA Initialized");
}

void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  uint8_t retries = 3;
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

void mqttKeepAlive() {
  unsigned long tNow = millis();
  if (tNow < tLastMqttKeepAlive) {
    tLastMqttKeepAlive = 0;  // Handle millis() rollover
  }
  if ((tNow - tLastMqttKeepAlive) >= keepalive_interval_ms) {
    Serial.println(F("Pinging MQTT server to keep alive"));
    if (! mqtt.ping()) {
      mqtt.disconnect();
    }
    tLastMqttKeepAlive = tNow;
  }
}

void checkWifiConnection(void) {
  static long previousMillis = 0;
  // if WiFi is down, try reconnecting
  if ((WiFi.status() != WL_CONNECTED) && (millis() - previousMillis >= 30000)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = millis();
  }
}