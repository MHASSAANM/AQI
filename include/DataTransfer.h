#ifndef __DATATRANSFER_H__
#define __DATATRANSFER_H__

#include <Arduino.h>
#include <PubSubClient.h>
#include "rtc.h"
#include "config.h"
#include "espMQTT.h"
#include "ESPWiFi.h"
#include "ESP32Ping.h"

extern String sensorID;
extern bool serverData;
extern String serverWifiCreds[2];
extern PubSubClient mqttClient;
extern String towrite, toTransfer;
extern SemaphoreHandle_t semaAqData1, semaStorage1, semaWifi1, semaPull1;
extern const IPAddress remote_ip;

void handleWifiConnection();
void handleStorageTransfer();
void handleLiveTransfer();

#endif