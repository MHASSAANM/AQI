
#ifndef __WEBSERVER_H__
#define __WEBSERVER_H__

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "AQISensor.h"
#include "myNVS.h"

// these macros convert a number macro to string
#define __STRHELPER(x) #x
#define __STR(x) __STRHELPER(x)

// define all relevant HTML element IDs here
#define HTML_WIFI_SSID_ID "HTML_SSID"
#define HTML_WIFI_PASS_ID "HTML_PASS"

// variables
extern bool serverData;
extern String serverWifiCreds[2];
extern AQISensor aqiSensor;

// functions
void myServerInitialize(void);

#endif