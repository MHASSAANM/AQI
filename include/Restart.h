#ifndef __RESTART_H__
#define __RESTART_H__

#include <Arduino.h>
#include "config.h"
#include "myNVS.h"
#include "ESPWiFi.h"
#include "espMQTT.h"
#include "Storage.h"
#include"rtc.h"

#define RESTART_PUSH_TOPIC "test/restart/" // appended with sensorID and filename
#define RESTART_THRESH_SECS 3600
#define RESTART_COUNT_LIMIT 5

extern String sensorID;

void incrementRestartCount();
bool reachedRestartLimit();
void resetRestartCount();
void onRestart();

#endif