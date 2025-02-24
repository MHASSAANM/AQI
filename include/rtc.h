#ifndef _RTC_H_
#define _RTC_H_
#include <Arduino.h> //for using arduino String
#include <ESP32Time.h>

extern ESP32Time esp_sys_time;
bool initRTC();
String getTime();
String getTime2();
uint32_t getUnixTime();
String unixTime();
String getNextDay(int, int, int);
void _set_esp_time();
void _set_esp_time(int, int, int);
void setRtcTime();

#endif