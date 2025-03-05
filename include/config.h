#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Arduino.h>

#define FIRMWARE_VERSION "1.1.1"
#define FIRMWARE_DESCRIPTION "AQI Meter Firmware Created"
#define COMMIT_DATE "15th Feburary 2025"

#define OTA_UPDATE
// #define DUMMY_DATA
// #define OLED_DISPLAY
//#define RESET_ESP_STORAGE
#define Storage_init
#define wifi_ini
#define data_pull
// #define led_status

#define DEFAULT_SSID "NeuBolt"

#define DEFAULT_PASSWORD "12344321"
#define SOFT_AP_PASSWORD "123456789"

#define DEFAULT_MQTT_SERVER "203.135.63.47"
#define DEFAULT_MQTT_PORT 1883

#define FIREBASE_HOST "https://aqiid-f80ba-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "AIzaSyCqTKDtDcxL_xs8p0_a9S86tVDAQvj1j74"

#define WIFI_SCAN_FAILURE_LIMIT 3
#define STORAGE_WRITE_FAILURE_LIMIT 3

#define MS_IN_SECOND 1000
#define DATA_ACQUISITION_TIME 3 // perform action every "DATA_ACQUISITION_TIME" milliseconds
#define DATA_STORAGE_TIME 3    // store the string every "DATA_STORAGE_TIME" seconds

// LED pins
#define AQ_LED 32
#define STORAGE_LED 33
#define WIFI_LED 25
#define CLOUD_LED 13

enum flags_
{
    aq_blink_f,
    sd_blink_f,
    wf_blink_f,
    cloud_blink_f,
    aq_f,
    sd_f,
    wf_f,
    cloud_f,
    cloud_setup_f,
    rtc_f
};

extern byte flags[10];

#endif