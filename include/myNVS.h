#ifndef __MYNVS_H__
#define __MYNVS_H__

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

#define BUFF_LEN_NVS_KEY 16

#define NVS_NAMESPACE_SENSOR_INFO "sm-app"
#define NVS_KEY_SENSOR_NAME "sensor-name"

#define NVS_NAMESPACE_RESTART_INFO "restart-info"
#define NVS_KEY_RESTART_COUNT "count"
#define NVS_KEY_RESTART_TIME "time"

#define NVS_NAMESPACE_MQTT_INFO "mqtt-app"
#define NVS_KEY_MQTT_SERVER "ip_address"
#define NVS_KEY_MQTT_PORT "port"

#define NVS_NAMESPACE_FILE_INFO "pointer"
#define NVS_KEY_FILE_NAME "file-name"

/// @brief Read and write to Non-Volatile Storage using arguments.
namespace myNVS
{
    namespace read
    {
        void restartData(int &restart_count, uint32_t &restart_time);
        void mqttData(String &mqtt_server, uint16_t &mqtt_port);
        void sensorName(String &sensor_name);
        void currentFileName(String &file_name);
    }

    namespace write
    {
        void restartData(int restart_count, uint32_t restart_time);
        void mqttData(String mqtt_server, uint16_t mqtt_port);
        void sensorName(String sensor_name);
        void currentFileName(String file_name);
    }
}

#endif