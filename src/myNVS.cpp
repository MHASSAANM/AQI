#include "myNVS.h"

void myNVS::read::restartData(int &restart_count, uint32_t &restart_time)
{
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE_RESTART_INFO);

    if (!preferences.isKey(NVS_KEY_RESTART_COUNT))
    {
        preferences.putInt(NVS_KEY_RESTART_COUNT, 0);
    }

    if (!preferences.isKey(NVS_KEY_RESTART_TIME))
    {
        preferences.putUInt(NVS_KEY_RESTART_TIME, 0);
    }

    restart_count = preferences.getInt(NVS_KEY_RESTART_COUNT, 0);
    restart_time = preferences.getUInt(NVS_KEY_RESTART_TIME, 0);
    preferences.end();
}

void myNVS::write::restartData(int restart_count, uint32_t restart_time)
{
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE_RESTART_INFO);
    preferences.putInt(NVS_KEY_RESTART_COUNT, restart_count);
    preferences.putUInt(NVS_KEY_RESTART_TIME, restart_time);
    preferences.end();
}

void myNVS::read::mqttData(String &mqtt_server, uint16_t &mqtt_port)
{
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE_MQTT_INFO);

    if (!preferences.isKey(NVS_KEY_MQTT_SERVER))
    {
        preferences.putString(NVS_KEY_MQTT_SERVER, DEFAULT_MQTT_SERVER);
    }

    if (!preferences.isKey(NVS_KEY_MQTT_PORT))
    {
        preferences.putInt(NVS_KEY_MQTT_PORT, DEFAULT_MQTT_PORT);
    }

    mqtt_server = preferences.getString(NVS_KEY_MQTT_SERVER, DEFAULT_MQTT_SERVER);
    mqtt_port = preferences.getInt(NVS_KEY_MQTT_PORT, DEFAULT_MQTT_PORT);
    preferences.end();
}

void myNVS::write::mqttData(String mqtt_server, uint16_t mqtt_port)
{
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE_MQTT_INFO);
    preferences.putString(NVS_KEY_MQTT_SERVER, mqtt_server);
    preferences.putInt(NVS_KEY_MQTT_PORT, mqtt_port);
    preferences.end();
}

void myNVS::read::sensorName(String &sensor_name)
{
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE_SENSOR_INFO, true);
    sensor_name = preferences.getString(NVS_KEY_SENSOR_NAME, "");
    preferences.end();
}

void myNVS::write::sensorName(String sensor_name)
{
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE_SENSOR_INFO);
    preferences.putString(NVS_KEY_SENSOR_NAME, sensor_name);
    preferences.end();
}

void myNVS::read::currentFileName(String &file_name)
{
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE_FILE_INFO, true);
    file_name = preferences.getString(NVS_KEY_FILE_NAME, "");
    preferences.end();
}

void myNVS::write::currentFileName(String file_name)
{
    Preferences preferences;
    preferences.begin(NVS_NAMESPACE_FILE_INFO);
    preferences.putString(NVS_KEY_FILE_NAME, file_name);
    preferences.end();
}