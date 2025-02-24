#include "DataTransfer.h"

void handleWifiConnection()
{
    static int wifiFailCounter = 0;

    if (serverData)
    {
        wf.create_new_connection(serverWifiCreds[0].c_str(), serverWifiCreds[1].c_str());
        serverData = false;
    }

    if (!wf.check_connection())
    {
        flags[wf_f] = 0;
        flags[cloud_f] = 0;

        if (wifiFailCounter >= 3)
        {
            wifiFailCounter = 0;
            log_i("Wifi disconnected!");
            digitalWrite(LED_BUILTIN, LOW);

            vTaskDelay(10000);
        }

        wifiFailCounter++;
    }
    else
    {
        

        if (!Ping.ping(remote_ip, 1))
        {
            digitalWrite(LED_BUILTIN, LOW);
            flags[wf_f] = 0;
            flags[cloud_f] = 0;
            log_d("Ping not received");
            WiFi.reconnect();
            vTaskDelay(2 * MS_IN_SECOND * DATA_STORAGE_TIME);
        }
        else
        {
            digitalWrite(LED_BUILTIN, HIGH);
            flags[wf_f] = 1;
            flags[wf_blink_f] = 1;

            if (!flags[cloud_f] && !flags[cloud_setup_f])
            {
                mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
                mqttClient.setBufferSize(MAX_CHUNK_SIZE_B + 20);
                flags[cloud_setup_f] = 1;
                log_d("Cloud IoT Setup Complete");
            }

            mqttClient.loop();

            vTaskDelay(10); // <- fixes some issues with WiFi stability

            if (!mqttClient.connected())
            {
                flags[cloud_f] = 0;

                for (int i = 0; i < 3; i++)
                {
                    vTaskDelay(10); // <- fixes some issues with WiFi stability
                    configTime(18000, 0, "pool.ntp.org");

                    while (time(nullptr) < cutoff_time) // time less than 1 Jan 2021 12:00 AM
                    {
                        vTaskDelay(1000);
                        log_e("Waiting for time update from NTP server...");
                    }

                    mqttClient.connect(sensorID.c_str());

                    if (mqttClient.connected())
                    {
                        flags[cloud_f] = 1;
                        break;
                    }
                }
            }
        }
    }
}

void handleStorageTransfer()
{
    static String toread = "";
    static int chunksSentCounter = 0;
    xSemaphoreTake(semaStorage1, portMAX_DELAY);
    xSemaphoreTake(semaAqData1, portMAX_DELAY);
    long unsent_bytes = storage.get_unsent_data(getTime2());
    xSemaphoreGive(semaAqData1);
    xSemaphoreGive(semaStorage1);
    if (unsent_bytes > MAX_CHUNK_SIZE_B)
    {
        if (flags[cloud_f])
        {

            if (mqttClient.connected())
            {
                xSemaphoreTake(semaStorage1, portMAX_DELAY);
                for (int i = 0; i < 10 && unsent_bytes > MIN_CHUNK_SIZE_B; i++)
                {
                    vTaskDelay(10); // <- fixes some issues with WiFi stability
                    xSemaphoreTake(semaAqData1, portMAX_DELAY);
                    toread = "";
                    toread = storage.read_data();
                    if (toread != "" && mqttClient.publish(MQTT_TOPIC, toread.c_str()))
                    {
                        flags[cloud_blink_f] = 1;
                        digitalWrite(LED_BUILTIN, LOW);
                        vTaskDelay(100);
                        digitalWrite(LED_BUILTIN, HIGH);
                        log_d("Data Sent: %s", toread.c_str());
                        log_d("sent data to cloud ");
                        chunksSentCounter++;
                        storage.mark_data(getTime2());
                        unsent_bytes -= strlen(toread.c_str()) + 4;
                    }
                    xSemaphoreGive(semaAqData1);
                }
                xSemaphoreGive(semaStorage1);
                vTaskDelay(DATA_STORAGE_TIME * MS_IN_SECOND);
            }
            else
            {
                log_e("MQTT Broker disconnected");
                flags[cloud_f] = 0;
            }
        }
    }
    else
    {
        log_i("No Data to be sent!");
        vTaskDelay(DATA_STORAGE_TIME * MS_IN_SECOND);
    }

    if (chunksSentCounter >= 2)
    {
        setRtcTime();
        log_i("Sent %d data chunks to cloud.", chunksSentCounter);
        chunksSentCounter = 0;
        vTaskDelay(1);
    }
}

void handleLiveTransfer()
{
    if (toTransfer != "")
    {
        if (flags[cloud_f])
        {
            if (mqttClient.connected())
            {
                xSemaphoreTake(semaStorage1, portMAX_DELAY);
                xSemaphoreTake(semaAqData1, portMAX_DELAY);
                if (mqttClient.publish(MQTT_TOPIC, toTransfer.c_str()))
                {
                    flags[cloud_blink_f] = 1;
                    log_d("Sending live data to cloud...");
                    log_d("Data Sent: %s", toTransfer.c_str());
                    toTransfer = "";
                }
                xSemaphoreGive(semaAqData1);
                xSemaphoreGive(semaStorage1);
            }
            else
            {
                log_e("MQTT Broker disconnected");
            }
            vTaskDelay(DATA_STORAGE_TIME * MS_IN_SECOND);
        }
    }
}