#ifndef __ESP_MQTT_H__
#define __ESP_MQTT_H__

#include <Arduino.h>
#include "Storage.h"
#include "PubSubClient.h"
#include "config.h"

#define PUSH_PACKET_SIZE 2048
#define PUSH_BUFFER_SIZE (((PUSH_PACKET_SIZE / 128) + 1) * 128)

#define MQTT_TOPIC "EiG/AQIMeter"

struct DataRequest
{
    char date[9]; // "YYYYMMDD" + '\0'
    DataRequest *next;
};

extern String mqtt_server;
extern uint16_t mqtt_port;
extern String latestVersion;

extern PubSubClient dataClient;
extern DataRequest *pendingDataRequests;
extern String dataPullTopic;
extern String dataPushTopic;

void onDataRequest(char *topic, uint8_t *buff, unsigned int size);
bool sendFile(const char *fileName, const char *topic);
void processDataRequests();

void update_essentials(void);

#endif