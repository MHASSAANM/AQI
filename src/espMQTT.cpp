#include "espMQTT.h"
#include "Preferences.h"
#include "FirebaseESP32.h"
#include "rtc.h"

#define FIREBASE_HOST "aqiid-f80ba-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "CFbINwH5QuFq41EcSAY3WcPcDJNhLbXNFcvWIuZM"

String mqtt_server = DEFAULT_MQTT_SERVER;
uint16_t mqtt_port = DEFAULT_MQTT_PORT;
extern FirebaseData firebaseData;

void update_essentials()
{
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);
    Firebase.setReadTimeout(firebaseData, 1000 * 60);
    Firebase.setwriteSizeLimit(firebaseData, "small");
    String tmps_IP = "";
    uint16_t tmps_port = 0;
    String s_IP = "/" + WiFi.macAddress() + "/IP";
    String s_port = "/" + WiFi.macAddress() + "/Port";
    String s_SD = "/" + WiFi.macAddress() + "/SD_Flag";
    String s_RTC = "/" + WiFi.macAddress() + "/RTC_Flag";
    String s_Time = "/" + WiFi.macAddress() + "/lastRestart";
    String currentVersionPath = "/" + WiFi.macAddress() + "/Version";
    String latestVersionPath = "/LatestVersion";
    Firebase.getString(firebaseData, s_IP, &tmps_IP);
    Firebase.getInt(firebaseData, s_port, &tmps_port);
    Firebase.getString(firebaseData, latestVersionPath, &latestVersion);
    Firebase.setString(firebaseData, currentVersionPath, String(FIRMWARE_VERSION));
    Firebase.setBool(firebaseData,  s_SD, flags[sd_f]);
    Firebase.setBool(firebaseData, s_RTC, flags[rtc_f]);
    Firebase.setString(firebaseData, s_Time, getTime());

    if (tmps_IP == "")
        Firebase.setString(firebaseData, s_IP, mqtt_server);
    else if (tmps_IP != mqtt_server)
    {
        mqtt_server = tmps_IP;
        log_i("%s", mqtt_server.c_str());
    }
    if (!tmps_port)
        Firebase.setInt(firebaseData, s_port, mqtt_port);
    else if (tmps_port != mqtt_port)
    {
        mqtt_port = tmps_port;
        log_i("%d", mqtt_port);
    }
    Firebase.end(firebaseData);
    Firebase.endStream(firebaseData);
}

bool isValidDate(const String &date)
{
    if (date.length() != 8)
    {
        return false;
    }

    for (char c : date)
    {
        if (!isdigit(c))
        {
            return false;
        }
    }

    int year, month, day;

    if (sscanf(date.c_str(), "%4d%2d%2d", &year, &month, &day) != 3)
    {
        return false;
    }

    if (year < 2000 || month < 1 || month > 12 || day < 1 || day > 31)
    {
        return false;
    }

    return true;
}

void addDataRequest(const char *date)
{
    DataRequest *newPacket = new DataRequest;
    strncpy(newPacket->date, date, 8);
    newPacket->date[8] = '\0';
    newPacket->next = nullptr;

    if (!pendingDataRequests)
    {
        pendingDataRequests = newPacket;
    }
    else
    {
        DataRequest *current = pendingDataRequests;
        while (current->next)
        {
            current = current->next;
        }
        current->next = newPacket;
    }

    log_d("Added date %s to data request queue", date);
}

void processPayload(const String &payload)
{
    char delimiter = ',';
    int startPos = 0;
    int endPos = payload.indexOf(delimiter);

    while (endPos != -1)
    {
        String date = payload.substring(startPos, endPos);
        date.trim();
        log_d("Extracted data from payload: %s", date.c_str());
        startPos = endPos + 1;
        endPos = payload.indexOf(delimiter, startPos);

        if (isValidDate(date))
        {
            addDataRequest(date.c_str());
        }
    }

    String lastDate = payload.substring(startPos);
    lastDate.trim();
    log_d("Extracted data from payload: %s", lastDate.c_str());
    if (isValidDate(lastDate))
    {
        addDataRequest(lastDate.c_str());
    }
}

void onDataRequest(char *topic, uint8_t *buff, unsigned int size)
{
    log_d("Callback function called with topic %s", topic);
    if (strcmp(topic, dataPullTopic.c_str()) == 0) // topic matches data pull topic
    {
        String payload = "";
        for (int i = 0; i < size; i++)
        {
            payload += (char)buff[i];
        }
        log_d("Processing payload: %s", payload.c_str());
        processPayload(payload);
    }
}

bool sendFile(const char *fileName, const char *topic)
{
    if (!SD.exists(fileName))
    {
        log_e("File %s does not exist. Data will not be sent!", fileName);
        if (dataClient.publish(topic, "Not Found"))
        {
            log_d("Published \"Not Found\" message to topic [%s]", topic);
        }
        return false;
    }

    File file = SD.open(fileName);

    if (!file)
    {
        log_e("File %s could not be opened. Data will not be sent!", fileName);
        if (dataClient.publish(topic, "Not Found"))
        {
            log_d("Published \"Not Found\" message to topic [%s]", topic);
        }
        return false;
    }

    if (file.isDirectory())
    {
        log_e("File %s is a directory. Data will not be sent!", fileName);
        if (dataClient.publish(topic, "Not Found"))
        {
            log_d("Published \"Not Found\" message to topic [%s]", topic);
        }
        file.close();
        return false;
    }

    log_d("File %s will be published on topic [%s]", fileName, topic);

    size_t fileSize = file.size();
    // first packet is file size
    if (dataClient.publish(topic, String(fileSize).c_str()))
    {
        log_d("Published file [%s] size [%zu] at topic [%s]", fileName, fileSize, topic);
    }

    size_t numOfPackets = fileSize / PUSH_PACKET_SIZE;
    size_t lastPacketSize = fileSize % PUSH_PACKET_SIZE;
    if (lastPacketSize)
    {
        numOfPackets += 1;
    }

    log_d("Number of packets to be sent: %d", numOfPackets);

    file.seek(0);
    size_t startIndex = 0;

    // send all packets except last
    for (unsigned int packetIndex = 0; packetIndex < numOfPackets - 1; packetIndex++)
    {
        file.seek(startIndex);

        char buffer[PUSH_PACKET_SIZE + 1];
        size_t bytesRead = file.readBytes(buffer, PUSH_PACKET_SIZE); // TODO: check if bytes are actually read
        buffer[PUSH_PACKET_SIZE] = '\0';

        startIndex += bytesRead;

        if (dataClient.publish(topic, buffer))
        {
            log_d("Sent data packet (%d/%d) for %s to cloud.", packetIndex + 1, numOfPackets, fileName);
        }
    }

    // send last packet
    file.seek(startIndex);
    char *lastBuff = new char[lastPacketSize + 1];
    file.readBytes(lastBuff, lastPacketSize);
    lastBuff[lastPacketSize] = '\0';

    if (dataClient.publish(topic, lastBuff))
    {
        log_d("Sent data packet (%d/%d) for %s to cloud.", numOfPackets, numOfPackets, fileName);
    }

    delete[] lastBuff;
    file.close();
    return true;
}

void processDataRequests()
{
    DataRequest *current = pendingDataRequests;
    while (current)
    {
        log_d("Packet size is %d bytes", PUSH_PACKET_SIZE);
        log_d("Buffer size is %d bytes", PUSH_BUFFER_SIZE);

        String fileName = "/" + String(current->date) + ".txt";
        //Serial.println(fileName);
        String filePushTopic = dataPushTopic + "/" + String(current->date); // specific topic for each file

        sendFile(fileName.c_str(), filePushTopic.c_str());

        DataRequest *nextPacket = current->next;
        delete current;
        current = nextPacket;
    }
    pendingDataRequests = nullptr;
}