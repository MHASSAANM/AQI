#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <ESP32Ping.h>
#include <FirebaseESP32.h>
#include "Storage.h"
#include "AQISensor.h"
#include "config.h"
#include "ESPWiFi.h"
#include "rtc.h"
#include "espMQTT.h"
#include "myNVS.h"
#include "Restart.h"
#include "DataTransfer.h"
#include "webServer.h"


// libraries for OLED
#ifdef OLED_DISPLAY
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#endif

#ifdef OTA_UPDATE
#include "OTA.h"
#endif



String dataPullTopic = "pull/";
String dataPushTopic = "push/";
String restartTopic = "restart/";
DataRequest *pendingDataRequests = nullptr;

AQISensor aqiSensor;
FirebaseData firebaseData;
WiFiClient espClient1;
WiFiClient espClient2;
PubSubClient mqttClient(espClient1);
PubSubClient dataClient(espClient2);
String sensorID = "";

// const uint8_t LEDS[] = {AQ_LED, STORAGE_LED, WIFI_LED, CLOUD_LED};

byte flags[10];

String serverWifiCreds[2];

void vAcquireData(void *pvParameters);
void vLiveBroadcast(void *pvParameters);
void vStorage(void *pvParameters);
void vDataPull(void *pvParameters);
void vWifiTransfer(void *pvParameters);
void vStatusLed(void *pvParameters);

TaskHandle_t dataTask, storageTask, wifiTask, ledTask, pullTask, liveBroadcastTask; 

SemaphoreHandle_t semaAqData1 , semaStorage1, semaWifi1;

#ifdef OLED_DISPLAY
Adafruit_SH1106 display(SDA, SCL);
void vOLEDDisplay(void *pvParameters);
TaskHandle_t oledTask;
bool oled_data = false;
String time_oled = "";
#endif

String towrite = "" , towrite_s = "", toTransfer = "", towrite_inst = "";
String latestVersion = "";

unsigned long reboottime;

byte _stg_fail_count = 0;
byte _rtc_fail_count = 0;


void setup() {
    Serial.begin(115200);
    log_d("\nCurrent Version: " FIRMWARE_VERSION "\nDescription: " FIRMWARE_DESCRIPTION "\nCommit Date: " COMMIT_DATE);

    #ifdef RESET_ESP_STORAGE

  Preferences preferences;
  preferences.begin(NVS_NAMESPACE_SENSOR_INFO, false);
  preferences.clear();
  preferences.end();

  log_i("Reset ESP Preferences!");
  delay(500);
  ESP.restart();
    #endif

    flags[aq_f] = 1;
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(AQ_LED, OUTPUT);
  // pinMode(STORAGE_LED, OUTPUT);
  // pinMode(WIFI_LED, OUTPUT);
  // pinMode(CLOUD_LED, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  // digitalWrite(AQ_LED, LOW);
  // digitalWrite(STORAGE_LED, LOW);
  // digitalWrite(WIFI_LED, LOW);
  // digitalWrite(CLOUD_LED, LOW);



  myNVS::read::mqttData(mqtt_server, mqtt_port);
#ifdef OLED_DISPLAY
  display.begin(SH1106_SWITCHCAPVCC, 0x3C); // Address 0x3D for 128x64
  log_d("Display Initialized");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("The first step to a more sustainable future");
  display.display();
#endif

while (storage.init_storage() == false && _stg_fail_count < FAIL_LIMIT)
{
  _stg_fail_count++;
  flags[sd_f] = 0;
  log_e("Storage initialization failed!");
}
if (_stg_fail_count >= FAIL_LIMIT)
{
  log_e("Storage failed after 3 tries...!");
}
else
{
  log_d("Storage initialization success!");
  flags[sd_f] = 1;
}

  if (wf.init())
  {
    if (wf.check_connection())
    {
      digitalWrite(LED_BUILTIN, HIGH);
      log_d("Initialized wifi successfully");
      log_d("MAC: %s", WiFi.macAddress().c_str());
      flags[wf_f] = 1;
    }
  }
  else
  {
    log_e("Wifi not Initialized");
  }

  myServerInitialize();


  while (initRTC() == false && _rtc_fail_count < FAIL_LIMIT)
  {
    _rtc_fail_count++;
    flags[rtc_f] = 0; // the system time would be 000 from start. The data would be ? I guess 1/1/2000, or maybe 1970..
    if (flags[sd_f])
    { // if storage is working    //This part of code isn't particularly needed.
      String curr_file = storage.curr_read_file;
      curr_file.remove(0, 1);
      String syear = curr_file.substring(1, 5);
      String smonth = curr_file.substring(5, 7);
      String sday = curr_file.substring(7, 9);
      int iyear = syear.toInt();
      int imonth = smonth.toInt();
      int iday = sday.toInt();
      _set_esp_time(iyear, imonth, iday);
    }
  }
  if (_rtc_fail_count >= FAIL_LIMIT)
  {
    log_e("RTC failed after 3 tries...!");
  }
  else
  {
    _set_esp_time();
    flags[rtc_f] = 1;
  }

  {
    myNVS::read::sensorName(sensorID);
     //sensorID = "testuu";
    if (sensorID != "")
    {
      log_i("ID found!\r\nDevice name: %s\r\n", sensorID.c_str());
    }
    else
    { // saved settings not found
      log_i("Settings not found! Loading from Firebase...");

      while (!wf.check_connection() || !wf.check_Internet())
      {
        flags[wf_f] = 0;
        log_e("No wifi available, waiting for connection");
        delay(1000);
      }
      flags[wf_f] = 1;
      digitalWrite(LED_BUILTIN, HIGH);
      Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
      Firebase.reconnectWiFi(true);
      Firebase.setReadTimeout(firebaseData, 1000 * 60);
      Firebase.setwriteSizeLimit(firebaseData, "small");
      String dataPath = "/" + WiFi.macAddress() + "/deviceID";
      if (Firebase.getString(firebaseData, dataPath, &sensorID))
      {
        myNVS::write::sensorName(sensorID);
      }
      log_i("Got device ID from Firebase: %s", sensorID.c_str());
      log_i("Restarting...");
      vTaskDelay(1000);
      ESP.restart();
    }
  }

  wf.start_soft_ap();
  // MQTT topics for handling data requests
  dataPullTopic += sensorID;
  dataPushTopic += sensorID;
  restartTopic += sensorID;

  flags[cloud_f] = 0;
  flags[cloud_blink_f] = 0;
  flags[cloud_setup_f] = 0;

  if (wf.check_connection())
  {
    if (wf.check_Internet())
    {
      flags[wf_f] = 1;
      digitalWrite(LED_BUILTIN, HIGH);
      update_essentials();
      myNVS::write::mqttData(mqtt_server, mqtt_port);
#ifdef OTA_UPDATE
      downloadUpdate();
#endif
      setRtcTime();

      if (!aqiSensor.init())
  {
    log_d("Failed to initialize AQI sensors!");
    } else {
        log_d("AQI sensor initialized successfully!");
    }

      mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
      mqttClient.setBufferSize(MAX_CHUNK_SIZE_B + 20);
      flags[cloud_setup_f] = 1;

      if (!mqttClient.connected())
      {
        mqttClient.connect(sensorID.c_str());

        if (mqttClient.connected())
        {
          flags[cloud_f] = 1;
          log_d("Cloud IoT Setup Complete");
        }
      }

      dataClient.setServer(mqtt_server.c_str(), mqtt_port);
      dataClient.setBufferSize(PUSH_BUFFER_SIZE);
      dataClient.setCallback(onDataRequest);
      String deviceName = sensorID + "-data";

      if (dataClient.connect(deviceName.c_str()))
      {
        if (dataClient.subscribe(dataPullTopic.c_str()))
        {
          log_d("Subscribed to topic %s", dataPullTopic.c_str());
        }
      }
    }
    else
    {
      flags[wf_f] = 0;
      log_i("Internet not availaible or wifi disconnected");
      WiFi.reconnect();
    }
  }
  else
  {
    flags[wf_f] = 0;
    log_i("Internet not availaible");
  }

  onRestart();


    semaAqData1 = xSemaphoreCreateBinary();
    semaStorage1 = xSemaphoreCreateBinary();
    semaWifi1 = xSemaphoreCreateBinary();

    xSemaphoreGive(semaWifi1);  // Initially allow DataPull to run
    xSemaphoreGive(semaAqData1);

    reboottime = time(nullptr);

  xTaskCreatePinnedToCore(vAcquireData, "Data Acquisition", 9000, NULL, 4, &dataTask, 1);
  xTaskCreatePinnedToCore(vLiveBroadcast, "Live Broadcast", 4000, NULL, 4, &liveBroadcastTask, 0);
  xTaskCreatePinnedToCore(vStorage, "Storage Handler", 5000, NULL, 3, &storageTask, 1);
  xTaskCreatePinnedToCore(vDataPull, "Data pull on Wifi", 7000, NULL, 2, &pullTask, 0);
  xTaskCreatePinnedToCore(vWifiTransfer, "Transfer data on Wifi", 6000, NULL, 2, &wifiTask, 0);
  #ifdef led_status
  xTaskCreatePinnedToCore(vStatusLed, "Status LED", 5000, NULL, 1, &ledTask, 1);
  #endif

  #ifdef OLED_DISPLAY
  xTaskCreatePinnedToCore(vOLEDDisplay, "OLED Display", 5000, NULL, 2, &oledTask, 0);
  #endif
}

void loop()
{
  vTaskDelay(10);
}


void vAcquireData(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int cycle_count = 1;

    for (;;) {
        cycle_count++;
        if (cycle_count > DATA_STORAGE_TIME * MS_IN_SECOND / (DATA_ACQUISITION_TIME * MS_IN_SECOND)) {
            cycle_count = 1;
        }

        xSemaphoreTake(semaAqData1, portMAX_DELAY); // Check semaphore

        towrite = "";
        log_i("Cycle number: %d\n", cycle_count);

        if (cycle_count == (DATA_STORAGE_TIME * MS_IN_SECOND / (DATA_ACQUISITION_TIME * MS_IN_SECOND))) {
            towrite = getTime() + "," + sensorID + ",";

            AQIData aqiData = {0};  // Ensures all sensor values are initialized to 0

            aqiData = aqiSensor.getData();



            #ifdef ENABLE_SO2_SENSOR
             towrite += (aqiSensor.aht20Initialized ? String(aqiData.temperature, 1) : "0") + ", " +
                       (aqiSensor.aht20Initialized ? String(aqiData.humidity, 1) : "0") + ", " +
                       (aqiSensor.bmp280Initialized ? String(aqiData.pressure, 1) : "0") + ", " +
                       (aqiSensor.pm5007Initialized ? String(aqiData.pm1_0) : "0") + ", " +
                       (aqiSensor.pm5007Initialized ? String(aqiData.pm2_5): "0") + ", " +
                       (aqiSensor.pm5007Initialized ? String(aqiData.pm10_0): "0") + ", " +
                       (aqiSensor.mics6814Initialized ? String(aqiData.co_ppm, 2) : "0") + ", " +
                       (aqiSensor.mics6814Initialized ? String(aqiData.nh3_ppm, 2) : "0") + ", " +
                       (aqiSensor.mics6814Initialized ? String(aqiData.no2_ppm, 2) : "0") + ", " +
                       (aqiSensor.sgp30Initialized ? String(aqiData.eCO2): "0") + ", " +
                       (aqiSensor.sgp30Initialized ? String(aqiData.TVOC): "0") + ", " +
                       (aqiSensor.ozoneSensorInitialized ? String(aqiData.ozone_ppb, 2) : "0") + ", " +
                       (aqiSensor.so2SensorInitialized ? String(aqiData.so2_ppm, 2) : "0");
                       #else
                       towrite += (aqiSensor.aht20Initialized ? String(aqiData.temperature, 1) : "0") + ", " +
                       (aqiSensor.aht20Initialized ? String(aqiData.humidity, 1) : "0") + ", " +
                       (aqiSensor.bmp280Initialized ? String(aqiData.pressure, 1) : "0") + ", " +
                       (aqiSensor.pm5007Initialized ? String(aqiData.pm1_0) : "0") + ", " +
                       (aqiSensor.pm5007Initialized ? String(aqiData.pm2_5): "0") + ", " +
                       (aqiSensor.pm5007Initialized ? String(aqiData.pm10_0): "0") + ", " +
                       (aqiSensor.mics6814Initialized ? String(aqiData.co_ppm, 2) : "0") + ", " +
                       (aqiSensor.mics6814Initialized ? String(aqiData.nh3_ppm, 2) : "0") + ", " +
                       (aqiSensor.mics6814Initialized ? String(aqiData.no2_ppm, 2) : "0") + ", " +
                       (aqiSensor.sgp30Initialized ? String(aqiData.eCO2): "0") + ", " +
                       (aqiSensor.sgp30Initialized ? String(aqiData.TVOC): "0") + ", " +
                       (aqiSensor.ozoneSensorInitialized ? String(aqiData.ozone_ppb, 2) : "0");
            #endif

#ifdef OLED_DISPLAY
            oled_data = true;
            time_oled = getTime();
#endif

            log_i("%s", towrite.c_str());

            if (flags[wf_f] && flags[cloud_f])
                toTransfer = towrite;
                live_broadcast(toTransfer);
        }

        xSemaphoreGive(semaAqData1); // Release semaphore
        xSemaphoreGive(semaStorage1);
        vTaskDelay(1);
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        log_v("Stack usage of acquiredata Task: %d", (int)uxHighWaterMark);
        vTaskDelayUntil(&xLastWakeTime, (DATA_ACQUISITION_TIME * MS_IN_SECOND));
    }
}

/**
 * @brief RTOS task for handling live data broadcast.
 *
 * - Receives data from `vAcquireData`.
 * - Tries to publish data over MQTT.
 * - If transmission fails, logs data to `/liveData/1.txt` or `/liveData/2.txt` (alternating every 10 minutes).
 * - If internet is restored, attempts to send stored backup files.
 *
 * Runs every DATA_ACQUISITION Times and is synchronized with data acquisition via `semaphores`.
 */
void vLiveBroadcast(void *pvParameters)
{
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    if (towrite_inst != "")
    {
      if (xSemaphoreTake(semaAqData1, portMAX_DELAY) == pdTRUE)
      {
        String towrite_live = towrite_inst;
        if (flags[wf_f])
        {
          if (towrite != "" && !flags[sd_f])
          {
            toTransfer = "";
            toTransfer = towrite;
            towrite = "";
            handleLiveTransfer();
          }
        }
        xSemaphoreGive(semaAqData1); // Release immediately after copying

        if (!live_broadcast(towrite_live))
        {
          if (flags[sd_f] && flags[rtc_f])
          {
            if (!storage.handleLiveDataFailure(towrite_live))
            {
              log_e("Storage write failed, possible SD issue!");
              flags[sd_f] = 0;
            }
          }
        }
        else
        {
          attemptLiveDataRecovery();
        }
      }
    }
    vTaskDelay(DATA_ACQUISITION_TIME * MS_IN_SECOND);
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    log_v("Stack usage of liveBroadCast Task: %d", (int)uxHighWaterMark);
  }
}



/**
 * @brief This function stores the data acquired from smartmeter.
 * The execution of this task is controlled by the semaphore semaStorage1. This
 * semaphore is given by the acquire data task. Since the semaphore is given
 * on every chunk of data generated so, the execution frequency of transmission
 * is same as that of string generation i.e. 60 seconds (DATA ACQ TIME * DATA STORAGE TIME).
 *
 * @param pvParameters void
 */
void vStorage(void *pvParameters)
{
  for (;;)
  { // infinite loop
    if (flags[sd_f])
    { // storage is working
      vTaskDelay(1);
      xSemaphoreTake(semaStorage1, portMAX_DELAY); // for syncing task with acquire
      if (towrite != "")
      {
        xSemaphoreTake(semaAqData1, portMAX_DELAY); // make copy of data and stop data acquisition
        towrite_s = "";
        towrite_s = towrite;
        towrite = "";
        xSemaphoreGive(semaAqData1);
        vTaskDelay(1);
        if (getUnixTime() > cutoff_time)
        {
          flags[rtc_f] = 1;
          bool _data_wrote = 0;
          uint8_t _write_counter = 0;
          while (_data_wrote == 0 && _write_counter < STORAGE_WRITE_FAILURE_LIMIT)
          {
            if (storage.write_data(getTime2(), towrite_s))
            {
              _data_wrote = 1;
              log_d("Data Written: %s", towrite_s.c_str());
              flags[sd_f] = 1;
              flags[sd_blink_f] = 1;
              log_i("data written to storage");
            }
            else
            {
              _data_wrote = 0;
              log_e("Storage stopped working!");
              _write_counter++;
              flags[sd_f] = 0;
            }
          }
          if (_write_counter >= STORAGE_WRITE_FAILURE_LIMIT)
          {
            log_e("Storage write failed after 3 tries...!");
          }
        }
        else
        {
          log_e("RTC time is incorrect! Data not logging...");
          flags[rtc_f] = 0;
        }
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        log_v("Stack usage of Storage Task: %d", (int)uxHighWaterMark);
      }
      xSemaphoreGive(semaStorage1);
      // else
      // log_d("No data received");
    }        
    else
    {
      int restartCount;
      uint32_t restartTime;
      myNVS::read::restartData(restartCount, restartTime);
      if (restartCount == 4)
      { 
        log_d("Restart count reached 4, restarting ESP after 10 mins...");
        vTaskDelay(10 * DATA_STORAGE_TIME * MS_IN_SECOND); // storage is not working, delay 10 minutes
      }

      log_i("ESP Restarting Due to storage failure...");
      ESP.restart();
    }
  } // end for
} // end vStorage task


void vDataPull(void *pvParameters)
{
  for (;;)
  {
    // Wait for Wi-Fi connection semaphore
    xSemaphoreTake(semaWifi1, portMAX_DELAY);
    vTaskDelay(1);

    if (flags[wf_f])
    {                                 
      if (dataClient.loop())
      {
        if (flags[sd_f] && pendingDataRequests)
        {
          xSemaphoreTake(semaStorage1, portMAX_DELAY);
          processDataRequests();
          xSemaphoreGive(semaStorage1);
          vTaskDelay(1000);
        }
      }
      else
      {
        dataClient.setServer(mqtt_server.c_str(), mqtt_port);
        dataClient.setBufferSize(PUSH_BUFFER_SIZE);
        dataClient.setCallback(onDataRequest);
        String deviceName = sensorID + "-data";

        if (dataClient.connect(deviceName.c_str()))
        {
          if (dataClient.subscribe(dataPullTopic.c_str()))
          {
            log_d("Subscribed to topic %s", dataPullTopic.c_str());
          }
          if (dataClient.subscribe(restartTopic.c_str()))
          {
            log_d("Subscribed to topic %s", restartTopic.c_str());
          } 
  

        }
        else
        {
          vTaskDelay(1000);
        }
      }
      vTaskDelay(10);
    }
    xSemaphoreGive(semaWifi1);
  }
}


void vWifiTransfer(void *pvParameters)
{
  for (;;)
  {
    xSemaphoreTake(semaWifi1, portMAX_DELAY);
    handleWifiConnection(); // Ensure Wi-Fi is connected
    xSemaphoreGive(semaWifi1);
    vTaskDelay(1);
    if (flags[wf_f])
    {
      if (wf.check_Internet())
      {
        if (flags[sd_f])
        {
          handleStorageTransfer();
        }
      }
      else
        log_e("Ping not received");
    }
    else if (!flags[sd_f])
      vTaskDelay(10 * 60 * MS_IN_SECOND);
    else
      vTaskDelay(DATA_STORAGE_TIME * MS_IN_SECOND);
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    log_v("Stack usage of Wifi Task: %d", (int)uxHighWaterMark);
    vTaskDelay(DATA_ACQUISITION_TIME * MS_IN_SECOND);
  }
}

#ifdef led_status
void vStatusLed(void *pvParameters)
{
  int _blink_count[] = {0, 0, 0, 0};
  const int max_blink = 3;
  for (;;)
  {                                         // infinite loop
    for (int _flag = 0; _flag < 4; _flag++) // Blink Index
    {
      uint8_t _ledPin = LEDS[_flag];
      if (flags[_flag])
      {
        digitalWrite(_ledPin, !digitalRead(_ledPin));
        if (_flag != cloud_blink_f)
        {
          _blink_count[_flag]++;
          if (_blink_count[_flag] > max_blink)
          {
            _blink_count[_flag] = 0;
            flags[_flag] = 0;
          }
        }
        else
          flags[_flag] = 0;
      }
      else
        digitalWrite(_ledPin, flags[_flag + 4]);
    }
    vTaskDelay(100);
  }
}
#endif

#ifdef OLED_DISPLAY
void vOLEDDisplay(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) // infinite loop
  {
    if (oled_data == true)
    {
      for (int i = 0; i < NUM_OF_SLOTS; i++)
      {
        display.clearDisplay();
        display.setCursor(0, 10);
        display.printf("%s\n V%d:%s\n I%d:%s\n P%d:%s\nPF%d:%s\n", time_oled.c_str(), i, voltage_oled[i].c_str(), i, current_oled[i].c_str(), i, power_oled[i].c_str(), i, pf_oled[i].c_str());
        display.display();
        vTaskDelay(DATA_STORAGE_TIME *  (DATA_ACQUISITION_TIME * MS_IN_SECOND) / NUM_OF_SLOTS);
      }

      oled_data = false;
    }
    vTaskDelayUntil(&xLastWakeTime, 10 *  (DATA_ACQUISITION_TIME * MS_IN_SECOND));
  }
}
#endif