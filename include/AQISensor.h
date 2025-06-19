
#ifndef __AQISENSOR_H__
#define __AQISENSOR_H__

#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SGP30.h>
#include <Wire.h>
#include <cmath>
#include <HardwareSerial.h>
#include <Preferences.h>


//#define ENABLE_SO2_SENSOR     // Uncomment to enable SO2 sensor

#ifdef ENABLE_SO2_SENSOR
#include <SoftwareSerial.h>
#endif

extern TwoWire SGP30_Wire;

// Struct to hold AQI data
struct AQIData {
    float temperature;
    float humidity;
    float pressure;
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10_0;
    float co_ppm;
    float no2_ppm;
    float nh3_ppm;
    uint16_t eCO2;
    uint16_t TVOC;
    float ozone_ppb;
    #ifdef ENABLE_SO2_SENSOR
    float so2_ppm;
    #endif

};

// Functions for AQI sensor
class AQISensor {
public:
    AQISensor();
    bool init();            // Initialize all sensors
    AQIData getData();      // Read data from sensors
    
    bool aht20Initialized = false;
    bool bmp280Initialized = false;
    bool pm5007Initialized = false;
    bool mics6814Initialized = false;
    bool sgp30Initialized = false;
    bool ozoneSensorInitialized = false;
    #ifdef ENABLE_SO2_SENSOR
    bool so2SensorInitialized = false;
    #endif

private:
    Adafruit_AHTX0 aht;
    Adafruit_BMP280 bmp;
    Adafruit_SGP30 sgp;
    Preferences preferences;

    HardwareSerial *ozoneSerial;
    #ifdef ENABLE_SO2_SENSOR
    SoftwareSerial *so2Serial;
    #endif

    const uint8_t OZONE_TX_PIN = 25;
    const uint8_t OZONE_RX_PIN = 26;

    const uint8_t PMS_RX_PIN = 16;
    const uint8_t PMS_TX_PIN = 17;
    const uint8_t FRAME_LENGTH = 32;
    uint8_t setPassiveCommand[7] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
    uint8_t requestDataCommand[7] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};

    const uint8_t CO_SENSOR_PIN = 32;
    const uint8_t NH3_SENSOR_PIN = 35;
    const uint8_t NO2_SENSOR_PIN = 34;

    const float CO_SCALE = 0.1;
    const float NH3_SCALE = 0.1;
    const float NO2_SCALE = 0.01;

    time_t calibrationStartMillis = 0;
    bool calibrationInProgress = false;


    bool readPMS5007(uint16_t &pm1_0, uint16_t &pm2_5, uint16_t &pm10_0);
    void readMiCS6814(float &co_ppm, float &nh3_ppm, float &no2_ppm);
    void readSGP30(uint16_t &eCO2, uint16_t &TVOC);
    bool initOzoneSensor();
    bool readOzoneData(float &ozonePPM);
    #ifdef ENABLE_SO2_SENSOR
    bool initSO2Sensor();
    bool readSO2Sensor(float &so2PPM);
    #endif
    
    uint32_t getAbsoluteHumidity(float temperature, float humidity);
    void updateSGP30Baseline();

    AQIData aqiData;

    bool baselineUpdated;
    unsigned long startTime;
    const unsigned long CALIBRATION_DURATION = 12 * 60 * 60 * 1000;
    const unsigned long SEVEN_DAYS = 7 * 24 * 60 * 60 * 1000;

};

#endif


