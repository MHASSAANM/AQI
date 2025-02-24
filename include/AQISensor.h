/*#ifndef __AQISENSOR_H__
#define __AQISENSOR_H__

#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Struct to hold AQI data
struct AQIData {
    float temperature;
    float humidity;
    float pressure;
    uint16_t pm1_0;  // PM1.0 concentration
    uint16_t pm2_5;  // PM2.5 concentration
    uint16_t pm10_0; // PM10.0 concentration
};

// Functions for AQI sensor
class AQISensor {
public:
    AQISensor();
    bool init();                     // Initialize all sensors
    AQIData getData();               // Read data from sensors
    bool readPMS5007();              // Read data from PMS5007 sensor

private:
    Adafruit_AHTX0 aht;              // AHT20 sensor object
    Adafruit_BMP280 bmp;             // BMP280 sensor object
    AQIData aqiData;                 // Store sensor readings
};

#endif

#ifndef __AQISENSOR_H__
#define __AQISENSOR_H__

#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Struct to hold AQI data
struct AQIData {
    float temperature;
    float humidity;
    float pressure;
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10_0;
};

// Functions for AQI sensor
class AQISensor {
public:
    AQISensor();
    bool init();                     // Initialize all sensors
    AQIData getData();               // Read data from sensors

private:
    // AHT20 and BMP280 sensor objects
    Adafruit_AHTX0 aht;
    Adafruit_BMP280 bmp;
    
    // PM5007 sensor variables
    const uint8_t PMS_RX_PIN = 16;
    const uint8_t PMS_TX_PIN = 17;
    const uint8_t FRAME_LENGTH = 32;
    uint8_t setPassiveCommand[7] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
    uint8_t requestDataCommand[7] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};
    bool readPMS5007(uint16_t &pm1_0, uint16_t &pm2_5, uint16_t &pm10_0);

    AQIData aqiData; // Store sensor readings
};

#endif*/

#ifndef __AQISENSOR_H__
#define __AQISENSOR_H__

#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <cmath>
#include <SensirionI2CSgp41.h>
#include <NOxGasIndexAlgorithm.h>
#include <VOCGasIndexAlgorithm.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

// Struct to hold AQI data
struct AQIData {
    float temperature;
    float humidity;
    float pressure;
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10_0;
    uint16_t ozone_ppb;
    float co_ppm;
    float no2_ppm;
    float nh3_ppm;
    int32_t voc_index;
    int32_t nox_index;
    uint16_t so2_ppm;

};

// Functions for AQI sensor
class AQISensor {
public:
    AQISensor();
    bool init();            // Initialize all sensors
    AQIData getData();      // Read data from sensors

private:
    Adafruit_AHTX0 aht;
    Adafruit_BMP280 bmp;
    SensirionI2CSgp41 sgp41;
    VOCGasIndexAlgorithm voc_algorithm;
    NOxGasIndexAlgorithm nox_algorithm;

    HardwareSerial mySerial = HardwareSerial(1);
    SoftwareSerial so2Serial;

    const uint8_t PMS_RX_PIN = 16;
    const uint8_t PMS_TX_PIN = 17;
    const uint8_t FRAME_LENGTH = 32;
    uint8_t setPassiveCommand[7] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
    uint8_t requestDataCommand[7] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};

    bool readPMS5007(uint16_t &pm1_0, uint16_t &pm2_5, uint16_t &pm10_0);
    float readOzone();
    void readMICS6814(float &co, float &no2, float &nh3);
    void readSGP41(int32_t &voc_index, int32_t &nox_index);
    uint16_t readSO2();

    AQIData aqiData;
};

#endif


