
#ifndef __AQISENSOR_H__
#define __AQISENSOR_H__

#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>

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
    bool init();            // Initialize all sensors
    AQIData getData();      // Read data from sensors
    bool aht20Initialized = false;
    bool bmp280Initialized = false;
    bool pm5007Initialized = false;

private:
    Adafruit_AHTX0 aht;
    Adafruit_BMP280 bmp;

    const uint8_t PMS_RX_PIN = 16;
    const uint8_t PMS_TX_PIN = 17;
    const uint8_t FRAME_LENGTH = 32;
    uint8_t setPassiveCommand[7] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
    uint8_t requestDataCommand[7] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};

    bool readPMS5007(uint16_t &pm1_0, uint16_t &pm2_5, uint16_t &pm10_0);
    uint32_t getAbsoluteHumidity(float temperature, float humidity);

    AQIData aqiData;
};

#endif


