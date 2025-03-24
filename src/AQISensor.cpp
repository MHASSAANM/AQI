#include "AQISensor.h"

// Constructor
AQISensor::AQISensor() : aqiData { 0.0,
                                   0.0,
                                   0.0,
                                   0,
                                   0,
                                   0 } {}

// Initialize sensors
bool AQISensor::init() {

    /*if (!aht.begin()) {
        log_e("Could not find AHT20 sensor! Check wiring.");
        return false;
    }
    log_d("AHT20 initialized");

    if (!bmp.begin(0x76)) {
        log_e("Could not find BMP280 sensor! Check wiring.");
        return false;
    }
    log_d("BMP280 initialized");

    Serial2.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
    Serial2.write(setPassiveCommand, sizeof(setPassiveCommand));
    vTaskDelay(1000);
    log_d("PM5007 initialized");*/

    aht20Initialized = aht.begin();
    if (!aht20Initialized) {
        log_e("Could not find AHT20 sensor! Check wiring.");
    } else {
        log_d("AHT20 initialized");
    }
    
    bmp280Initialized = bmp.begin(0x76);
    if (!bmp280Initialized) {
        log_e("Could not find BMP280 sensor! Check wiring.");
    } else {
        log_d("BMP280 initialized");
    }

    Serial2.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
    Serial2.write(setPassiveCommand, sizeof(setPassiveCommand));
    vTaskDelay(1000);
    
    uint16_t dummy_pm;
    pm5007Initialized = readPMS5007(dummy_pm, dummy_pm, dummy_pm);
    if (pm5007Initialized) {
        log_d("PM5007 initialized");
    } else {
        log_e("PM5007 not detected!");
    }

    return true;
}

// Read PM5007 data
bool AQISensor::readPMS5007(uint16_t &pm1_0, uint16_t &pm2_5, uint16_t &pm10_0) {
    uint8_t buffer[FRAME_LENGTH];
    uint16_t checksum = 0;

    while (Serial2.available()) {
        Serial2.read();
    }

    Serial2.write(requestDataCommand, sizeof(requestDataCommand));
    unsigned long startTime = millis();
    while (Serial2.available() < FRAME_LENGTH) {
        if (millis() - startTime > 1000) {
            log_e("Timeout waiting for data!");
            return false;
        }
    }

    for (uint8_t i = 0; i < FRAME_LENGTH; i++) {
        buffer[i] = Serial2.read();
    }

    if (buffer[0] != 0x42 || buffer[1] != 0x4D) {
        log_e("Invalid frame header!");
        return false;
    }

    for (uint8_t i = 0; i < FRAME_LENGTH - 2; i++) {
        checksum += buffer[i];
    }

    uint16_t receivedChecksum = (buffer[FRAME_LENGTH - 2] << 8) | buffer[FRAME_LENGTH - 1];
    if (checksum != receivedChecksum) {
        log_e("Checksum mismatch!");
        return false;
    }

    pm1_0 = (buffer[10] << 8) | buffer[11];
    pm2_5 = (buffer[12] << 8) | buffer[13];
    pm10_0 = (buffer[14] << 8) | buffer[15];
    return true;
}

// Get AQI data
AQIData AQISensor::getData() {
    sensors_event_t tempEvent, humidityEvent;
    aht.getEvent(&humidityEvent, &tempEvent);
    aqiData.temperature = tempEvent.temperature;
    aqiData.humidity = humidityEvent.relative_humidity;
    aqiData.pressure = bmp.readPressure() / 100.0;

    readPMS5007(aqiData.pm1_0, aqiData.pm2_5, aqiData.pm10_0);
    
    return aqiData;
}


