#include "AQISensor.h"

#ifdef ENABLE_SO2_SENSOR
AQISensor::AQISensor() : ozoneSerial(&Serial1), so2Serial(nullptr), aqiData{0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0}, baselineUpdated(false) {}
#else
AQISensor::AQISensor() : ozoneSerial(&Serial1), aqiData{0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0.0}, baselineUpdated(false) {}
#endif

TwoWire SGP30_Wire = TwoWire(1);

bool AQISensor::init() {
    // Initialize main I2C bus first
    Wire.begin();
    delay(100);
    
    // Initialize AHT20 and BMP280 on main I2C bus
    aht20Initialized = aht.begin();
    if (!aht20Initialized) {
        log_e("Could not find AHT20 sensor! Check wiring.");
    } else {
        log_d("AHT20 initialized");
    }

    // Initialize BMP280
    bmp280Initialized = bmp.begin(0x76);
    if (!bmp280Initialized) {
        log_e("Could not find BMP280 sensor on address 0x76! Trying alternate address 0x77...");
        // Try alternate address
        bmp280Initialized = bmp.begin(0x77);
        if (bmp280Initialized) {
            log_d("BMP280 initialized on address 0x77");
        }
    } else {
        log_d("BMP280 initialized successfully on address 0x76");
    }
    
        // Initialize SGP30 sensor and check baseline
        SGP30_Wire.begin(15, 27, 100000);  // SDA=15, SCL=27, 100kHz
        delay(500);
        log_d("Scanning SGP30 I2C bus...");
        SGP30_Wire.beginTransmission(0x58);
        uint8_t error = SGP30_Wire.endTransmission();
    
        if (error == 0) {
            log_d("SGP30 detected at address 0x58");
    
            int retryCount = 0;
            const int maxRetries = 3;
            while (!sgp30Initialized && retryCount < maxRetries) {
                sgp30Initialized = sgp.begin(&SGP30_Wire);
                if (!sgp30Initialized) {
                    retryCount++;
                    log_w("SGP30 initialization attempt %d failed, retrying...", retryCount);
                    delay(1000);
                }
            }
    
            if (sgp30Initialized) {
                log_d("SGP30 initialized successfully");
    
                preferences.begin("sgp30", false);
                uint16_t eCO2_base = preferences.getUShort("eCO2_base", 0);
                uint16_t TVOC_base = preferences.getUShort("TVOC_base", 0);
                unsigned long lastCalibrationTime = preferences.getULong("lastCalibTime", 0);
                calibrationInProgress = preferences.getBool("calibInProgress", false);
                calibrationStartMillis = preferences.getULong("calibStartTime", 0);
                preferences.end();

                log_d("Loaded preferences:");
                log_d("  eCO2_base: %d", eCO2_base);
                log_d("  TVOC_base: %d", TVOC_base);
                log_d("  lastCalibrationTime: %lu", lastCalibrationTime);
                log_d("  calibrationInProgress: %s", calibrationInProgress ? "true" : "false");
                log_d("  calibrationStartMillis: %lu", calibrationStartMillis);
    
                time_t now = time(nullptr);
                log_d("Current time: %lu", now);
                if (eCO2_base != 0 && TVOC_base != 0 && (now - lastCalibrationTime) < (SEVEN_DAYS / 1000)) {
                    sgp.setIAQBaseline(eCO2_base, TVOC_base);
                    log_d("Restored SGP30 baseline: eCO2: %d, TVOC: %d", eCO2_base, TVOC_base);
                } else if (!calibrationInProgress) {
                    calibrationStartMillis = now;
                    preferences.begin("sgp30", false);
                    preferences.putULong("calibStartTime", now);
                    preferences.putBool("calibInProgress", true);
                    preferences.end();
                    calibrationInProgress = true;
                    log_d("Starting 12-hour calibration (baseline too old or missing)...");
                } else {
                    log_d("Resuming calibration started at: %lu", calibrationStartMillis);
                }
            } else {
                log_e("SGP30 failed to initialize after %d attempts!", maxRetries);
            }
        } else {
            log_e("SGP30 not detected! I2C error: %d", error);
        }
    

    // Initialize PMS5007
    Serial2.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
    Serial2.write(setPassiveCommand, sizeof(setPassiveCommand));
    delay(1000);
    
    uint16_t dummy_pm;
    pm5007Initialized = readPMS5007(dummy_pm, dummy_pm, dummy_pm);
    if (pm5007Initialized) {
        log_d("PM5007 initialized");
    } else {
        log_e("PM5007 not detected!");
    }

    // Initialize MICS6814
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(CO_SENSOR_PIN, INPUT);
    pinMode(NH3_SENSOR_PIN, INPUT);
    pinMode(NO2_SENSOR_PIN, INPUT);

    int co_raw = analogRead(CO_SENSOR_PIN);
    int nh3_raw = analogRead(NH3_SENSOR_PIN);
    int no2_raw = analogRead(NO2_SENSOR_PIN);

    float VREF = 3.3;
    float coVoltage = (co_raw / 4095.0) * VREF;
    float no2Voltage = (nh3_raw / 4095.0) * VREF;
    float nh3Voltage = (no2_raw / 4095.0) * VREF;

    bool validCO = (coVoltage > 0.1 && coVoltage < 2.5);
    bool validNO2 = (no2Voltage > 0.1 && no2Voltage < 2.5);
    bool validNH3 = (nh3Voltage > 0.1 && nh3Voltage < 2.5);

    mics6814Initialized = (validCO || validNO2 || validNH3);
    if (mics6814Initialized) {
        log_d("MICS6814 sensor initialized");
    } else {
        log_e("MICS6814 sensor not detected!");
    }

    // Initialize ozone sensor
    ozoneSerial->begin(9600, SERIAL_8N1, OZONE_TX_PIN, OZONE_RX_PIN);
    delay(500);
    ozoneSensorInitialized = initOzoneSensor();

    float testOzonePPM;
    ozoneSensorInitialized = ozoneSensorInitialized && readOzoneData(testOzonePPM);
    log_d("%s", ozoneSensorInitialized ? "O3 sensor initialized" : "O3 sensor failed to initialize!");

    #ifdef ENABLE_SO2_SENSOR
    so2Serial = new SoftwareSerial(4, 13);
    so2Serial->begin(9600);
    delay(500);

    so2SensorInitialized = initSO2Sensor();
    log_d("%s", so2SensorInitialized ? "SO₂ sensor initialized" : "SO₂ sensor failed to initialize!");
    #endif

    return true;
}

bool AQISensor::initOzoneSensor() {
    uint8_t disableCmd[9] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
    ozoneSerial->write(disableCmd, 9);
    delay(200);
    return true;
}

bool AQISensor::readOzoneData(float &ozonePPM) {
    const int maxRetries = 3;
    
    for (int attempt = 0; attempt < maxRetries; attempt++) {
        while (ozoneSerial->available()) {
            ozoneSerial->read();
        }
        delay(100);
        uint8_t requestCmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
        ozoneSerial->write(requestCmd, 9);
        delay(500);
        uint8_t buffer[9];
        int bytesRead = 0;
        unsigned long startTime = millis();
        while (bytesRead < 9 && millis() - startTime < 1000) {
            if (ozoneSerial->available()) {
                buffer[bytesRead++] = ozoneSerial->read();
            }
        }
        if (bytesRead != 9) {
            log_w("O3 Sensor: Attempt %d - No valid response! Received %d bytes", attempt + 1, bytesRead);
            if (attempt < maxRetries - 1) continue;
            log_e("O3 Sensor: Failed after %d attempts!", maxRetries);
            return false;
        }
        log_d("O3 Sensor received bytes: ");
        for (int i = 0; i < 9; i++) {
            log_d("0x%02X ", buffer[i]);
        }
        if (buffer[0] != 0xFF || buffer[1] != 0x17) {
            log_w("O3 Sensor: Attempt %d - Invalid response format! Header: 0x%02X 0x%02X", attempt + 1, buffer[0], buffer[1]);
            if (attempt < maxRetries - 1) continue;
            log_e("O3 Sensor: Invalid format after %d attempts!", maxRetries);
            return false;
        }
        uint8_t checksum = ~(buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6] + buffer[7]) + 1;
        if (checksum != buffer[8]) {
            log_w("O3 Sensor: Attempt %d - Checksum mismatch! Expected: 0x%02X, Received: 0x%02X", attempt + 1, checksum, buffer[8]);
            if (attempt < maxRetries - 1) continue;
            log_e("O3 Sensor: Checksum failed after %d attempts!", maxRetries);
            return false;
        }
        int ozonePPB = (buffer[4] << 8) | buffer[5];
        ozonePPM = ozonePPB / 1000.0;
        log_d("O3 Sensor: Successfully read %d PPB (%.3f PPM) on attempt %d", ozonePPB, ozonePPM, attempt + 1);
        return true;
    }
    return false;
}


#ifdef ENABLE_SO2_SENSOR
bool AQISensor::initSO2Sensor() {
    if (!so2Serial) {
        log_e("SO2 Serial not initialized!");
        return false;
    }

    delay(1000);
    uint8_t set_QA_mode[] = {0xFF, 0x01, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00, 0x83};
    so2Serial->write(set_QA_mode, sizeof(set_QA_mode));
    delay(500);

    uint8_t buffer[9];
    int bytesRead = 0;
    unsigned long startTime = millis();

    while (bytesRead < 9 && millis() - startTime < 1000) {
        if (so2Serial->available()) {
            buffer[bytesRead++] = so2Serial->read();
        }
    }

    if (bytesRead == 9 && buffer[0] == 0xFF && buffer[1] == 0x78) {
        log_d("SO2 sensor confirmed QA mode");
        return true;
    } else {
        log_e("SO2 sensor failed to confirm QA mode! Received %d bytes", bytesRead);
        return false;
    }
}

bool AQISensor::readSO2Sensor(float &so2PPM) {
    if (!so2Serial) {
        log_e("SO2 Serial not initialized!");
        return false;
    }

    uint8_t requestCmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    so2Serial->write(requestCmd, sizeof(requestCmd));
    delay(500);

    uint8_t buffer[9];
    int bytesRead = 0;
    unsigned long startTime = millis();

    while (bytesRead < 9 && millis() - startTime < 1000) {
        if (so2Serial->available()) {
            buffer[bytesRead++] = so2Serial->read();
        }
    }

    if (bytesRead > 0) {
        log_d("SO2 received %d bytes: ", bytesRead);
        for (int i = 0; i < bytesRead; i++) {
            log_d("0x%02X ", buffer[i]);
        }
    }

    if (bytesRead < 9) {
        log_e("SO₂ sensor: Received %d bytes, expected 9!", bytesRead);
        return false;
    }

    if (buffer[0] != 0xFF || buffer[1] != 0x86) {
        log_e("SO₂ sensor: Invalid response format!");
        return false;
    }

    uint8_t checksum = 0;
    for (int i = 1; i < 8; i++) {
        checksum += buffer[i];
    }
    checksum = (~checksum) + 1;
    if (checksum != buffer[8]) {
        log_e("SO₂ sensor: Checksum mismatch! Expected 0x%02X, got 0x%02X", checksum, buffer[8]);
        return false;
    }

    int so2_value = (buffer[2] << 8) | buffer[3];
    so2PPM = so2_value / 1000.0;
    return true;
}
#endif

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

void AQISensor::readMiCS6814(float &co_ppm, float &nh3_ppm, float &no2_ppm) {
    int co_raw = analogRead(CO_SENSOR_PIN);
    int nh3_raw = analogRead(NH3_SENSOR_PIN);
    int no2_raw = analogRead(NO2_SENSOR_PIN);

    co_ppm = (co_raw / 4095.0f) * 1000.0f * CO_SCALE;
    nh3_ppm = (nh3_raw / 4095.0f) * 500.0f * NH3_SCALE;
    no2_ppm = (no2_raw / 4095.0f) * 200.0f * NO2_SCALE;
}

void AQISensor::readSGP30(uint16_t &eCO2, uint16_t &TVOC) {
    if (!sgp30Initialized) {
        log_e("SGP30 not initialized!");
        eCO2 = 0;
        TVOC = 0;
        return;
    }

    sensors_event_t humidity, temp;
    if (aht20Initialized) {
        aht.getEvent(&humidity, &temp);
        uint32_t absoluteHumidity = getAbsoluteHumidity(temp.temperature, humidity.relative_humidity);
        sgp.setHumidity(absoluteHumidity);
    }

    if (!sgp.IAQmeasure()) {
        log_e("SGP30 measurement failed!");
        eCO2 = 0;
        TVOC = 0;
        return;
    }
    eCO2 = sgp.eCO2;
    TVOC = sgp.TVOC;

    updateSGP30Baseline();
}

uint32_t AQISensor::getAbsoluteHumidity(float temperature, float humidity) {
    return static_cast<uint32_t>(1000.0f * 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)));
}

void AQISensor::updateSGP30Baseline() {
    if (!sgp30Initialized || !calibrationInProgress)
        return;

    time_t now = time(nullptr);
    if ((now - calibrationStartMillis) >= (CALIBRATION_DURATION / 1000)) {
        uint16_t eCO2_base, TVOC_base;
        if (sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
            preferences.begin("sgp30", false);
            preferences.putUShort("eCO2_base", eCO2_base);
            preferences.putUShort("TVOC_base", TVOC_base);
            preferences.putULong("lastCalibTime", now);
            preferences.putBool("calibInProgress", false);
            preferences.end();
            baselineUpdated = true;
            calibrationInProgress = false;
            log_d("Stored new SGP30 baseline: eCO2: %d, TVOC: %d", eCO2_base, TVOC_base);
        } else {
            log_e("Failed to get SGP30 baseline.");
        }
    }
}

AQIData AQISensor::getData() {
    if (aht20Initialized) {
        sensors_event_t tempEvent, humidityEvent;
        aht.getEvent(&humidityEvent, &tempEvent);
        aqiData.temperature = tempEvent.temperature;
        aqiData.humidity = humidityEvent.relative_humidity;
    } else {
        aqiData.temperature = 0.0;
        aqiData.humidity = 0.0;
    }
    
    if (bmp280Initialized) {
        aqiData.pressure = bmp.readPressure() / 100.0;
    } else {
        aqiData.pressure = 0.0;
    }

    if (pm5007Initialized) {
        readPMS5007(aqiData.pm1_0, aqiData.pm2_5, aqiData.pm10_0);
    } else {
        aqiData.pm1_0 = aqiData.pm2_5 = aqiData.pm10_0 = 0;
    }
    
    if (sgp30Initialized) {
        readSGP30(aqiData.eCO2, aqiData.TVOC);
    } else {
        aqiData.eCO2 = aqiData.TVOC = 0;
    }

    if (ozoneSensorInitialized) {
        float ozonePPM;
        aqiData.ozone_ppb = readOzoneData(ozonePPM) ? ozonePPM * 1000 : 0;
    } else {
        aqiData.ozone_ppb = 0;
    }

    #ifdef ENABLE_SO2_SENSOR
    if (so2SensorInitialized) {
        float so2PPM;
        aqiData.so2_ppm = readSO2Sensor(so2PPM) ? so2PPM : 0.0;
    } else {
        aqiData.so2_ppm = 0.0;
    }
    #endif

    if (mics6814Initialized) {
        float co, nh3, no2;
        readMiCS6814(co, nh3, no2);
        aqiData.co_ppm = co;
        aqiData.nh3_ppm = nh3;
        aqiData.no2_ppm = no2;
    } else {
        aqiData.co_ppm = aqiData.nh3_ppm = aqiData.no2_ppm = 0.0;
    }
    
    return aqiData;
}