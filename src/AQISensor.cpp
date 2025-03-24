#include "AQISensor.h"

AQISensor::AQISensor() :  /*ozoneSerial(&Serial1) , so2Serial(nullptr),*/ aqiData{0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0}, baselineUpdated(false) {}

TwoWire SGP30_Wire = TwoWire(1);

bool AQISensor::init() {

    /*ozoneSerial->begin(9600, SERIAL_8N1, OZONE_RX_PIN, OZONE_TX_PIN);
    delay(100);
    ozoneSensorInitialized = initOzoneSensor();

    float testOzonePPM;
    ozoneSensorInitialized = ozoneSensorInitialized && readOzoneData(testOzonePPM);
    log_d("%s", ozoneSensorInitialized ? "O3 sensor initialized" : "O3 sensor failed to initialize!");*/

    SGP30_Wire.begin(27, 14);
    delay(100);

    sgp30Initialized = sgp.begin(&SGP30_Wire);
    if (!sgp30Initialized) {
        log_e("Could not find SGP30 sensor on custom I2C bus!");
    } else {
        log_d("SGP30 initialized on separate I2C bus (GPIO27, GPIO14)");
    }

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

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(32, INPUT);
    pinMode(34, INPUT);
    pinMode(35, INPUT);

    float co, no2, nh3;
    readMICS6814(co, no2, nh3);
    mics6814Initialized = (co > 0.0 || no2 > 0.0 || nh3 > 0.0);
    if (mics6814Initialized) {
        log_d("MICS6814 sensor initialized");
    } else {
        log_e("MICS6814 sensor not detected!");
    }



    if (sgp30Initialized) {
        preferences.begin("sgp30", false);
        uint16_t eCO2_base = preferences.getUShort("eCO2_base", 0);
        uint16_t TVOC_base = preferences.getUShort("TVOC_base", 0);
        unsigned long lastCalibrationTime = preferences.getULong("lastCalibTime", 0);
        preferences.end();

        unsigned long timeSinceLastCalibration = millis() - lastCalibrationTime;
        if (eCO2_base != 0 && TVOC_base != 0 && timeSinceLastCalibration < SEVEN_DAYS) {
            sgp.setIAQBaseline(eCO2_base, TVOC_base);
            log_d("Restored SGP30 baseline: eCO2: %d, TVOC: %d", eCO2_base, TVOC_base);
        } else {
            log_d("Baseline too old or not found. Running 12-hour calibration...");
            startTime = millis();
        }
    }


    /*so2Serial = new SoftwareSerial(25, 26);
    so2Serial->begin(9600);
    delay(500);

    so2SensorInitialized = initSO2Sensor();
    log_d("%s", so2SensorInitialized ? "SO₂ sensor initialized" : "SO₂ sensor failed to initialize!");*/

    return true;
}

/*bool AQISensor::initOzoneSensor() {
    uint8_t disableCmd[9] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
    ozoneSerial->write(disableCmd, 9);
    delay(100);  // Allow time for mode switch
    return true; // Assume command was sent correctly
}



bool AQISensor::readOzoneData(float &ozonePPM) {
    uint8_t requestCmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    ozoneSerial->write(requestCmd, 9);
    delay(100);  // Wait for sensor response

    uint8_t buffer[9];
    int bytesRead = 0;
    unsigned long startTime = millis();

    while (bytesRead < 9 && millis() - startTime < 1000) {
        if (ozoneSerial->available()) {
            buffer[bytesRead++] = ozoneSerial->read();
        }
    }

    if (bytesRead != 9) {
        log_e("O3 Sensor: No valid response! Check wiring.");
        return false; // No valid data received
    }

    if (buffer[0] != 0xFF || buffer[1] != 0x17) {
        log_e("O3 Sensor: Invalid response format!");
        return false;
    }

    uint8_t checksum = ~(buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6] + buffer[7]) + 1;
    if (checksum != buffer[8]) {
        log_e("O3 Sensor: Checksum mismatch!");
        return false;
    }

    // Extract ozone concentration
    int ozonePPB = (buffer[4] << 8) | buffer[5];
    ozonePPM = ozonePPB / 1000.0;

    return true; // Sensor responded correctly
}*/

/*bool AQISensor::initSO2Sensor() {
    if (!so2Serial) return false;

    uint8_t set_QA_mode[] = {0xFF, 0x01, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00, 0x83};
    so2Serial->write(set_QA_mode, sizeof(set_QA_mode));
    delay(500);

    return true; // Assuming command was sent correctly
}


bool AQISensor::readSO2Sensor(float &so2PPM) {
    if (!so2Serial) return false;

    uint8_t requestCmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    so2Serial->write(requestCmd, sizeof(requestCmd));
    delay(500);

    if (so2Serial->available() >= 9) {
        uint8_t response[9];
        so2Serial->readBytes(response, 9);

        if (response[0] == 0xFF && response[1] == 0x86) {
            int so2_value = (response[2] << 8) | response[3];
            so2PPM = so2_value / 1000.0; // Convert to ppm
            return true;
        } else {
            log_e("SO₂ sensor: Invalid response format!");
            return false;
        }
    } else {
        log_e("SO₂ sensor: No valid response received!");
        return false;
    }
}*/





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

// Read MICS6814 sensor data
void AQISensor::readMICS6814(float &co, float &no2, float &nh3) {
    sensors_event_t tempEvent, humidityEvent;
    aht.getEvent(&humidityEvent, &tempEvent);
    float currentTemp = tempEvent.temperature;
    float currentHumidity = humidityEvent.relative_humidity;

    int coRaw = analogRead(32);
    int no2Raw = analogRead(34);
    int nh3Raw = analogRead(35);

    float VREF = 3.3;
    float coVoltage = (coRaw / 4095.0) * VREF;
    float no2Voltage = (no2Raw / 4095.0) * VREF;
    float nh3Voltage = (nh3Raw / 4095.0) * VREF;

    float RL_CO = 47000;
    float RL_NO2 = 47000;
    float RL_NH3 = 47000;

    float Rs_CO = RL_CO * ((VREF - coVoltage) / coVoltage);
    float Rs_NO2 = RL_NO2 * ((VREF - no2Voltage) / no2Voltage);
    float Rs_NH3 = RL_NH3 * ((VREF - nh3Voltage) / nh3Voltage);

    float CO_R0 = 2200.0;
    float NO2_R0 = 261500.0;
    float NH3_R0 = 3600.0;

    float kT_CO = -0.015, kH_CO = 0.01;
    float kT_NO2 = 0.02, kH_NO2 = -0.02;
    float kT_NH3 = -0.01, kH_NH3 = 0.015;

    float temp_correction = currentTemp - 25;
    float humidity_correction = currentHumidity - 50;

    Rs_CO *= exp(kT_CO * temp_correction + kH_CO * humidity_correction);
    Rs_NO2 *= exp(kT_NO2 * temp_correction + kH_NO2 * humidity_correction);
    Rs_NH3 *= exp(kT_NH3 * temp_correction + kH_NH3 * humidity_correction);

    // Prevent division by near-zero Rs values
    if (Rs_CO < 1.0) Rs_CO = 1.0;

    co = 10.0 * pow((Rs_CO / CO_R0), -1.5);
    no2 = 0.1 * pow((Rs_NO2 / NO2_R0), 1.2);
    nh3 = 1.0 * pow((Rs_NH3 / NH3_R0), -1.0);

    // Cap ppm values
    co = fmin(co, 1000.0);
    no2 = fmin(no2, 10.0);
    nh3 = fmin(nh3, 500.0);


}


// Read SGP30 sensor data
void AQISensor::readSGP30(uint16_t &eCO2, uint16_t &TVOC) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    uint32_t absoluteHumidity = getAbsoluteHumidity(temp.temperature, humidity.relative_humidity);
    sgp.setHumidity(absoluteHumidity);

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
// Get absolute humidity
uint32_t AQISensor::getAbsoluteHumidity(float temperature, float humidity) {
    return static_cast<uint32_t>(1000.0f * 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)));
}
// Update SGP30 baseline
void AQISensor::updateSGP30Baseline() {
    if (!baselineUpdated && millis() - startTime >= CALIBRATION_DURATION) {
        uint16_t eCO2_base, TVOC_base;
        if (sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
            preferences.begin("sgp30", false);
            preferences.putUShort("eCO2_base", eCO2_base);
            preferences.putUShort("TVOC_base", TVOC_base);
            preferences.putULong("lastCalibTime", millis());
            preferences.end();
            baselineUpdated = true;
            log_d("Stored new SGP30 baseline: eCO2: %d, TVOC: %d", eCO2_base, TVOC_base);
        } else {
            log_e("Failed to get SGP30 baseline.");
        }
    }
}

// Get AQI data
AQIData AQISensor::getData() {
    sensors_event_t tempEvent, humidityEvent;
    aht.getEvent(&humidityEvent, &tempEvent);
    aqiData.temperature = tempEvent.temperature;
    aqiData.humidity = humidityEvent.relative_humidity;
    aqiData.pressure = bmp.readPressure() / 100.0;

    readPMS5007(aqiData.pm1_0, aqiData.pm2_5, aqiData.pm10_0);
    readMICS6814(aqiData.co_ppm, aqiData.no2_ppm, aqiData.nh3_ppm);
    readSGP30(aqiData.eCO2, aqiData.TVOC);

    /*if (ozoneSensorInitialized) {
        float ozonePPM;
        aqiData.ozone_ppb = readOzoneData(ozonePPM) ? ozonePPM * 1000 : 0;
    }*/

   /* if (so2SensorInitialized) {
        float so2PPM;
        aqiData.so2_ppm = readSO2Sensor(so2PPM) ? so2PPM : 0.0;
    }*/
    
    return aqiData;
}



