
#include "AQISensor.h"

// Constructor
AQISensor::AQISensor() : so2Serial(25, 26), aqiData{0.0, 0.0, 0.0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0} {} 

// Initialize sensors
bool AQISensor::init() {
    if (!aht.begin()) {
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
    log_d("PM5007 initialized");

    mySerial.begin(9600, SERIAL_8N1, 13, 12);
    log_d("Ozone sensor initialized");

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(32, INPUT);
    pinMode(34, INPUT);
    pinMode(35, INPUT);
    log_d("MICS6814 sensor initialized");


    // Initialize SO2 sensor
    so2Serial.begin(9600);
    delay(2000);
    uint8_t set_QA_mode[] = {0xFF, 0x01, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00, 0x83};
    so2Serial.write(set_QA_mode, sizeof(set_QA_mode));
    delay(1000);
    log_d("SO2 sensor initialized");

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

// Read ozone sensor data
float AQISensor::readOzone() {
    uint8_t requestCmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    mySerial.write(requestCmd, 9);
    delay(500);

    uint8_t buffer[9];
    if (mySerial.available() >= 9) {
        mySerial.readBytes(buffer, 9);
        if (buffer[0] == 0xFF && buffer[1] == 0x17) {
            return ((buffer[4] << 8) | buffer[5]);
        }
    }
    return 0.0;
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

// Read SO2 sensor data
uint16_t AQISensor::readSO2() {
    uint8_t read_SO2[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    so2Serial.write(read_SO2, sizeof(read_SO2));
    delay(500);

    if (so2Serial.available() >= 9) {
        uint8_t response[9];
        so2Serial.readBytes(response, 9);

        if (response[0] == 0xFF && response[1] == 0x86) {
            return (response[2] << 8) | response[3];
        }
    }
    return 0; // Return 0 if no valid data
}

// Get AQI data
AQIData AQISensor::getData() {
    sensors_event_t tempEvent, humidityEvent;
    aht.getEvent(&humidityEvent, &tempEvent);
    aqiData.temperature = tempEvent.temperature;
    aqiData.humidity = humidityEvent.relative_humidity;
    aqiData.pressure = bmp.readPressure() / 100.0;

    readPMS5007(aqiData.pm1_0, aqiData.pm2_5, aqiData.pm10_0);
    aqiData.ozone_ppb = readOzone();
    readMICS6814(aqiData.co_ppm, aqiData.no2_ppm, aqiData.nh3_ppm);
    aqiData.so2_ppm = readSO2();
    
    return aqiData;
}



