#include "webServer.h"
#include "rtc.h"

AsyncWebServer server(80);
bool serverData = false;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AQI Meter Web Server</title>
    <link rel="stylesheet" href="styles.css">
</head>

<body>
    <div class="tab-container">

        <div class="tab-header">
            <button onclick="openTab('tab1')">WiFi Configuration</button>
            <button onclick="openTab('tab2')">AQI Sensor Data</button>
        </div>

        <div id="tab1" class="tab-content">
            <h2>WiFi Configuration</h2>
            <form action="/wifi" method="post">
                <div class="form-group">
                    <label for="HTML_SSID">SSID:</label>
                    <input type="text" id="HTML_SSID" name="HTML_SSID" value="" required>
                </div>
                <div class="form-group">
                    <label for="HTML_PASS">Password:</label>
                    <input type="password" id="HTML_PASS" name="HTML_PASS" value="" required>
                </div>
                <div class="form-group">
                    <button type="submit">Connect</button>
                </div>
            </form>
        </div>

        <div id="tab2" class="tab-content">
            <h2>AQI Sensor Data</h2>
            <div class="sensor-grid">
                <div class="sensor-card">
                    <h3>Temperature & Humidity</h3>
                    <div class="sensor-data">
                        <div class="data-item">
                            <span class="label">Temperature:</span>
                            <span class="value" id="temperature">--</span>
                            <span class="unit">°C</span>
                        </div>
                        <div class="data-item">
                            <span class="label">Humidity:</span>
                            <span class="value" id="humidity">--</span>
                            <span class="unit">%</span>
                        </div>
                        <div class="data-item">
                            <span class="label">Pressure:</span>
                            <span class="value" id="pressure">--</span>
                            <span class="unit">hPa</span>
                        </div>
                    </div>
                </div>

                <div class="sensor-card">
                    <h3>Particulate Matter</h3>
                    <div class="sensor-data">
                        <div class="data-item">
                            <span class="label">PM1.0:</span>
                            <span class="value" id="pm1_0">--</span>
                            <span class="unit">μg/m³</span>
                        </div>
                        <div class="data-item">
                            <span class="label">PM2.5:</span>
                            <span class="value" id="pm2_5">--</span>
                            <span class="unit">μg/m³</span>
                        </div>
                        <div class="data-item">
                            <span class="label">PM10:</span>
                            <span class="value" id="pm10_0">--</span>
                            <span class="unit">μg/m³</span>
                        </div>
                    </div>
                </div>

                <div class="sensor-card">
                    <h3>Gas Sensors</h3>
                    <div class="sensor-data">
                        <div class="data-item">
                            <span class="label">CO:</span>
                            <span class="value" id="co">--</span>
                            <span class="unit">ppm</span>
                        </div>
                        <div class="data-item">
                            <span class="label">NO2:</span>
                            <span class="value" id="no2">--</span>
                            <span class="unit">ppm</span>
                        </div>
                        <div class="data-item">
                            <span class="label">NH3:</span>
                            <span class="value" id="nh3">--</span>
                            <span class="unit">ppm</span>
                        </div>
                        <div class="data-item">
                            <span class="label">Ozone:</span>
                            <span class="value" id="ozone">--</span>
                            <span class="unit">ppb</span>
                        </div>
                    </div>
                </div>

                <div class="sensor-card">
                    <h3>Air Quality</h3>
                    <div class="sensor-data">
                        <div class="data-item">
                            <span class="label">eCO2:</span>
                            <span class="value" id="eco2">--</span>
                            <span class="unit">ppm</span>
                        </div>
                        <div class="data-item">
                            <span class="label">TVOC:</span>
                            <span class="value" id="tvoc">--</span>
                            <span class="unit">ppb</span>
                        </div>
                    </div>
                </div>

                <div class="sensor-status">
                    <h3>Sensor Status</h3>
                    <div class="status-grid">
                        <div class="status-item">
                            <span class="status-label">AHT20:</span>
                            <span class="status-value" id="aht20_status">--</span>
                        </div>
                        <div class="status-item">
                            <span class="status-label">BMP280:</span>
                            <span class="status-value" id="bmp280_status">--</span>
                        </div>
                        <div class="status-item">
                            <span class="status-label">PMS5007:</span>
                            <span class="status-value" id="pms5007_status">--</span>
                        </div>
                        <div class="status-item">
                            <span class="status-label">MiCS6814:</span>
                            <span class="status-value" id="mics6814_status">--</span>
                        </div>
                        <div class="status-item">
                            <span class="status-label">SGP30:</span>
                            <span class="status-value" id="sgp30_status">--</span>
                        </div>
                        <div class="status-item">
                            <span class="status-label">Ozone:</span>
                            <span class="status-value" id="ozone_status">--</span>
                        </div>
                    </div>
                </div>
            </div>
        </div>

    </div>

    <script src="index.js"></script>
</body>

</html>
)rawliteral";

const char styles_css[] PROGMEM = R"rawliteral(
body {
    font-family: Arial, sans-serif;
    font-size: 16px;
    background-color: #faf9f6;
    margin: 0;
}

.tab-container {
    width: 100%;
    margin: 0 auto;
}

.tab-header {
    overflow: hidden;
    border: 1px solid #ccc;
}

.tab-header button {
    background-color: inherit;
    color: black;
    float: left;
    border: none;
    cursor: pointer;
    padding: 1em;
    font-size: 1em;
    transition: 0.3s;
    font-weight: bold;
    width: 50%;
}

.tab-header button:hover {
    background-color: #f2f2f2;
}

.tab-header button.active {
    background-color: #4CAF50;
    color: white;
}

.tab-content {
    display: none;
    padding: 1em;
    border: 1px solid #ccc;
}

.form-group {
    display: flex;
    align-items: center;
    margin-bottom: 1em;
}

.form-group label {
    flex: 1;
    font-weight: bold;
    margin-right: 1em;
    color: black;
}

.form-group input {
    flex: 6;
    margin-right: 1em;
    padding: 1em;
    border: 1px solid #ccc;
    border-radius: 20px;
    box-sizing: border-box;
}

.form-group button {
    background-color: #4CAF50;
    color: white;
    padding: 1em;
    border: none;
    font-size: 1em;
    border-radius: 20px;
    cursor: pointer;
    transition: background-color 0.3s ease;
}

.form-group button:hover {
    background-color: #45a049;
}

h2 {
    text-align: center;
    margin-bottom: 30px;
    color: black;
}

h3 {
    text-align: center;
    margin-bottom: 20px;
    color: #333;
}

.sensor-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 20px;
    margin-top: 20px;
}

.sensor-card {
    background-color: white;
    border: 2px solid #ddd;
    border-radius: 10px;
    padding: 20px;
    box-shadow: 0 2px 5px rgba(0,0,0,0.1);
}

.sensor-data {
    display: flex;
    flex-direction: column;
    gap: 15px;
}

.data-item {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px;
    background-color: #f8f9fa;
    border-radius: 5px;
}

.label {
    font-weight: bold;
    color: #333;
}

.value {
    font-size: 1.2em;
    font-weight: bold;
    color: #2c3e50;
}

.unit {
    color: #666;
    font-size: 0.9em;
}

.sensor-status {
    grid-column: 1 / -1;
    background-color: white;
    border: 2px solid #ddd;
    border-radius: 10px;
    padding: 20px;
    box-shadow: 0 2px 5px rgba(0,0,0,0.1);
}

.status-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 10px;
}

.status-item {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 8px 12px;
    background-color: #f8f9fa;
    border-radius: 5px;
}

.status-label {
    font-weight: bold;
    color: #333;
}

.status-value {
    font-weight: bold;
    padding: 4px 8px;
    border-radius: 3px;
}

.status-on {
    background-color: #d4edda;
    color: #155724;
}

.status-off {
    background-color: #f8d7da;
    color: #721c24;
}

@media (max-width: 768px) {
    .tab-header button {
        width: 100%;
        float: none;
    }
    
    .sensor-grid {
        grid-template-columns: 1fr;
    }
}
)rawliteral";

const char index_js[] PROGMEM = R"rawliteral(
let sensorDataInterval = undefined;

function openTab(tabName) {
    var tabContent = document.getElementsByClassName("tab-content");
    var tabButtons = document.getElementsByClassName("tab-header")[0].getElementsByTagName("button");
    
    // Hide all tab content
    for (var i = 0; i < tabContent.length; i++) {
        tabContent[i].style.display = "none";
    }
    
    // Remove active class from all buttons
    for (var i = 0; i < tabButtons.length; i++) {
        tabButtons[i].classList.remove("active");
    }
    
    // Show selected tab
    document.getElementById(tabName).style.display = "block";
    
    // Add active class to clicked button
    event.target.classList.add("active");

    // Clear existing interval
    clearInterval(sensorDataInterval);

    if (tabName == "tab2") {
        updateSensorData();
        updateSensorStatus();
        sensorDataInterval = setInterval(function() {
            updateSensorData();
            updateSensorStatus();
        }, 2000);
    }
}

function updateSensorData() {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function () {
        if (xhr.readyState == 4 && xhr.status == 200) {
            var sensorData = JSON.parse(xhr.responseText);

            // Update sensor values
            document.getElementById('temperature').textContent = sensorData.temperature;
            document.getElementById('humidity').textContent = sensorData.humidity;
            document.getElementById('pressure').textContent = sensorData.pressure;
            document.getElementById('pm1_0').textContent = sensorData.pm1_0;
            document.getElementById('pm2_5').textContent = sensorData.pm2_5;
            document.getElementById('pm10_0').textContent = sensorData.pm10_0;
            document.getElementById('co').textContent = sensorData.co_ppm;
            document.getElementById('no2').textContent = sensorData.no2_ppm;
            document.getElementById('nh3').textContent = sensorData.nh3_ppm;
            document.getElementById('ozone').textContent = sensorData.ozone_ppb;
            document.getElementById('eco2').textContent = sensorData.eCO2;
            document.getElementById('tvoc').textContent = sensorData.TVOC;
        }
    };
    xhr.open("GET", "/data", true);
    xhr.send();
}

function updateSensorStatus() {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function () {
        if (xhr.readyState == 4 && xhr.status == 200) {
            var statusData = JSON.parse(xhr.responseText);

            // Update status indicators
            updateStatusElement('aht20_status', statusData.aht20);
            updateStatusElement('bmp280_status', statusData.bmp280);
            updateStatusElement('pms5007_status', statusData.pms5007);
            updateStatusElement('mics6814_status', statusData.mics6814);
            updateStatusElement('sgp30_status', statusData.sgp30);
            updateStatusElement('ozone_status', statusData.ozone);
        }
    };
    xhr.open("GET", "/status", true);
    xhr.send();
}

function updateStatusElement(elementId, status) {
    var element = document.getElementById(elementId);
    element.textContent = status ? "On" : "Off";
    element.className = "status-value " + (status ? "status-on" : "status-off");
}

document.addEventListener("DOMContentLoaded", function () {
    openTab('tab2');
});
)rawliteral";

String aqiDataAsJson()
{
    AQIData data = aqiSensor.getData();
    String json = "{";
    json += "\"temperature\":\"" + String(data.temperature, 2) + "\",";
    json += "\"humidity\":\"" + String(data.humidity, 2) + "\",";
    json += "\"pressure\":\"" + String(data.pressure, 2) + "\",";
    json += "\"pm1_0\":\"" + String(data.pm1_0) + "\",";
    json += "\"pm2_5\":\"" + String(data.pm2_5) + "\",";
    json += "\"pm10_0\":\"" + String(data.pm10_0) + "\",";
    json += "\"co_ppm\":\"" + String(data.co_ppm, 2) + "\",";
    json += "\"no2_ppm\":\"" + String(data.no2_ppm, 2) + "\",";
    json += "\"nh3_ppm\":\"" + String(data.nh3_ppm, 2) + "\",";
    json += "\"ozone_ppb\":\"" + String(data.ozone_ppb, 2) + "\",";
    json += "\"eCO2\":\"" + String(data.eCO2) + "\",";
    json += "\"TVOC\":\"" + String(data.TVOC) + "\"";
    #ifdef ENABLE_SO2_SENSOR
    json += ",\"so2_ppm\":\"" + String(data.so2_ppm, 2) + "\"";
    #endif
    json += "}";
    return json;
}

String sensorStatusAsJson()
{
    String json = "{";
    json += "\"aht20\":" + String(aqiSensor.aht20Initialized ? "true" : "false") + ",";
    json += "\"bmp280\":" + String(aqiSensor.bmp280Initialized ? "true" : "false") + ",";
    json += "\"pms5007\":" + String(aqiSensor.pm5007Initialized ? "true" : "false") + ",";
    json += "\"mics6814\":" + String(aqiSensor.mics6814Initialized ? "true" : "false") + ",";
    json += "\"sgp30\":" + String(aqiSensor.sgp30Initialized ? "true" : "false") + ",";
    json += "\"ozone\":" + String(aqiSensor.ozoneSensorInitialized ? "true" : "false");
    #ifdef ENABLE_SO2_SENSOR
    json += ",\"so2\":" + String(aqiSensor.so2SensorInitialized ? "true" : "false");
    #endif
    json += "}";
    return json;
}

void myServerInitialize()
{
    // end server if already on
    server.end();

    // serve html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/html", index_html); });

    // serve css
    server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/css", styles_css); });

    // serve js
    server.on("/index.js", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/javascript", index_js); });

    // instantaneous data for AQI sensors
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "application/json", aqiDataAsJson()); });

    // sensor status
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "application/json", sensorStatusAsJson()); });

    // retrieve wifi credentials from web page
    server.on(
        "/wifi", HTTP_POST, [](AsyncWebServerRequest *request)
        {
            if (request->hasParam(HTML_WIFI_SSID_ID, true))
            {
                serverWifiCreds[0] = request->getParam(HTML_WIFI_SSID_ID, true)->value();
            }

            if (request->hasParam(HTML_WIFI_PASS_ID, true))
            {
                serverWifiCreds[1] = request->getParam(HTML_WIFI_PASS_ID, true)->value();
            }

            for (int i = 0; i < 2; i++)
            {
                Serial.println(serverWifiCreds[i]);
            }
            serverData = true;
            request->redirect("/"); });

    server.onNotFound([](AsyncWebServerRequest *request)
                      { request->send(404, "text/plain", "Not found"); });

    server.begin();
}
