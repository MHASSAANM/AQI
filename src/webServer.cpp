#include "webServer.h"

AsyncWebServer server(80);
bool serverData = false;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Smart Meter Web Server</title>
    <link rel="stylesheet" href="styles.css">
</head>

<body>
    <div class="tab-container">

        <div class="tab-header">
            <button onclick="openTab('tab1')">WiFi Configuration</button>
            <button onclick="openTab('tab2')">Channel Checker</button>
            <button onclick="openTab('tab3')">Slot Configuration</button>
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
            <h2>Channel Checker</h2>
            <table id="dataTable">
                <thead>
                    <tr>
                        <th>Slot #</th>
                        <th>Voltage (V)</th>
                        <th>Current (A) </th>
                        <th>Power (W)</th>
                        <th>Power Factor</th>
                    </tr>
                </thead>
                <tbody></tbody>
            </table>
        </div>

        <div id="tab3" class="tab-content">
            <h2>Slot Configuration</h2>
            <form id="slotConfigForm" action="/slots" method="post">
            </form>
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
}

.tab-content {
    display: none;
    padding: 1em;
    border: 1px solid #ccc;
}

table {
    width: 100%;
}

th,
td {
    border: 1px solid #dddddd;
    text-align: center;
    padding: 0.5em 0;
}

th {
    font-weight: bold;
    background-color: #f2f2f2;
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

.form-group select {
    flex: 2;
    margin-right: 1em;
    padding: 1em;
    border: 1px solid #ccc;
    border-radius: 10px;
    box-sizing: border-box;
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

.container:before,
.container:after {
    content: "";
    display: table;
    clear: both;
}

.container:after {
    clear: both;
}

.form-group button[type="submit"] {
    margin-left: 25%;
    margin-right: 25%;
    width: 100%;
}
)rawliteral";

const char index_js[] PROGMEM = R"rawliteral(
const numOfSlots = )rawliteral" __STR(NUM_OF_SLOTS) R"rawliteral(;
sensorDataInterval = undefined;

function openTab(tabName) {
    var tabContent = document.getElementsByClassName("tab-content");
    for (var i = 0; i < tabContent.length; i++) {
        tabContent[i].style.display = "none";
    }
    document.getElementById(tabName).style.display = "block";

    clearInterval(sensorDataInterval);

    if (tabName == "tab2") {
        updateSensorData();
        sensorDataInterval = setInterval(updateSensorData, 1000);
    }

    if (tabName == "tab3") {
        updateSlotInfo();
    }
}

function generateTableRows() {
    var tableBody = document.getElementById("dataTable").getElementsByTagName('tbody')[0];

    for (var i = 1; i <= numOfSlots; i++) {
        tableBody.innerHTML += '<tr><td>' + i + '</td><td id=V' + i + '>--</td><td id=I' + i + '>--</td><td id=P' + i + '>--</td><td id=PF' + i + '>--</td></tr>';
    }
}

function generateSlotConfigRows() {
    var slotConfigForm = document.getElementById("slotConfigForm");

    for (var i = 1; i <= numOfSlots; i++) {
        slotConfigForm.innerHTML += `
            <div class="form-group">
                <label>S${i}</label>
                <select id="slot-factor-${i}" name="slot-factor-${i}">
                    <option value="0.30">30 A / 100 mA</option>
                    <option value="0.60">60 A / 100 mA</option>
                    <option value="1.00">100 A / 100 mA</option>
                    <option value="2.00">200 A / 100 mA</option>
                    <option value="3.00">300 A / 100 mA</option>
                    <option value="5.00">500 A / 100 mA</option>
                    <option value="10.00">1000 A / 100 mA</option>
                    <option value="20.00">100 A / 5 A</option>
                    <option value="40.00">200 A / 5 A</option>
                    <option value="60.00">300 A / 5 A</option>
                    <option value="100.00">500 A / 5 A</option>
                    <option value="200.00">1000 A / 5 A</option>
                    <option value="300.00">1500 A / 5 A</option>
                    <option value="400.00">2000 A / 5 A</option>
                    <option value="600.00">3000 A / 5 A</option>

                </select>
                <input type="text" id="slot-label-${i}" name="slot-label-${i}" maxlength="100" placeholder="Enter S${i} label">
            </div>
        `;
    }

    slotConfigForm.innerHTML += `
        <div class="form-group">
            <button type="submit">Update</button>
        </div>
    `;
}

function updateSensorData() {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function () {
        if (xhr.readyState == 4 && xhr.status == 200) {
            var sensorData = JSON.parse(xhr.responseText);

            for (var i = 1; i <= numOfSlots; i++) {
                document.getElementById('V' + i).textContent = sensorData['V' + i];
                document.getElementById('I' + i).textContent = sensorData['I' + i];
                document.getElementById('P' + i).textContent = sensorData['P' + i];
                document.getElementById('PF' + i).textContent = sensorData['PF' + i];
            }
        }
    };
    xhr.open("GET", "/data", true);
    xhr.send();
}

function updateSlotInfo() {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function () {
        if (xhr.readyState == 4 && xhr.status == 200) {
            console.log(xhr.responseText)
            var slotInfo = JSON.parse(xhr.responseText);

            for (var i = 1; i <= numOfSlots; i++) {
                document.getElementById('slot-label-' + i).value = slotInfo['slot-label-' + i];

                var selectElement = document.getElementById('slot-factor-' + i);
                var desiredValue = slotInfo['slot-factor-' + i];

                for (var j = 0; j < selectElement.options.length; j++) {
                    if (selectElement.options[j].value == desiredValue) {
                        selectElement.selectedIndex = j;
                        break;
                    }
                }
            }
        }
    };
    xhr.open("GET", "/slots", true);
    xhr.send();
}

document.addEventListener("DOMContentLoaded", function () {
    generateTableRows();
    generateSlotConfigRows();
    openTab('tab2');
});
)rawliteral";

/*String meterDataAsJson()
{
    String json = "{";
    for (int i = 0; i < NUM_OF_SLOTS; i++)
    {
        json += "\"V" + String(i + 1) + "\":\"" + String(meterData[i].getInstVoltage(), 2) + "\",";
        json += "\"I" + String(i + 1) + "\":\"" + String(meterData[i].getInstCurrent(), 2) + "\",";
        json += "\"P" + String(i + 1) + "\":\"" + String(meterData[i].getInstPower(), 2) + "\",";

        if (i == NUM_OF_SLOTS - 1)
        {
            json += "\"PF" + String(i + 1) + "\":\"" + String(meterData[i].getInstPf(), 2) + "\"";
        }
        else
        {
            json += "\"PF" + String(i + 1) + "\":\"" + String(meterData[i].getInstPf(), 2) + "\",";
        }
    }
    json += "}";
    log_d("%s", json.c_str());
    return json;
}

String slotLabelsAsJson()
{
    String slot_labels[NUM_OF_SLOTS];
    float slot_factors[NUM_OF_SLOTS];
    myNVS::read::slotLabels(slot_labels);
    myNVS::read::slotFactors(slot_factors);
    String json = "{";
    for (int i = 0; i < NUM_OF_SLOTS; i++)
    {
        if (i == NUM_OF_SLOTS - 1)
        {
            json += "\"slot-label-" + String(i + 1) + "\":\"" + slot_labels[i] + "\",";
            json += "\"slot-factor-" + String(i + 1) + "\":\"" + slot_factors[i] + "\"";
        }
        else
        {
            json += "\"slot-label-" + String(i + 1) + "\":\"" + slot_labels[i] + "\",";
            json += "\"slot-factor-" + String(i + 1) + "\":\"" + slot_factors[i] + "\",";
        }
    }
    json += "}";
    log_d("%s", json.c_str());
    return json;
}

String templateProcessor(const String &var)
{
    if (var == "NUM_OF_SLOTS")
    {
        return String(NUM_OF_SLOTS);
    }
    return String();
}

void myServerInitialize()
{
    // end server if already on
    server.end();

    // serve html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/html", index_html); });

    // serve css
    server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/css", styles_css); });

    // serve js
    server.on("/index.js", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/javascript", index_js); });

    // instantaneous data for channel checker
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "application/json", meterDataAsJson()); });

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

    // serve slot labels on web page
    server.on(
        "/slots", HTTP_GET, [](AsyncWebServerRequest *request)
        { request->send(200, "application/json", slotLabelsAsJson()); });

    // retrieve slot labels from web page
    server.on(
        "/slots", HTTP_POST, [](AsyncWebServerRequest *request)
        {
            String slot_labels[NUM_OF_SLOTS];
            float slot_factors[NUM_OF_SLOTS];
            myNVS::read::slotLabels(slot_labels);
            myNVS::read::slotFactors(slot_factors);
            for (int i = 0; i < NUM_OF_SLOTS; i++)
            {
                String labelParam = "slot-label-" + String(i + 1);
                if (request->hasParam(labelParam, true))
                {
                    slot_labels[i] = request->getParam(labelParam, true)->value();
                    Serial.println(slot_labels[i]);
                }
                String factorParam = "slot-factor-" + String(i + 1);
                if (request->hasParam(factorParam, true))
                {
                    slot_factors[i] = atof(request->getParam(factorParam, true)->value().c_str());
                    Serial.println(slot_factors[i]);
                }
                meterData[i].setLabel(slot_labels[i]);
                meterData[i].setScalingFactor(slot_factors[i]);
            }
            myNVS::write::slotLabels(slot_labels);
            myNVS::write::slotFactors(slot_factors);
            request->redirect("/"); });

    server.onNotFound([](AsyncWebServerRequest *request)
                      { request->send(404, "text/plain", "Not found"); });

    server.begin();
}*/