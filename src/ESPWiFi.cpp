#include <ESPWiFi.h>

void onWifiScan(WiFiEvent_t event, WiFiEventInfo_t info)
{
    static uint8_t scanFailureCountdown = WIFI_SCAN_FAILURE_LIMIT;
    uint8_t numOfScans = info.scan_done.number;

    if (numOfScans > 0)
    {
        scanFailureCountdown = WIFI_SCAN_FAILURE_LIMIT;
        log_d("Number of WiFi Scan results: %d", numOfScans);
    }
    else
    {
        if (--scanFailureCountdown)
            log_e("Scan Failure shutdown after %d fails\n", scanFailureCountdown);
        else
        {
            log_e("Scan Failure Reset\n");
            ESP.restart();
        }
    }
}

// TODO: optimize wifi reconnection time.
/**
 * This function initializes the ESP wifi module with the requisite
 * configurations. It attempts to connect to a default SSID and password and
 * also updates the vector for APs in the WiFiMulti class.
 */
bool ESP_WiFi::init()
{
    WiFi.mode(WIFI_AP_STA);
    WiFi.onEvent(onWifiScan, SYSTEM_EVENT_SCAN_DONE);
    log_i("updating AP list from SD card ");
    this->update_APs();
    return true;
}

/**
 * This function adds new APs to the WiFiMulti class. This should only be called
 * on the first initialization as it may result in multiple instances of the
 * same credentials in the vector.
 */
void ESP_WiFi::update_APs()
{
    int i = 0;
    // String SSIDs[10]; String Passwords[10];
    if (flags[sd_f])
    {
        storage.return_APList(SSID_List, Password_List);
    }
    else
    {
        SSID_List[0] = DEFAULT_SSID;
        Password_List[0] = DEFAULT_PASSWORD;
    }
    access_points = new WiFiMulti();
    while (!SSID_List[i].isEmpty() && i < 10)
    {
        access_points->addAP(SSID_List[i].c_str(), Password_List[i].c_str());
        log_v("Added AP ");
        log_v("SSID: %s ", SSID_List[i].c_str());
        log_v("Password: %s ", Password_List[i].c_str());
        i++;
    }
    credential_length = i;
    log_v("credential length: %u ", credential_length);
}

/**
 * This function adds a new connection to the vector in the WiFiMulti class and
 * the text file for APs in the Sd card. In the situation that the SSID sent is
 * the same as one that already exists, the function will either fail if the
 * password is the same or update the specific entry if not.
 */
bool ESP_WiFi::create_new_connection(const char *SSID, const char *Password)
{
    // log_i("Wifi Disconnect");
    WiFi.disconnect();
    vTaskDelay(10);
    // log_i("Wifi Station Mode");

    log_d("connecting to new AP");
    vTaskDelay(5);
    // log_i("Wifi begin at new SSID");
    WiFi.begin(SSID, Password);
    vTaskDelay(5);

    int32_t timer = 0;
    vTaskDelay(5);

    // log_i("try connection with WiFi");
    while (!WiFi.isConnected()) // Checking if credentials conect with an AP
    {
        vTaskDelay(1000);
        if (timer > 15) // Timeout
        {
            log_e("Failed to connect. Please check SSID and Password");
            return false;
        }
        timer += 1;
    }

    // Connection has been established
    log_v("credential_length = %u ", credential_length);
    // log_i("check credential data");
    if (credential_length >= 10) // Check if AP data limit reached
    {
        // clear all data and write single AP along with default
        int i = 0;
        log_v("recreating AP data ");
        while (!SSID_List[i].isEmpty() || !Password_List[i].isEmpty())
        {
            log_i("Clear AP list");
            SSID_List[i].clear();
            Password_List[i].clear();
            i++;
        }
        i = 0;
        SSID_List[0] = DEFAULT_SSID;
        Password_List[0] = DEFAULT_PASSWORD;
        SSID_List[1] = (String)SSID;
        Password_List[1] = (String)Password;
        credential_length = 2;
        vTaskDelay(5);
        // log_i("Try to make access points");
        if (remake_access_points())
        {
            if (flags[sd_f])
            {
                // log_i("flag true & try to store AP in storage");
                if (storage.rewrite_storage_APs(SSID_List, Password_List))
                    return true;
                else
                {
                    log_e("Failed to rewrite storage APs");
                    return false;
                }
            }
            return true;
        }
        else
        {
            log_e("Failed to remake access points");
            return false;
        }
    }

    for (int i = 0; i < 10; i++) // Check if the SSID matches with a pre-existing entry
    {
        if (SSID_List[i].isEmpty())
        {
            break;
        }
        Serial.println(SSID_List[i]);
        if ((String)SSID == SSID_List[i])
        {
            log_d("SSID match found ");
            if ((String)Password == Password_List[i]) // Same credentials were entered
            {
                log_e("Credentials already exist ");
                return false;
            }
            else // Password was different
            {
                log_i("Updating Password Only ");
                // add code to update all lists within if condition (must be
                // type bool)
                Password_List[i] = (String)Password;
                if (remake_access_points())
                {
                    if (flags[sd_f])
                    {
                        if (storage.rewrite_storage_APs(SSID_List, Password_List))
                            return true;
                        else
                            log_e("Error in rewriting storage APs ");
                    }
                }
                else
                    log_e("error in remaking access points ");
            }
        }
    }

    if (flags[sd_f])
    {
        // log_i("New AP store");
        vTaskDelay(5);
        storage.write_AP(SSID, Password); // Add to storage
        vTaskDelay(5);
        access_points->addAP(SSID, Password); // Add to WiFiMulti Class
        vTaskDelay(5);
        credential_length++; // Add to credential length
    }
    return true;
}

/**
 * This function serves to destroy and recreate a WiFiMulti class in order to
 * change the list of APs. This is done as the AP list (vector) is a private
 * variable
 */
bool ESP_WiFi::remake_access_points()
{
    delete access_points;
    access_points = new WiFiMulti();
    int i = 0;
    while (!SSID_List[i].isEmpty() && i < 10)
    {
        vTaskDelay(5);
        access_points->addAP(SSID_List[i].c_str(), Password_List[i].c_str());
        log_v("%s and %s added ", SSID_List[i].c_str(), Password_List[i].c_str());
        i++;
    }
    return true;
}

/**
 * This function sets up the soft access point and hosts web server.
 */
bool ESP_WiFi::start_soft_ap()
{
    if (WiFi.softAP(sensorID.c_str(), SOFT_AP_PASSWORD))
    {
        log_d("Soft AP IP: %s", WiFi.softAPIP().toString().c_str());
        //myServerInitialize();
        //log_d("Web server initalized.");
        return true;
    }
    return false;
}

/**
 * This function is used to connect to the nearest available access point.
 */
bool ESP_WiFi::connect_to_nearest()
{
    if (access_points->run(15000) == WL_CONNECTED)
    {
        log_i("Connection established ");
        return true;
    }
    else
    {
        log_e("Connection timed out ");
        return false;
    }
}

/**
 * @brief This function is a wrapper function which checks the status of the
 * wifi connection. It is designed to be used in a loop. Can be altered
 * according to requirements.
 *
 * @return previous connection exists
 * @return previous connection doesn't exist
 */
bool ESP_WiFi::check_connection()
{
    if (WiFi.isConnected()) // Check WiFi connection
    {
        return true;
    }
    else
    {
        log_e("WiFi not connected. Establishing connection ");
        if (this->connect_to_nearest())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

ESP_WiFi wf;