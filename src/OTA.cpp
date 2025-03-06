#include "OTA.h"

/**
 * @brief downloadUpdate downloads the updated version of firmware
 * by creating a client and updates the ESP after the whole file is
 * downloaded.
 *
 * @return true if latest update is installed
 */
bool downloadUpdate()
{
    String currentVersion = String(FIRMWARE_VERSION);

    if (latestVersion == "")
    {
        log_d("Latest version not defined in Firebase! Skipping update...");
        return true;
    }

    if (currentVersion == latestVersion)
    {
        log_d("Already running latest version [%s]", latestVersion);
        return true;
    }

    log_d("Trying to update from version %s to %s...", currentVersion, latestVersion);

    String url = CLOUD_HOST_URL + latestVersion + ".bin";

    HTTPClient http;
    http.setTimeout(15000);

    for (int retryCount = 0; retryCount < OTA_UPDATE_FAILURE_LIMIT; retryCount++)
    {
        log_d("[HTTP] Download attempt %d of %d...", retryCount + 1, OTA_UPDATE_FAILURE_LIMIT);

        if (!http.begin(url))
        {
            log_e("Unable to begin HTTP connection");
        }
        else
        {
            int httpCode = http.GET();
            log_d("[HTTP] GET... code: %d", httpCode);

            if (httpCode != HTTP_CODE_OK)
            {
                log_e("HTTP GET request failed with status code %d", httpCode);
            }
            else
            {
                int contentLength = http.getSize();
                log_d("contentLength : %d", contentLength);

                if (contentLength <= 0)
                {
                    log_e("There was no content in the response!");
                }
                else
                {
                    if (!Update.begin(contentLength))
                    {
                        log_e("Not enough space to begin OTA!");
                    }
                    else
                    {
                        WiFiClient &stream = http.getStream();
                        log_d("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
                        size_t written = Update.writeStream(stream);

                        if (written == contentLength)
                            log_d("Written : %zu successfully", written);
                        else
                            log_e("Written only : %zu/%d.", written, contentLength);

                        if (!Update.end())
                        {
                            log_e("Error Occurred. Error # %u", Update.getError());
                        }
                        else
                        {
                            log_d("OTA done!");

                            if (!Update.isFinished())
                            {
                                log_e("Update not finished. Something went wrong!");
                            }
                            else
                            {
                                log_d("Update successfully completed. Rebooting.\n");
                                ESP.restart();
                                return true;
                            }
                        }
                    }
                }
            }

            http.end();
        }

        int retryDelay = pow(2, retryCount) * 1000;
        log_d("Retrying update in %d seconds...", retryDelay / 1000);
        delay(retryDelay);
    }

    return false;
}