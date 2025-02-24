#include "Restart.h"

void incrementRestartCount()
{
    int prevRestartCount;
    uint32_t prevRestartTime;
    myNVS::read::restartData(prevRestartCount, prevRestartTime);
    uint32_t currRestartTime = (uint32_t)(time(nullptr));

    log_i("Previous Restart Count: %d", prevRestartCount);
    log_i("Previous Restart Time: %zu", prevRestartTime);
    log_i("Current Restart Time: %zu", currRestartTime);

    if (prevRestartTime == 0 || currRestartTime - prevRestartTime > RESTART_THRESH_SECS)
    {
        myNVS::write::restartData(0, currRestartTime);
    }
    else
    {
        myNVS::write::restartData(++prevRestartCount, prevRestartTime);
    }
}

bool reachedRestartLimit()
{
    int restartCount;
    uint32_t restartTime;
    myNVS::read::restartData(restartCount, restartTime);

    if (restartCount > RESTART_COUNT_LIMIT)
    {
        return true;
    }

    return false;
}

void resetRestartCount()
{
    myNVS::write::restartData(0, (uint32_t)(time(nullptr)));
}

void onRestart()
{
    incrementRestartCount();

    if (flags[sd_f] && flags[rtc_f] && wf.check_connection() && dataClient.connected() && reachedRestartLimit())
    {
        log_d("Restarted %d times in %d minutes...", RESTART_COUNT_LIMIT, RESTART_THRESH_SECS / 60);

        String currFile = storage.curr_read_file;
        String currFilePushTopic = RESTART_PUSH_TOPIC + sensorID + currFile.substring(0, currFile.length() - 4);
        String configFile = "/config.txt";
        String configPushTopic = RESTART_PUSH_TOPIC + sensorID + "/config";

        if (sendFile(currFile.c_str(), currFilePushTopic.c_str()) && sendFile(configFile.c_str(), configPushTopic.c_str()))
        {
            String newCurrFile = currFile.substring(0, currFile.length() - 4) + "_" + String(getUnixTime()) + ".txt";
            if (SD.rename(currFile, newCurrFile))
            {
                log_i("Renamed file %s to %s.", currFile.c_str(), newCurrFile.c_str());
            }

            if (SD.remove(configFile))
            {
                log_i("Deleted existing /config.txt.");
            }
            else
            {
                log_e("Failed to delete existing /config.txt.");
            }

            log_i("Creating new /config.txt...");
            storage.update_config();
            resetRestartCount();
        }
    }
}
