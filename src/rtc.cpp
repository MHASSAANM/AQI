#include <rtc.h>
#include <RTClib.h>

RTC_DS1307 rtc;
ESP32Time esp_sys_time;
ESP32Time __esptime;

/**
 * @brief initRTC initializes RTC and adjusts date and time in case
 * of power loss, configures the time offset with the NTP server.
 *
 * TODO: Handle error in case RTC initialization fails
 *
 * @return returns true only if the RTC is initialized properly.
 */
bool initRTC()
{
    configTime(18000, 0, "pool.ntp.org");
    if (rtc.begin())
    {
        log_d("RTC initialization successful");
        return true;
    }
    else
    {
        log_d("RTC initialization failed");
        return false;
    }
}

/**
 * @brief getTime returns time from RTC hardware in form of string.
 *
 * TODO: Optimize the function by replacing individual strings in the return statement.
 *       This will reduce the temporary ram usage
 *
 * @return returns the time in format "YYYY-MM-DD HH:MM:SS"
 */
String getTime()
{
    if (time(nullptr) > cutoff_time)
    { // system time is adjusted
        tm now = esp_sys_time.getTimeStruct();
        String YYYY = String(now.tm_year + 1900, DEC);
        String mm = String(now.tm_mon + 1, DEC);
        if (mm.length() == 1)
        {
            mm = "0" + mm;
        }
        String dd = String(now.tm_mday, DEC);
        if (dd.length() == 1)
        {
            dd = "0" + dd;
        }
        String HH = String(now.tm_hour, DEC);
        if (HH.length() == 1)
        {
            HH = "0" + HH;
        }
        String MM = String(now.tm_min, DEC);
        if (MM.length() == 1)
        {
            MM = "0" + MM;
        }
        String SS = String(now.tm_sec, DEC);
        if (SS.length() == 1)
        {
            SS = "0" + SS;
        }
        return (YYYY + "-" + mm + "-" + dd + " " + HH + ":" + MM + ":" + SS);
    }
    else
    {
        DateTime now = rtc.now();
        String YYYY = String(now.year(), DEC);
        String mm = String(now.month(), DEC);
        if (mm.length() == 1)
        {
            mm = "0" + mm;
        }
        String dd = String(now.day(), DEC);
        if (dd.length() == 1)
        {
            dd = "0" + dd;
        }
        String HH = String(now.hour(), DEC);
        if (HH.length() == 1)
        {
            HH = "0" + HH;
        }
        String MM = String(now.minute(), DEC);
        if (MM.length() == 1)
        {
            MM = "0" + MM;
        }
        String SS = String(now.second(), DEC);
        if (SS.length() == 1)
        {
            SS = "0" + SS;
        }
        return (YYYY + "-" + mm + "-" + dd + " " + HH + ":" + MM + ":" + SS);
    }
}

/**
 * @brief getTime2 returns time from RTC hardware in form of string.
 * Difference from getTime() function is the format. It is required for writing data in file
 *
 * @return returns the time in format "YYYYMMDD"
 */
String getTime2()
{
    if (time(nullptr) > cutoff_time)
    { // system time is adjusted
        tm now = esp_sys_time.getTimeStruct();
        String YYYY = String(now.tm_year + 1900, DEC);
        String mm = String(now.tm_mon + 1, DEC);
        if (mm.length() == 1)
        {
            mm = "0" + mm;
        }
        String dd = String(now.tm_mday, DEC);
        if (dd.length() == 1)
        {
            dd = "0" + dd;
        }
        return YYYY + mm + dd;
    }
    else
    {
        DateTime now = rtc.now();
        String YYYY = String(now.year(), DEC);
        String mm = String(now.month(), DEC);
        if (mm.length() == 1)
        {
            mm = "0" + mm;
        }
        String dd = String(now.day(), DEC);
        if (dd.length() == 1)
        {
            dd = "0" + dd;
        }
        return YYYY + mm + dd;
    }
}

/**
 * @brief getNextDay returns the next date.
 * It is required for moving over to next file
 * once the previous file is sent completely.
 *
 * @return returns the day in format "YYYYMMDD"
 */
String getNextDay(int iyear, int imonth, int iday)
{
    DateTime temp1(iyear, imonth, iday, 0, 0, 0);
    TimeSpan temp2(1, 0, 0, 0);
    temp1 = temp1 + temp2;
    String YYYY = String(temp1.year(), DEC);
    String mm = String(temp1.month(), DEC);
    if (mm.length() == 1)
    {
        mm = "0" + mm;
    }
    String dd = String(temp1.day(), DEC);
    if (dd.length() == 1)
    {
        dd = "0" + dd;
    }
    return (YYYY + mm + dd);
}

/**
 * @brief Get the Unix Time object
 *
 * @return returns the unix time as uint32_t object
 */
uint32_t getUnixTime()
{
    if (time(nullptr) > cutoff_time)
        return time(nullptr);
    else
    {
        DateTime now = rtc.now();
        return now.unixtime();
    }
}

/**
 * @brief unixTime returns ESP time in form of string.
 *
 * @return returns the time in unix format.
 */
String unixTime()
{
    // DateTime now = rtc.now();
    // return String(now.unixtime());
    return String(time(nullptr));
}

/**
 * @brief _set_esp_time sync the ESP time with RTC time.
 *
 * @param void
 */
void _set_esp_time()
{
    DateTime now = rtc.now();
    esp_sys_time.setTime(now.unixtime() - 18000, 0);
}

/**
 * @brief _set_esp_time sync the ESP time with the provided day.
 * The difference with the other format with void parameters is
 * that takes time from RTC but this one needs manual setting.
 *
 * @param yr is the current year
 * @param mn is the current month
 * @param dy is the date that is to be set
 */
void _set_esp_time(int yr, int mn, int dy)
{
    esp_sys_time.setTime(0, 0, 0, dy, mn, yr, 0);
}

/**
 * @brief setRtcTime sync the RTC time with ESP time.
 *
 * @param void
 */
void setRtcTime()
{
    log_i("Set RTC time");
    tm now = esp_sys_time.getTimeStruct();
    rtc.adjust(DateTime(now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour, now.tm_min, now.tm_sec));
}