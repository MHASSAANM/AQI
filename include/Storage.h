#ifndef __STORAGE_H__
#define __STORAGE_H__
#include <Arduino.h> //for using String type
#include <SD.h>
#include "Preferences.h"
#include "config.h"

#define FILE_START_POS 80 // depends on size of header placed at start of file.
#define MIN_CHUNK_SIZE_B 47
#define MAX_CHUNK_SIZE_B 110  //2025-05-27 12:59:59,AQIMeter100,85.0,100.0,1100.0,1000,1000,1000,1000.0,500.0,10.0,60000,60000,9.999,20.0
#define FILE_HEADER "Time,ID,Temp,Humidity,Pressure,PM1.0,PM2.5,PM10.0,CO,NH3,NO2,eCO2,TVOC,O3,SO2\n"


#define CARD_SIZE_LIMIT_MB 30000
#define LOW_SPACE_LIMIT_MB 1024

extern Preferences configuration__;

/*
 * The fomrmat for file name is: YYYYMMDD.txt this should be strictly followed
 */
// WARNING: before calling mark_data, read data should be called. Also, the read data function should
//          be checked for validity (comparing with "") before calling mark_data.

class Storage
{
private:
    bool resume;
    long curr_read_pos;
    int curr_chunk_size;
    bool mount_success;
    void remove_oldest_file();
    String next_file(String);
    void create_header(File);
    bool APList_exists;
    String curr_SSID;
    String curr_Password;

public:
    bool init_storage(); // Small addition added to init to check for AP list as well.
    bool write_data(String timenow, String data);
    bool write_AP(String SSID, String Password);                    // Add a new AP to the list on the SD card
    bool rewrite_storage_APs(String SSID[10], String Password[10]); // Erase and rewrite the storage using the String array as paremeters
    String read_data(void);
    void return_APList(String SSID[10], String Password[10]); // Return String arrays for SSIDs and Passwords
    void mark_data(String timenow);
    long get_unsent_data(String timenow); // return unsent data in MBs
    void update_config();                 // update config with next file and start pos
    String curr_read_file;
};

extern Storage storage;

#endif