#ifndef __OTA_H__
#define __OTA_H__

#include <HTTPClient.h>
#include <Update.h>
#include "config.h"

#define AWS_S3_BUCKET "aqi-firemware-updates"
#define AWS_S3_REGION "eu-north-1"
#define OTA_UPDATE_FAILURE_LIMIT 3

#define CLOUD_HOST_URL "https://" AWS_S3_BUCKET ".s3." AWS_S3_REGION ".amazonaws.com/aqi/firmware_"

extern String latestVersion;

bool downloadUpdate();

#endif