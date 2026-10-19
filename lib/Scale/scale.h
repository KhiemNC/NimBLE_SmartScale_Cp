#ifndef SCALE_BLE_SERVICE_H
#define SCALE_BLE_SERVICE_H

#include "Arduino.h"
#include "math.h"
#include "config.h"
#include "ble.h"
#include <HardwareSerial.h>
#include "GM65_scanner.h"

#define SCALE_TAG "Scale Service"

struct ScaleTimestamp {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

struct ScaleValue {
    ScaleTimestamp timestamp;
    bool is_lbs;
    float weight;
    float bmi;
    uint8_t body_fat_percentage;
    uint8_t muscle_mass_percentage;
    uint8_t water_percentage;
    uint8_t bone_mass_kg;
    uint8_t visceral_fat_level;
    uint8_t metabolic_age;
};

bool verifyCitizenID (String buffer);
void qrScannerService(void *pvParameters);

/**
 * @brief Function to interpolate mass from analog value
 * 
 * @param pin - Specify the pin of analog sensor can be used to read, if the analog value cannot be read directly, this pin is set to -1
 * @param analog_val - Specify the analog value that is read from other device - Come after pin -1
 * @retval The mass of sensor point in kg
 */
float massInterpolate(int pin, int analog_val = 0, int sensor_idx = -1);
void  parseScaleData (String UUID, unsigned char * data, size_t length);
bool  verifyForceData (uint8_t * buffer, int buffer_len);
void  readForceSensors(void * pvParameters);
void  updateValuesService(void * pvParameters);

void initScaleBLEService();
void reconnectDevicesService();

void runScaleService(void *pvParameters);
#endif