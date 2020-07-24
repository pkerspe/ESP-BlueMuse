
//      ******************************************************************
//      *                                                                *
//      *                    Header file for ESP-BlueMuse                *
//      *                                                                *
//      *            Paul Kerspe                     14.7.2020           *
//      *     based on reverse engineering work of Alexandre Barachant   *
//      *                      and his BlueMuse Script                   *
//      *         https://github.com/alexandrebarachant/muse-lsl         *
//      *                                                                *
//      ******************************************************************

// MIT License
//
// Copyright (c) 2020 Paul Kerspe
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ESP_BlueMuse_h
#define ESP_BlueMuse_h

#define MUSE_SENSOR_TP9 3
#define MUSE_SENSOR_AF7 4
#define MUSE_SENSOR_AF8 5
#define MUSE_SENSOR_TP10 6
#define MUSE_SENSOR_RIGHTAUX 7

#ifdef ESP32
//
//#elif defined(ESP8266)
//
#else
#error Platform not supported, only ESP32 modules are currently supported
#endif

#include <Arduino.h>
#include "BLEDevice.h"
#include <vector>

struct eegData {
    long timestamp;
    int TP9[12];
    int TP10[12];
    int AF7[12];
    int AF8[12];
    int RIGHTAUX[12];
};

typedef void (*callbackFunction)(void);
typedef void (*connectionStateChangeCallbackFunction)(BLEClient *pclient);
typedef void (*eegDataCallbackFunction)(eegData);
typedef void (*gyroDataCallbackFunction)(float, float, float);
typedef void (*accelerometerDataCallbackFunction)(float, float, float);

class ESP_BlueMuse : public BLEClientCallbacks, public BLEAdvertisedDeviceCallbacks
{
public:
    ESP_BlueMuse();

    std::map<std::string, BLEAdvertisedDevice *> discoverHeadbands(void);

    void onConnect(BLEClient *pclient);
    void onDisconnect(BLEClient *pclient);

    void onResult(BLEAdvertisedDevice advertisedDevice);
    bool connectToMuseHeadband(std::string headbandName, boolean autoReconnect = true);
    bool connectToMuseHeadbandByUUID(std::string uuidString, boolean autoReconnect = true);

    void registerOnConnectHandler(connectionStateChangeCallbackFunction callbackFunction);
    void registerOnDisconnectHandler(connectionStateChangeCallbackFunction callbackFunction);
    void registerEEGHandler(eegDataCallbackFunction callbackFunction);
    void registerGyroHandler(gyroDataCallbackFunction callbackFunction);
    void registerAccelerometerHandler(accelerometerDataCallbackFunction callbackFunction);

    float getBatteryLevel(void);
    float getFirmwareVersion(void);

    void startEEGData(void);
    void stopEEGData(void);

    void startGyroData(void);
    void stopGyroData(void);

    void startAccelerometerData(void);
    void stopAccelerometerData(void);

    void sendKeepAlive(void);
    void startStreaming(void);
    void stopStreaming(void);

    static constexpr double MUSE_ACCELEROMETER_SCALE_FACTOR = 0.0000610352;
    static constexpr double MUSE_GYRO_SCALE_FACTOR = 0.0074768;

    static const char *MUSE_GATT_ATTR_GYRO;
    static const char *MUSE_GATT_ATTR_ACCELEROMETER;
    // EEG Electrode data
    static const char *MUSE_GATT_ATTR_TP9;
    static const char *MUSE_GATT_ATTR_AF7;
    static const char *MUSE_GATT_ATTR_AF8;
    static const char *MUSE_GATT_ATTR_TP10;
    static const char *MUSE_GATT_ATTR_RIGHTAUX;

private:
    static void notifyCallbackCmd(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);
    static void genericDataCallbackHandler(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);
    static void eegDataCallbackHandler(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);

    bool sendCommandToHeadset(const char *characteristicUUID, String cmd);
    bool subscribe(const char *characteristicUUID);
    bool unsubscribe(const char *characteristicUUID);

    void requestDeviceInfo(void);
    void requestControlStatus(void);

    BLERemoteService *pRemoteService;
    String _headbandName;
    String currentMessage = String();
    std::map<std::string, BLEAdvertisedDevice *> discoveredMuseDevices;
    bool _autoReconnect = false;
    BLEAdvertisedDevice *advertisedDeviceToConnectTo;

    eegData eegDataPackage;

    long _connectionTimestamp;

    static ESP_BlueMuse *anchor;

    const char *MUSE_GATT_ATTR_STREAM_TOGGLE = "273e0001-4c4d-454d-96be-f03bac821358";

    const char *MUSE_GATT_ATTR_TELEMETRY = "273e000b-4c4d-454d-96be-f03bac821358";

    const char *MUSE_GATT_ATTR_PPG1 = "273e000f-4c4d-454d-96be-f03bac821358";
    const char *MUSE_GATT_ATTR_PPG2 = "273e0010-4c4d-454d-96be-f03bac821358";
    const char *MUSE_GATT_ATTR_PPG3 = "273e0011-4c4d-454d-96be-f03bac821358";

    const char *museServiceUUID = "0000fe8d-0000-1000-8000-00805f9b34fb";

    // The remote service we wish to connect to.
    BLEUUID serviceUUID = BLEUUID("0000fe8d-0000-1000-8000-00805f9b34fb");
    // The characteristic of the remote service we are interested in.
    const BLEUUID charUUID = BLEUUID(MUSE_GATT_ATTR_ACCELEROMETER);
    const BLEUUID charUUIDControl = BLEUUID(MUSE_GATT_ATTR_STREAM_TOGGLE);

    bool doConnect = false;
    bool _isConnected = false;
    bool doScan = false;
    BLERemoteCharacteristic *pRemoteCharacteristic;
    BLERemoteCharacteristic *pRemoteCharacteristicControlChannel;
    BLEAdvertisedDevice *myDevice;

    //callback function handles
    connectionStateChangeCallbackFunction _callbackConnect;
    connectionStateChangeCallbackFunction _callbackDisconnect;
    eegDataCallbackFunction _callbackEEGData;
    gyroDataCallbackFunction _callbackGyroData;
    accelerometerDataCallbackFunction _callbackAccell;
};

// ------------------------------------ End ---------------------------------
#endif
