
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

class ESP_BlueMuse : public BLEClientCallbacks, public BLEAdvertisedDeviceCallbacks
{
public:
    ESP_BlueMuse();

    std::map<std::string, BLEAdvertisedDevice*> discoverHeadbands(void);
    void onConnect(BLEClient *pclient);
    void onDisconnect(BLEClient *pclient);
    void onResult(BLEAdvertisedDevice advertisedDevice);
    bool connectToMuseHeadband(std::string headbandName);

    static void notifyCallbackCmd(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);
    static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);

    static constexpr double MUSE_ACCELEROMETER_SCALE_FACTOR = 0.0000610352;
    static constexpr double MUSE_GYRO_SCALE_FACTOR = 0.0074768;

private:
    String _headbandName;
    String currentMessage = String();
    std::map<std::string, BLEAdvertisedDevice*> discoveredMuseDevices;

    const char *MUSE_GATT_ATTR_STREAM_TOGGLE = "273e0001-4c4d-454d-96be-f03bac821358";
    const char *MUSE_GATT_ATTR_TP9 = "273e0003-4c4d-454d-96be-f03bac821358";
    const char *MUSE_GATT_ATTR_AF7 = "273e0004-4c4d-454d-96be-f03bac821358";
    const char *MUSE_GATT_ATTR_AF8 = "273e0005-4c4d-454d-96be-f03bac821358";
    const char *MUSE_GATT_ATTR_TP10 = "273e0006-4c4d-454d-96be-f03bac821358";
    const char *MUSE_GATT_ATTR_RIGHTAUX = "273e0007-4c4d-454d-96be-f03bac821358";
    const char *MUSE_GATT_ATTR_GYRO = "273e0009-4c4d-454d-96be-f03bac821358";
    const char *MUSE_GATT_ATTR_ACCELEROMETER = "273e000a-4c4d-454d-96be-f03bac821358";
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
    bool connected = false;
    bool doScan = false;
    BLERemoteCharacteristic *pRemoteCharacteristic;
    BLERemoteCharacteristic *pRemoteCharacteristicControlChannel;
    BLEAdvertisedDevice *myDevice;
};

// ------------------------------------ End ---------------------------------
#endif
