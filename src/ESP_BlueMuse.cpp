
//      ******************************************************************
//      *                                                                *
//      *                           ESP_BlueMuse                         *
//      *                                                                *
//      *            Paul Kerspe                     14.7.2020           *
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

//
// for more details and a manual on how to use this libaray, check the README.md on github and the provided examples
// https://github.com/pkerspe/ESP-BlueMuse/blob/master/README.md
//

#include "ESP_BlueMuse.h"

ESP_BlueMuse::ESP_BlueMuse()
{
    BLEDevice::init("");
}

/**
 * start scanning for Muse headbands via BLE.
 * This is a blocking function call, that returns once scanning has been completed
 */
std::map<std::string, BLEAdvertisedDevice *> ESP_BlueMuse::discoverHeadbands(void)
{
    Serial.println("Starting discovery of BLE devices");
    BLEScan *pBLEScan = BLEDevice::getScan();
    //pBLEScan->setAdvertisedDeviceCallbacks(this); //use this if we want to discover the first available device only and stop the scan
    pBLEScan->setInterval(600);
    pBLEScan->setWindow(500);
    pBLEScan->setActiveScan(false);
    BLEScanResults results = pBLEScan->start(5, false);

    this->discoveredMuseDevices.clear();
    for (int i = 0; i < results.getCount(); i++)
    {
        BLEAdvertisedDevice *advertisedDevice = new BLEAdvertisedDevice(results.getDevice(i));
        if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(serviceUUID))
        {
            Serial.printf("Muse headband found: %s\n", advertisedDevice->getName().c_str());
            this->discoveredMuseDevices.insert(std::pair<std::string, BLEAdvertisedDevice *>(advertisedDevice->getName(), advertisedDevice));
        }
    }

    Serial.printf("Discovery completed, found a total of %i BLE devices, out of which %i are muse devices\n", results.getCount(), this->discoveredMuseDevices.size());
    return this->discoveredMuseDevices;
}

/**
 * called when connection to headband has been established
 */
void ESP_BlueMuse::onConnect(BLEClient *pclient)
{
}

/**
 * called when connection to headband is lost 
 */
void ESP_BlueMuse::onDisconnect(BLEClient *pclient)
{
    connected = false;
    Serial.println("onDisconnect");
}

/**
* Called for each advertising BLE server during headband scan
*/
void ESP_BlueMuse::onResult(BLEAdvertisedDevice advertisedDevice)
{
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID))
    {
        Serial.printf("Muse Device found: %s\n", advertisedDevice.toString().c_str());
        //BLEDevice::getScan()->stop();
        //BLEAdvertisedDevice myDevice = BLEAdvertisedDevice(advertisedDevice);
    }
}

void ESP_BlueMuse::notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    unsigned int timestamp = pData[0] << 8 | pData[1];
    signed int val1 = pData[2] << 8 | pData[3];
    signed int val2 = pData[4] << 8 | pData[5];
    signed int val3 = pData[6] << 8 | pData[7];
    /*
  signed int val4 = pData[8] << 8 | pData[9];
  signed int val5 = pData[10] << 8 | pData[11];
  signed int val6 = pData[12] << 8 | pData[13];
  signed int val7 = pData[14] << 8 | pData[15];
  signed int val8 = pData[16] << 8 | pData[17];
  signed int val9 = pData[18] << 8 | pData[19];
  */
    Serial.printf("%i x: %2.2f, y: %2.2f, z: %2.2f\n", timestamp, val1 * MUSE_ACCELEROMETER_SCALE_FACTOR, val2 * MUSE_ACCELEROMETER_SCALE_FACTOR, val3 * MUSE_ACCELEROMETER_SCALE_FACTOR);
}

void ESP_BlueMuse::notifyCallbackCmd(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    char *data = (char *)pData;
    int dataLength = data[0];
    String dataString = String(data);
    Serial.println(dataString.substring(1, dataLength + 1).c_str());
    /*
    currentMessage.concat(dataString.substring(1, dataLength + 1));
    if (currentMessage.endsWith("}") || currentMessage.length() > 400)
    {
        Serial.println(currentMessage.c_str());
        currentMessage.clear();
    }
    */
}

bool ESP_BlueMuse::connectToMuseHeadband(std::string headbandName)
{
    BLEAdvertisedDevice *advertisedDevice;

    if (!this->discoveredMuseDevices.empty())
    {
        auto search = this->discoveredMuseDevices.find(headbandName);
        if (search != this->discoveredMuseDevices.end())
        {
            advertisedDevice = search->second;
        }
    }
    else
    {
        Serial.println("No devices known, did you start discovery before to search for muse headbands?");
        //try to scan for device
    }

    if (advertisedDevice == NULL)
    {
        Serial.println("Unknown device, connection failed");
    }
    else
    {
        Serial.printf("Forming a connection to %s", advertisedDevice->getAddress().toString().c_str());

        BLEClient *pClient = BLEDevice::createClient();
        Serial.println(" - Created client");

        pClient->setClientCallbacks(this);
        // Connect to the remove BLE Server.
        pClient->connect(advertisedDevice); // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
        Serial.println(" - Connected to server");
        

    // Obtain a reference to the service we are after in the remote BLE server.
    std::map<std::string, BLERemoteService *> *serviceMap = pClient->getServices();
    Serial.printf("found %i services\n", serviceMap->size());

    for (const auto &pair : *serviceMap)
    {
        Serial.printf("\nkey: %s, value: %s\n", pair.first.c_str(), pair.second->getUUID().toString().c_str());
        std::map<std::string, BLERemoteCharacteristic *> *characteristics = pair.second->getCharacteristics();
        for (const auto &characteristicsPair : *characteristics)
        {
            Serial.printf(" - %s\n", characteristicsPair.second->toString().c_str());
        }
    }

    BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr)
    {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.printf(" - Found our service:  %s\n", serviceUUID.toString().c_str());
/*
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr)
    {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our gyro characteristic");

    if (pRemoteCharacteristic->canNotify())
    {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
        Serial.println(" - registered for notifications");
    }

    //register for attribute characteristic
    pRemoteCharacteristicControlChannel = pRemoteService->getCharacteristic(charUUIDControl);
    if (pRemoteCharacteristicControlChannel == nullptr)
    {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charUUIDControl.toString().c_str());
        pClient->disconnect();
        return false;
    }
    if (pRemoteCharacteristicControlChannel->canNotify())
    {
        pRemoteCharacteristicControlChannel->registerForNotify(notifyCallbackCmd);
        Serial.printf(" - registered for notifications on attribute characteristic %s\n", pRemoteCharacteristicControlChannel->getUUID().toString().c_str());
        Serial.println(pRemoteCharacteristicControlChannel->toString().c_str());
    }
    if (pRemoteCharacteristicControlChannel->canWriteNoResponse())
    {
        uint8_t cmd[] = {0x02, 0x73, 0x0a};
        Serial.println(" - sending status cmd for control status");
        pRemoteCharacteristicControlChannel->writeValue(cmd, 3, false);

        uint8_t cmdInfo[] = {0x03, 0x76, 0x31, 0x0a};
        Serial.println(" - sending status cmd for device info");
        pRemoteCharacteristicControlChannel->writeValue(cmdInfo, 4, false);

        uint8_t cmdStream[] = {0x02, 0x64, 0x0a};
        Serial.println(" - sending status cmd to start streaming");
        pRemoteCharacteristicControlChannel->writeValue(cmdStream, 3, false);
    }

    connected = true;
    */
    }
    return true;
}
