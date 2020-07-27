
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

ESP_BlueMuse *ESP_BlueMuse::anchor = NULL;

const char *ESP_BlueMuse::MUSE_GATT_ATTR_GYRO = "273e0009-4c4d-454d-96be-f03bac821358";
const char *ESP_BlueMuse::MUSE_GATT_ATTR_ACCELEROMETER = "273e000a-4c4d-454d-96be-f03bac821358";
const char *ESP_BlueMuse::MUSE_GATT_ATTR_TP9 = "273e0003-4c4d-454d-96be-f03bac821358";
const char *ESP_BlueMuse::MUSE_GATT_ATTR_AF7 = "273e0004-4c4d-454d-96be-f03bac821358";
const char *ESP_BlueMuse::MUSE_GATT_ATTR_AF8 = "273e0005-4c4d-454d-96be-f03bac821358";
const char *ESP_BlueMuse::MUSE_GATT_ATTR_TP10 = "273e0006-4c4d-454d-96be-f03bac821358";
const char *ESP_BlueMuse::MUSE_GATT_ATTR_RIGHTAUX = "273e0007-4c4d-454d-96be-f03bac821358";

ESP_BlueMuse::ESP_BlueMuse()
{
    if (ESP_BlueMuse::anchor != NULL)
    {
        Serial.println("ESP_BlueMuse must be used as a singleton, do not create more than one instance or it will fail!");
    }
    BLEDevice::setCustomGattcHandler(gattcEventHandler);
    BLEDevice::init("MuseClient");
    ESP_BlueMuse::anchor = this;
}

bool ESP_BlueMuse::setPresetMode(const byte preset)
{
    if (preset == MUSE_MODE_ALL_CHANNELS_WITH_ACCELEROMETER || preset == MUSE_MODE_NO_AUX_WITH_ACCELEROMETER || preset == MUSE_MODE_NO_AUX_NO_ACCELEROMETER)
    {
        Serial.printf("Setting preset mode to %i\n", preset);
        String cmd;
        this->_musePresetMode = preset;
        if (this->_isConnected)
        {
            switch (preset)
            {
            case ESP_BlueMuse::MUSE_MODE_ALL_CHANNELS_WITH_ACCELEROMETER:
                cmd = "\x04\x70\x32\x30\x0a";
                break;

            case ESP_BlueMuse::MUSE_MODE_NO_AUX_NO_ACCELEROMETER:
                cmd = "\x04\x70\x32\x32\x0a";
                break;

            case ESP_BlueMuse::MUSE_MODE_NO_AUX_WITH_ACCELEROMETER:
            default:
                cmd = "\x04\x70\x32\x31\x0a";
                break;
            }
            this->sendCommandToHeadset(MUSE_GATT_ATTR_STREAM_TOGGLE, cmd);
        }
        return true;
    }
    return false;
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
    pBLEScan->setInterval(500);
    pBLEScan->setWindow(1500);
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

void ESP_BlueMuse::gattcEventHandler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    //ESP_LOGW(LOG_TAG, "custom gattc event handler, event: %d", (uint8_t)event);
    if (event == ESP_GATTC_DISCONNECT_EVT)
    {
        Serial.printf("Disconnect reason: %i\n", (int)param->disconnect.reason);
    }
}

/**
 * called when connection to headband has been established
 */
void ESP_BlueMuse::onConnect(BLEClient *pclient)
{
    this->_connectionTimestamp = millis();
    Serial.printf("Connected to device at %ld\n", this->_connectionTimestamp);

    if (this->_callbackConnect != NULL)
    {
        Serial.println("callback defined");
        //        this->_callbackConnect(pclient); TODO fix this part
    }
    else
    {
        Serial.println("NO connection callback defined");
    }
}

/**
 * called when connection to headband is lost 
 */
void ESP_BlueMuse::onDisconnect(BLEClient *pclient)
{
    _isConnected = false;
    Serial.printf("Device disconnected after %ld ms\n", millis() - this->_connectionTimestamp);
    /*
    if(this->_callbackDisconnect != NULL){
        this->_callbackDisconnect(pclient);
    }
*/
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

void ESP_BlueMuse::eegDataCallbackHandler(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    int sensor = pBLERemoteCharacteristic->getUUID().getNative()->uuid.uuid128[12];

    if (length == 20)
    {
        unsigned int timestamp = pData[0] << 8 | pData[1];
        const byte dataPackageCounter = 12;
        signed int values[dataPackageCounter];
        byte indexCounter = 0;

        for (int i = 2; i < length; i++)
        {
            if (i % 3 == 0)
            {
                values[indexCounter] = pData[i] << 4 | pData[i + 1] >> 4;
                indexCounter++;
            }
            else
            {
                values[indexCounter] = (pData[i] & 0xf) << 8 | pData[i + 1];
                indexCounter++;
                i++;
            }
        }
        if (indexCounter != 12)
        {
            Serial.printf("Problem with decoding samples. Expected to find 12 samples with 12 bit each, but found %i\n", indexCounter);
        }

        //check if we got data for the next package already, so send it out before processing new data
        if (ESP_BlueMuse::anchor->eegDataPackage.timestamp != timestamp)
        {
            if (ESP_BlueMuse::anchor->_callbackEEGData)
            {
                ESP_BlueMuse::anchor->_callbackEEGData(ESP_BlueMuse::anchor->eegDataPackage);
            }
            else
            {
                Serial.println("Got EEG data package, but no handler function has been defined");
            }
        }
        //now add data to existing package / update it
        ESP_BlueMuse::anchor->eegDataPackage.timestamp = timestamp;
        switch (sensor)
        {
        case MUSE_SENSOR_AF7:
            for (int i = 0; i < indexCounter; ++i)
                ESP_BlueMuse::anchor->eegDataPackage.AF7[i] = values[i];
            break;
        case MUSE_SENSOR_AF8:
            for (int i = 0; i < indexCounter; ++i)
                ESP_BlueMuse::anchor->eegDataPackage.AF8[i] = values[i];
            break;
        case MUSE_SENSOR_TP9:
            for (int i = 0; i < indexCounter; ++i)
                ESP_BlueMuse::anchor->eegDataPackage.TP9[i] = values[i];
            break;
        case MUSE_SENSOR_TP10:
            for (int i = 0; i < indexCounter; ++i)
                ESP_BlueMuse::anchor->eegDataPackage.TP10[i] = values[i];
            break;
        case MUSE_SENSOR_RIGHTAUX:
            for (int i = 0; i < indexCounter; ++i)
                ESP_BlueMuse::anchor->eegDataPackage.RIGHTAUX[i] = values[i];
            break;
        default:
            //unknown sensor
            Serial.printf("Unknown sensor id received: %i. data will be ignored\n", sensor);
            break;
        }
    }
    else
    {
        Serial.printf("Unexpexted package size detected: %i\n", length);
    }
}

/**,
 * generic callback handler, not extremly optimized, thus should only be used for non EEG data
 */
void ESP_BlueMuse::genericDataCallbackHandler(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
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

    if (strcmp(pBLERemoteCharacteristic->getUUID().toString().c_str(), ESP_BlueMuse::MUSE_GATT_ATTR_GYRO) == 0)
    {
        if (ESP_BlueMuse::anchor->_callbackGyroData)
        {
            ESP_BlueMuse::anchor->_callbackGyroData(val1 * MUSE_GYRO_SCALE_FACTOR, val2 * MUSE_GYRO_SCALE_FACTOR, val3 * MUSE_GYRO_SCALE_FACTOR);
        }
        else
        {
            Serial.println("Got Gyro data package, but no handler function has been defined");
        }
    }
    else if (strcmp(pBLERemoteCharacteristic->getUUID().toString().c_str(), ESP_BlueMuse::MUSE_GATT_ATTR_ACCELEROMETER) == 0)
    {
        if (ESP_BlueMuse::anchor->_callbackAccell)
        {
            ESP_BlueMuse::anchor->_callbackAccell(val1 * MUSE_ACCELEROMETER_SCALE_FACTOR, val2 * MUSE_ACCELEROMETER_SCALE_FACTOR, val3 * MUSE_ACCELEROMETER_SCALE_FACTOR);
        }
        else
        {
            Serial.println("Got Accelerometer data package but nor handler function has been defined");
        }
    }
    else
    {
        Serial.printf("%i x: %2.2f, y: %2.2f, z: %2.2f\n", timestamp, val1 * MUSE_ACCELEROMETER_SCALE_FACTOR, val2 * MUSE_ACCELEROMETER_SCALE_FACTOR, val3 * MUSE_ACCELEROMETER_SCALE_FACTOR);
    }
}

void ESP_BlueMuse::controlDataCallbackHandler(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
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

/**
 * register a callback function that is triggered when a connection to the headband has been established
 */
void ESP_BlueMuse::registerOnConnectHandler(connectionStateChangeCallbackFunction callbackFunction)
{
    this->_callbackConnect = callbackFunction;
}

/**
 * register a callback function that is triggered when an existing connection to a headband has been closed
 */
void ESP_BlueMuse::registerOnDisconnectHandler(connectionStateChangeCallbackFunction callbackFunction)
{
    this->_callbackDisconnect = callbackFunction;
}

/**
 * register a callback function that is triggered when a new EEG data package has been recieved.
 * Each packet is encoded with a 16bit timestamp followed by 12 samples with a 12 bit resolution. 12 bits on a 2 mVpp range.
 * The callback function needs to have the following signature: void eegDataCallbackFunction(long timestamp, float sample1, float sample2, float sample3, float sample4, float sample5, float sample6, float sample7, float sample8, float sample9, float sample10, float sample11, float sample12)
 */
void ESP_BlueMuse::registerEEGHandler(eegDataCallbackFunction callbackFunction)
{
    this->_callbackEEGData = callbackFunction;
}

/**
 * register a callback function that is triggered when a gyro data package has been received
 */
void ESP_BlueMuse::registerGyroHandler(gyroDataCallbackFunction callbackFunction)
{
    this->_callbackGyroData = callbackFunction;
}

/**
 * register a callback function that is triggered when a accelerometer data package has been received
 */
void ESP_BlueMuse::registerAccelerometerHandler(accelerometerDataCallbackFunction callbackFunction)
{
    this->_callbackAccell = callbackFunction;
}

/**
 * request and return the battery level from the headset.
 * Requires an active connection to a headset, otherwise -1 is returned.
 */
float ESP_BlueMuse::getBatteryLevel(void)
{
    if (this->_isConnected)
    {
        //TODO return
    }

    return -1;
}

/**
 * request and return the firmware version of the headset.
 * Requires an active connection to a headset, otherwise -1 is returned.
 */
float ESP_BlueMuse::getFirmwareVersion(void)
{
    if (this->_isConnected)
    {
        //TODO return
    }
    return -1;
}

/**
 * request start of the EEG data transmission from the headset.
 * Make sure to register a callback handler using registerEEGHandler() before starting the data flow
 */
void ESP_BlueMuse::startEEGData(void)
{
    //register to data characteristic if needed
    this->subscribe(MUSE_GATT_ATTR_TP9);
    this->subscribe(MUSE_GATT_ATTR_AF7);
    this->subscribe(MUSE_GATT_ATTR_AF8);
    this->subscribe(MUSE_GATT_ATTR_TP10);
    this->subscribe(MUSE_GATT_ATTR_RIGHTAUX);
    this->startStreaming();
}

/**
 *
 */
void ESP_BlueMuse::stopEEGData(void)
{
    //unregister to data characteristic if needed
    this->unsubscribe(MUSE_GATT_ATTR_TP9);
    this->unsubscribe(MUSE_GATT_ATTR_AF7);
    this->unsubscribe(MUSE_GATT_ATTR_AF8);
    this->unsubscribe(MUSE_GATT_ATTR_TP10);
    this->unsubscribe(MUSE_GATT_ATTR_RIGHTAUX);
}

/**
 * 
 */
void ESP_BlueMuse::startGyroData(void)
{
    //register to data characteristic if needed
    this->subscribe(MUSE_GATT_ATTR_GYRO);
    this->startStreaming();
}

/**
 * 
 */
void ESP_BlueMuse::stopGyroData(void)
{
    this->unsubscribe(MUSE_GATT_ATTR_GYRO);
}

/**
 *
 */
void ESP_BlueMuse::startAccelerometerData(void)
{
    //register to data characteristic if needed
    this->subscribe(MUSE_GATT_ATTR_ACCELEROMETER);
    this->startStreaming();
}

/**
 *
 */
void ESP_BlueMuse::stopAccelerometerData(void)
{
    this->unsubscribe(MUSE_GATT_ATTR_ACCELEROMETER);
}

/**
 * send a command to the headset for the given characteristic UUID
 * Command structure:
 * <length cmd + nl caharacter><cmd><\x0a>
 * All cahracters hex encoded, e.g.:
 * \x04\x70\x32\x31\x0a
 * 
 */
bool ESP_BlueMuse::sendCommandToHeadset(const char *characteristicUUID, String cmd)
{
    if (this->_isConnected)
    {
        pRemoteCharacteristicControlChannel = this->pRemoteService->getCharacteristic(characteristicUUID);
        if (pRemoteCharacteristicControlChannel != nullptr)
        {
            Serial.printf("Sending command: %s to UUID %s\n", (unsigned char *)cmd.c_str(), characteristicUUID);
            pRemoteCharacteristicControlChannel->writeValue((unsigned char *)cmd.c_str(), cmd.length(), false);
            return true;
        }
    }
    return false;
}

/**
 * subscribe to a BLE characteristic of the headband
 */
bool ESP_BlueMuse::subscribe(const char *characteristicUUID)
{
    if (this->pRemoteService != nullptr)
    {
        pRemoteCharacteristic = pRemoteService->getCharacteristic(characteristicUUID);
        if (pRemoteCharacteristic != nullptr && pRemoteCharacteristic->canNotify())
        {
            if (strcmp(characteristicUUID, MUSE_GATT_ATTR_TP9) == 0 || strcmp(characteristicUUID, MUSE_GATT_ATTR_TP10) == 0 || strcmp(characteristicUUID, MUSE_GATT_ATTR_AF7) == 0 || strcmp(characteristicUUID, MUSE_GATT_ATTR_AF8) == 0 || strcmp(characteristicUUID, MUSE_GATT_ATTR_RIGHTAUX) == 0)
            {
                pRemoteCharacteristic->registerForNotify(eegDataCallbackHandler);
            }
            else if (strcmp(characteristicUUID, MUSE_GATT_ATTR_STREAM_TOGGLE) == 0)
            {
                pRemoteCharacteristic->registerForNotify(controlDataCallbackHandler);
            }
            else if (strcmp(characteristicUUID, MUSE_GATT_ATTR_STREAM_TOGGLE) == 0)
            {
                pRemoteCharacteristic->registerForNotify(genericDataCallbackHandler);
            }
            return true;
        }
    }
    return false;
}

/**
 * unsubscribe from a BLE characteristic of the headband
 */
bool ESP_BlueMuse::unsubscribe(const char *characteristicUUID)
{
    pRemoteCharacteristic = pRemoteService->getCharacteristic(characteristicUUID);
    if (pRemoteCharacteristic != nullptr && pRemoteCharacteristic->canNotify())
    {
        pRemoteCharacteristic->registerForNotify(NULL); //unregister by providing NULL as callback
        return true;
    }
    return false;
}

void ESP_BlueMuse::sendKeepAlive(void)
{
    String cmd = "\x02\x6b\x0a";
    this->sendCommandToHeadset(MUSE_GATT_ATTR_STREAM_TOGGLE, cmd);
}

void ESP_BlueMuse::startStreaming(void)
{
    //set preset mode (Again) just to be sure, since it seems to be reset after sending the stopStreaming command)
    this->setPresetMode(this->_musePresetMode);

    String cmd = "\x02\x64\x0a";
    this->sendCommandToHeadset(MUSE_GATT_ATTR_STREAM_TOGGLE, cmd);
}

void ESP_BlueMuse::stopStreaming(void)
{
    String cmd = "\x02\x68\x0a";
    this->sendCommandToHeadset(MUSE_GATT_ATTR_STREAM_TOGGLE, cmd);
}

void ESP_BlueMuse::requestDeviceInfo(void)
{
    String cmd = "\x03\x76\x31\x0a";
    this->sendCommandToHeadset(MUSE_GATT_ATTR_STREAM_TOGGLE, cmd);
}

void ESP_BlueMuse::requestControlStatus(void)
{
    String cmd = "\x02\x73\x0a";
    this->sendCommandToHeadset(MUSE_GATT_ATTR_STREAM_TOGGLE, cmd);
}

bool ESP_BlueMuse::connectToMuseHeadbandByUUID(std::string uuidString, boolean autoReconnect)
{
    this->_autoReconnect = autoReconnect;
    BLEClient *client = BLEDevice::createClient();
    //Serial.println(" - Created client");

    client->setClientCallbacks(this);
    // Connect to the remove BLE Server.
    BLEAddress address = BLEAddress(uuidString);

    if (client->connect(address, BLE_ADDR_TYPE_PUBLIC))
    {
        Serial.println("Connected to headband");
        this->pRemoteService = client->getService(serviceUUID);
        if (this->pRemoteService == nullptr)
        {
            Serial.print("Failed to find our service UUID: ");
            Serial.println(serviceUUID.toString().c_str());
            client->disconnect();
            return false;
        }
        Serial.printf("Found reguired BLE service:  %s\n", serviceUUID.toString().c_str());
        _isConnected = true;
        this->subscribe(MUSE_GATT_ATTR_STREAM_TOGGLE);
        return true;
    }
    return false;
}

/**
 * 
 */
bool ESP_BlueMuse::connectToMuseHeadband(std::string headbandName, boolean autoReconnect)
{
    this->_autoReconnect = autoReconnect;

    if (this->discoveredMuseDevices.empty())
    {
        Serial.println("No devices known, did you start discovery before to search for muse headbands?");
        //TODO: try to scan for device
        return false;
    }
    else
    {
        auto search = this->discoveredMuseDevices.find(headbandName);
        if (search != this->discoveredMuseDevices.end())
        {
            this->advertisedDeviceToConnectTo = search->second;
        }
    }

    if (this->advertisedDeviceToConnectTo == NULL)
    {
        Serial.println("Unknown device, connection failed");
        return false;
    }
    else
    {
        Serial.printf("Forming a connection to %s with type %i \n", this->advertisedDeviceToConnectTo->getAddress().toString().c_str(), this->advertisedDeviceToConnectTo->getAddressType());

        BLEClient *client = BLEDevice::createClient();
        //Serial.println(" - Created client");
        client->setClientCallbacks(this);
        // Connect to the remove BLE Server.
        if (client->connect(this->advertisedDeviceToConnectTo))
        {
            // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
            Serial.println("Connected to headband");

            // Obtain a reference to the service we are after in the remote BLE server.
            /*
        std::map<std::string, BLERemoteService *> *serviceMap = this->pClient->getServices();
        //Serial.printf("found %i services\n", serviceMap->size());

        for (const auto &pair : *serviceMap)
        {
            Serial.printf("\nkey: %s, value: %s\n", pair.first.c_str(), pair.second->getUUID().toString().c_str());
            std::map<std::string, BLERemoteCharacteristic *> *characteristics = pair.second->getCharacteristics();
            for (const auto &characteristicsPair : *characteristics)
            {
                Serial.printf(" - %s\n", characteristicsPair.second->toString().c_str());
            }
        }
*/
            this->pRemoteService = client->getService(serviceUUID);
            if (this->pRemoteService == nullptr)
            {
                Serial.print("Failed to find our service UUID: ");
                Serial.println(serviceUUID.toString().c_str());
                client->disconnect();
                return false;
            }
            Serial.printf("Found reguired BLE service:  %s\n", serviceUUID.toString().c_str());
        }

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
        pRemoteCharacteristic->registerForNotify(genericDataCallbackHandler);
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
    */
        _isConnected = true;
        return true;
    }
    return false;
}
