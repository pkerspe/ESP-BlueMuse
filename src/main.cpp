#include <Arduino.h>
#include "ESP_BlueMuse.h"
#include "soc/efuse_reg.h"

int getChipRevision()
{
  return (REG_READ(EFUSE_BLK0_RDATA3_REG) >> (EFUSE_RD_CHIP_VER_REV1_S) && EFUSE_RD_CHIP_VER_REV1_V);
}

void printChipInfo()
{
  Serial.print("REG_READ(EFUSE_BLK0_RDATA3_REG) ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG), BIN);

  Serial.printf("Chip Revision (official version): %i\n", getChipRevision());
 
  Serial.print("Chip Revision from shift Opration ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG) >> 15, BIN);
}

// do not create more than one blueMuse instance, it will not work due to the static use of callback functions and variables
ESP_BlueMuse *blueMuse;

void gyroDataHandler(float x, float y, float z)
{
  Serial.printf("Gyro: %f/%f/%f\n", x, y, z);
}

void accelerometerDataHandler(float x, float y, float z)
{
  Serial.printf("Accelerometer: %f/%f/%f\n", x, y, z);
}

void eegDataHandler(eegData dataPackage)
{
 Serial.printf("%ld EEGData: AF7: %i / AF8: %i, %i, %i ... / TP9: %i / TP10: %i / Rightaux: %i\n", dataPackage.timestamp, dataPackage.AF7[0], dataPackage.AF7[1], dataPackage.AF7[2], dataPackage.AF8[0], dataPackage.TP9[0], dataPackage.TP10[0], dataPackage.RIGHTAUX[0]);
}

void setup()
{
  Serial.begin(230400);

  printChipInfo();
  Serial.println("Starting Arduino BLE Client application...");

  blueMuse = new ESP_BlueMuse();

  blueMuse->registerGyroHandler(gyroDataHandler);
  blueMuse->registerAccelerometerHandler(accelerometerDataHandler);
  blueMuse->registerEEGHandler(eegDataHandler);

  if (blueMuse->connectToMuseHeadbandByUUID("00:55:da:b0:e9:95"))
  {
    Serial.println("Connected!");
    //blueMuse->startGyroData();
    blueMuse->startEEGData();
  }
  else
  {
    std::map<std::string, BLEAdvertisedDevice *> museHeadbands = blueMuse->discoverHeadbands();

    Serial.printf("Found %i muse headbands\n", museHeadbands.size());
    if (museHeadbands.size() > 0)
    {
      Serial.printf("connecting to muse headband %s\n", museHeadbands.begin()->second->getName().c_str());
      if (blueMuse->connectToMuseHeadband(museHeadbands.begin()->second->getName()))
      {
        Serial.println("Connected!");
        //blueMuse->startGyroData();
        //blueMuse->startEEGData();
      }
    }
    else
    {
      Serial.println("No muse headbands found");
    }
  }
}

void loop()
{
  blueMuse->sendKeepAlive();
  Serial.println("*********** K ************");
  delay(1000);
  blueMuse->startStreaming();
  Serial.println("*********** STREAM ************");
  delay(1000);
  /*
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true)
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
    }
    else
    {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  if (connected)
  {
   if (pRemoteCharacteristicControlChannel != nullptr && pRemoteCharacteristicControlChannel->canWriteNoResponse())
    {
      delay(5000); // Delay a second between loops.
      uint8_t cmdInfo[] = {0x02, 0x6b, 0x0a};
      Serial.println(" - sending status cmd for keep alive");
      pRemoteCharacteristicControlChannel->writeValue(cmdInfo, 4, false);
    }
  }
  else if (doScan)
  {
    BLEDevice::getScan()->start(0); // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }

  delay(1000); // Delay a second between loops.
  */
} // End of loop