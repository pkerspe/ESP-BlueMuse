#include <Arduino.h>
#include "ESP_BlueMuse.h"

ESP_BlueMuse *blueMuse;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  blueMuse = new ESP_BlueMuse();
  std::map<std::string, BLEAdvertisedDevice*> museHeadbands = blueMuse->discoverHeadbands();  

  Serial.printf("Found %i muse headbands\n", museHeadbands.size());
  if(museHeadbands.size() > 0){
    Serial.printf("connecting to muse headband %s\n", museHeadbands.begin()->second->getName().c_str());
    blueMuse->connectToMuseHeadband(museHeadbands.begin()->second->getName());
  } else {
    Serial.println("No muse headbands found");
  }
}

void loop()
{ /*
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