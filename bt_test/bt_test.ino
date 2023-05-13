#include "BluetoothSerial.h"
  
BluetoothSerial SerialBT;
String mode;
String btInput;
   
void setup()
{
  SerialBT.begin("ESP32test"); // bluetooth device name
  delay(1000);
  Serial.begin(9600);
}
   
void loop()
{
  
  // read input
  if (SerialBT.available()) {
    btInput = SerialBT.readString();
    if (btInput != mode) {
      mode = btInput;
    }
    SerialBT.println("Selected mode: ");
    SerialBT.println(mode);
    Serial.println(btInput);
  }
}