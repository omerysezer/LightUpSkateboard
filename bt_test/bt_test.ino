#include "BluetoothSerial.h"
  
BluetoothSerial SerialBT;
String mode;
String btInput;
   
void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); // bluetooth device name
  //delay(1000);
  Serial.println("The device started, now you can pair it with bluetooth!");
}
   
void loop()
{

  //   if (Serial.available()) {
  //   SerialBT.write(Serial.read());
  // }
  // if (SerialBT.available()) {
  //   Serial.write(SerialBT.read());
  // }
  // delay(20);
  
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