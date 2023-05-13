// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS 84
#define PIN_NEO_PIXEL 2
Adafruit_MPU6050 mpu;
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel2 = Adafruit_NeoPixel(1, 25, NEO_GRB + NEO_KHZ800);

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  pixel2.begin();
  pixel2.setBrightness(32);
  pixel2.setPixelColor(0, pixel2.Color(255, 255, 255));
  pixel2.show();

  pixel.begin();
  for(int i = 0; i < NUM_PIXELS; i++){
    pixel.setPixelColor(i, pixel.Color(0,0,0));
  }
  pixel.setBrightness(40);
  pixel.show();

  for(int i = 0; i < NUM_PIXELS; i++){
    pixel.setPixelColor(i, pixel.Color(0, 0, 0));
  }

  int r[7] = {255, 255, 255, 0, 0, 75, 148};
  int g[7] = {0, 127, 255, 255, 0, 0, 0};
  int b[7] = {0, 0, 0, 0, 255, 130, 211};

  for(int a = 0; a < 255; a+=20){
    Serial.println(a);
    pixel.setBrightness(a);
    for(int i = 0; i < NUM_PIXELS; i++){
      int x = i%7;
      pixel.setPixelColor(i, pixel.Color(r[x], g[x], b[x]));
    }
    pixel.show();
    delay(5000);
  }
}


void loop() {
  // for(int i = 0; i < NUM_PIXELS; i++){
  //   pixel.setPixelColor(i, pixel.Color(0, 0, 0));
  // }

  // int r[7] = {255, 255, 255, 0, 0, 75, 148};
  // int g[7] = {0, 127, 255, 255, 0, 0, 0};
  // int b[7] = {0, 0, 0, 0, 255, 130, 211};
  // for(int i = 0; i < NUM_PIXELS; i++){
  //   int x = i%7;
  //   pixel.setPixelColor(i, pixel.Color(r[x], g[x], b[x]));
  //   pixel.show();
  // }
  return;
}