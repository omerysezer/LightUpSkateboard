// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: +- 2g");
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: +- 500 deg/s");

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: 21 hz");

  Serial.println("");
  delay(100);
}

int get_pixel_distance_from_left(pixelIndex){
  return abs(76 - pixelIndex);
}

int get_pixel_distance_from_right(pixelIndex){
  return abs(25 - pixelIndex);
}


// x accel = from pins to ITG/MPU label
// y accel = from vcc to int label
// z accel = from botton of board to top of board

// change in y accel means moving forward or back
// change in x accel means moving left or right
// change in z accel means moving up or down

// when stable there should be x ~= 0, y ~= 0, z ~= 9.8

// let led 0 be directly next to arduino on right side and led 103 be directly next to arduino on left side
// led 103/2 = 51 will be directly opposite of arduino on board length wise (if arduion is on nose 51 is on tail)
// leds 0 to 50 are on right side of board
// leds 51 to 102 are on left side of board
// left most led is 76
// right most node is 25
int leds[103]; 
double lastX = 0.0, lastY = 0.0, lastZ = 0.0; 
int maxR = 255, maxG = 255, maxB = 255;
int minR = 50, minG = 50, minB = 50;
int mode = 0;
void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  double newX = a.acceleration.x;
  double newY = a.acceleration.y;
  double newZ = a.acceleration.z;

  // if(abs(newX - lastX) > 9){
  //   Serial.print("Spike in X: ");
  //   Serial.println(newX - lastX);
  // }

  // if(abs(newY - lastY) > 9){
  //   Serial.print("Spike in Y: ");
  //   Serial.println(newY - lastY);
  // }

  // if(abs(newZ - lastZ) > 9){
  //   Serial.print("Z Acceleration: ");
  //   Serial.println(newZ - lastZ);
  // }

  Serial.print("X Acceleration: ");
  Serial.println(newX);
  Serial.print("Y Acceleration: ");
  Serial.println(newY);
  Serial.print("Z Acceleration: ");
  Serial.println(newZ);
  Serial.println("==============================");
  lastX = newX;
  lastY = newY;
  lastZ = newZ;
  
  delay(500);
}