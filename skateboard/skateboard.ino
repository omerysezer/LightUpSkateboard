// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define NUM_PIXELS 85
#define PIN_NEO_PIXEL 2
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel2 = Adafruit_NeoPixel(1, 25, NEO_GRB + NEO_KHZ800);

Adafruit_MPU6050 mpu;

// DEFINE ALL GLOBALS HERE
#define MODE0 0
#define MODE1 1
#define MODE2 2
#define MODE3 3
#define MODE4 4
#define MODE5 5
#define MODE6 6
#define MODE7 7
#define INDICATOR_ON 8

double accel_readings[3][5];
double gyro_readings[3][5];
const char READING_ARR_SIZE = 5;
double accel[3];
double gyro[3];
char accel_idx = 0, gyro_idx = 0;

uint8_t curr_brightness = 30;
uint8_t prev_rgb_vals[NUM_PIXELS][3];

int NOSE_PIXEL = 0;
int TAIL_PIXEL = 100;

unsigned long last_read_time = 0;
double velocity[] = {0.0, 0.0, 0.0};
double speed = 0;

int rainbow_colors[7][3] = {{255, 0, 0}, {255, 165, 0}, {255, 255, 0}, {0, 128, 0}, {0, 0, 255}, {75, 0, 130}, {148, 130, 211}};
int rainbow_shift = 0;

void reset(){
  for(int i = 0; i < NUM_PIXELS; i++){
    set_pixel(i, 0, 0, 0);
  }
  pixel.show();
}

double get_average(double* arr, int k){
  double sum = 0;
  for(int i = 0; i < k; i++){
    sum += arr[i];
  }

  return sum/k;
}

double* get_acceleration(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  double x = a.acceleration.x;
  double y = a.acceleration.y;
  double z = a.acceleration.z;

  accel_readings[0][accel_idx] = x;
  accel_readings[1][accel_idx] = y;
  accel_readings[2][accel_idx] = z;

  accel_idx = (accel_idx + 1) % READING_ARR_SIZE;

  for(int i = 0; i < 3; i++){
    accel[i] = get_average(accel_readings[i], READING_ARR_SIZE);
  }

  return accel;
}

double* get_gyro(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  double x = g.gyro.x;
  double y = g.gyro.y;
  double z = g.gyro.z;

  double new_readings[3] = {x, y, z};
  for(int i = 0; i < 3; i++){
    gyro_readings[i][gyro_idx] = new_readings[i];
  }

  gyro_idx = (gyro_idx + 1) % READING_ARR_SIZE;

  for(int i = 0; i < 3; i++){
    gyro[i] = get_average(gyro_readings[i], READING_ARR_SIZE);
  }

  return gyro;
}

void calculate_velocity(){
  get_acceleration();
  double x = accel[0], y = accel[1];

  unsigned long curr_time = millis();
  velocity[0] += (x * curr_time) - last_read_time;
  velocity[1] += (y * curr_time) - last_read_time;
}

double get_speed(){
  calculate_velocity();
  return sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
}

uint8_t update_brightness(){
  double* accel = get_acceleration();
  double* gyro = get_gyro();
  double speed = get_speed();

  uint8_t brightness = curr_brightness;

  double a_mag = sqrt(accel[0] * accel[0] + accel[1] * accel[1]);
  double max_a = .5 * 9.8;
  brightness = (uint8_t) ((a_mag / max_a) * 255);

  if(brightness > 255){
    brightness = 255;
  }
  else if(brightness < 10){
    brightness = 10;
  }

  if((double) brightness / curr_brightness > .75){
    curr_brightness = brightness;

    for(int i = 0; i < NUM_PIXELS; i++){
      uint8_t* rgb = prev_rgb_vals[i];

      set_pixel(i, rgb[0], rgb[1], rgb[2]);
    }  

  }
  return brightness;
}

int map_pixel(int p){
  if(p >= NUM_PIXELS){
    p = p % NUM_PIXELS;
  }
  if(p >= 0 && p <= 17){
    p = p + 67;
  }
  else if(p >= 18 && p <= 84){
    p = p - 18;
  }
  
  return p;
}

void set_pixel(int p_idx, uint8_t r, uint8_t g, uint8_t b){
  uint8_t red = r, green = g, blue = b;  

  prev_rgb_vals[p_idx][0] = r;
  prev_rgb_vals[p_idx][1] = g;
  prev_rgb_vals[p_idx][2] = b;

  r = (r * curr_brightness)/255;
  g = (g * curr_brightness)/255;
  b = (b * curr_brightness)/255;

  pixel.setPixelColor(map_pixel(p_idx), r, g, b);
}

void rainbow(){
  for(int i = 0; i < NUM_PIXELS; i++){
    int* rgb = rainbow_colors[(i + rainbow_shift)% 7];
    set_pixel(i, rgb[0], rgb[1], rgb[2]);
  }
  rainbow_shift = (rainbow_shift + 1)%7;
  pixel.show();
  delay(250);
}

int q_idx = 0;
int q_size = NUM_PIXELS/2;
int q[NUM_PIXELS/2];
void dark_sparkle(){
  int k = random(q_size/4, q_size / 2);  
  for(int i = 0; i < k; i++){
    int p = random(0, NUM_PIXELS);
    set_pixel(q[q_idx], 0, 0, 0);
    int r = random(0, 256);
    int g = random(0, 256);
    int b = random(0, 256);
    set_pixel(p, r, g, b);
    q[q_idx] = p;
    q_idx = (q_idx + 1) % q_size;

    if(i % 10 == 0){
      pixel.show();
    }  
  }
  delay(250);
}

void right_indicate(){
  for(int i = 0; i < NUM_PIXELS/2; i++){
    set_pixel(i, 240, 166, 7);
    pixel.show();
    delay(100);
  }

  reset();
}

void left_indicate(){
  for(int i = NUM_PIXELS; i >= NUM_PIXELS / 2; i--){
    set_pixel(i, 240, 166, 7);
    pixel.show();
    delay(100);
  }

  reset();
}

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

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  Serial.print("Accelerometer range set to: +- 2g");
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: +- 500 deg/s");

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: 21 hz");

  for(int i = 0; i < READING_ARR_SIZE; i++){
    get_acceleration();
    get_gyro();
  }
  
  for(int i = 0; i < NUM_PIXELS; i++){
    set_pixel(i, 240, 166, 7);
  }
  pixel.show();
}


// x accel = from pins to ITG/MPU label
// y accel = from vcc to int label
// z accel = from botton of board to top of board

// change in y accel means moving forward or back
// change in x accel means moving left or right
// change in z accel means moving up or down

// when stable there should be x ~= 0, y ~= 0, z ~= 9.8
void loop() { 
  // get_acceleration();
  // double x = accel[0];
  // double y = accel[1];
  // double z = accel[2]; 

  // if(y > 2){
  //   reset();
  //   left_indicate();
  //   reset();
  // }
  // else if(y < -2){
  //   reset();
  //   right_indicate();
  //   reset();
  // }else{
  //   rainbow();
  // }


  // rainbow();
  // delay(100);

  update_brightness();
  dark_sparkle();
  pixel.show();
  // delay(1000);
}