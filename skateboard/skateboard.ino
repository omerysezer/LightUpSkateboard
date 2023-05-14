// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "BluetoothSerial.h"

#define NUM_PIXELS 85
#define PIN_NEO_PIXEL 2
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel2 = Adafruit_NeoPixel(16, 25, NEO_GRB + NEO_KHZ800);

Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;
int mode = -1;
String btInput;

// DEFINE ALL GLOBALS HERE
#define RainbowMode 0
#define DarkSparkleMode 1
#define LaserMode 2
#define HeadlightMode 3
#define ONOFF 4
#define INDICATOR_ON_OFF 5
#define MODE6 6
#define MODE7 7
#define MODE8 8
bool IndicatorOn = false;
bool OFF = false;

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

uint8_t rainbow_colors[7][3] = {{255, 0, 0}, {255, 100, 0}, {255, 255, 0}, {0, 205, 0}, {0, 0, 255}, {0, 255, 166}, {200, 0, 200}};
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
  if(p < 0){
    p = (p + NUM_PIXELS) % NUM_PIXELS;
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
    uint8_t* rgb = rainbow_colors[(i + rainbow_shift)% 7];
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

int stripe_offset = 0;
int num_stripes = (NUM_PIXELS / 17);
void stripe(){
  for(int i = 0; i < num_stripes; i++){
    for(int j = 0; j < 10; j++){
      set_pixel((j + stripe_offset + i * 17), 0, 0, 0);
    }
    for(int j = 10; j < 17; j++){
      uint8_t* rgb = rainbow_colors[j - 10];
      set_pixel((j + stripe_offset + i * 17), rgb[0], rgb[1], rgb[2]);
    }
  }

  delay(50);
  stripe_offset = (stripe_offset + 1) % NUM_PIXELS;
}

int single_stripe_offset = 0;
void single_stripe(){
  for(int i = 0; i < 7; i++){
    uint8_t* rgb = rainbow_colors[i];
    set_pixel((i + single_stripe_offset + 1) % NUM_PIXELS, rgb[0], rgb[1], rgb[2]);
  }
  set_pixel(single_stripe_offset, 0, 0, 0);
  single_stripe_offset = (single_stripe_offset + 1) % NUM_PIXELS;
  delay(100);
}

void headlights(){
  reset();
  for(int i = 0; i < 8; i++){
    set_pixel(i, 255, 255, 255);
  }
  for(int i = NUM_PIXELS - 8; i < NUM_PIXELS; i++){
    set_pixel(i, 255, 255, 255);
  }
  for(int i = NUM_PIXELS/2 - 8; i < NUM_PIXELS/2 + 8; i++){
    set_pixel(i, 255, 0, 0);
  }
}

int offset = 0;
int num_pixels_per_color = NUM_PIXELS / 7;
void pin_wheel(){
  for(int i = 0; i < 7; i++){
    for(int j = 0; j < 7; j++){
      uint8_t* rgb = rainbow_colors[i];
      set_pixel((i + offset + i*num_pixels_per_color) % NUM_PIXELS, rgb[0], rgb[1], rgb[2]);
    }
  }
  offset = (offset + 1) % NUM_PIXELS;
  delay(100);
}

int color = 0, color_wheel_pos = 0;
void color_wheel(){
  uint8_t* rgb = rainbow_colors[color];
  set_pixel(color_wheel_pos, rgb[0], rgb[1], rgb[2]);

  color_wheel_pos += 1;
  if(color_wheel_pos == NUM_PIXELS){
    color_wheel_pos = 0;
    color = (color + 1) % 7;
  }

  delay(25);
}

uint8_t fade_colors[NUM_PIXELS][3];
void fade(){
  for(int i = 0; i < NUM_PIXELS; i++){
    uint8_t* fade_rgb = fade_colors[i];
    uint8_t f_r = fade_rgb[0];
    uint8_t f_g = fade_rgb[1];
    uint8_t f_b = fade_rgb[2];

    uint8_t* rgb = prev_rgb_vals[i];
    uint8_t r = rgb[0];
    uint8_t g = rgb[1];
    uint8_t b = rgb[2];
      
    // if fade is done, choose new color
    // does while loop due to off chance of getting the same color again
    while(r == f_r && g == f_g && b == f_b){
      f_r = (uint8_t) random(0, 256);
      f_g = (uint8_t) random(0, 256);
      f_b = (uint8_t) random(0, 256);


      fade_colors[i][0] = f_r;
      fade_colors[i][1] = f_g;
      fade_colors[i][2] = f_b;
    }

    if(r < f_r){
      r++;
    }else if(r > f_r){
      r--;
    }

    if(g < f_g){
      g++;
    }else if(g > f_g){
      g--;
    }

    if(b < f_b){
      b++;
    }else if(b > f_b){
      b--;
    }

    set_pixel(i, r, g, b);
  }
  delay(10);
}

uint8_t black_fade_colors[NUM_PIXELS][3];
void black_fade(){
for(int i = 0; i < NUM_PIXELS; i++){
    uint8_t* fade_rgb = fade_colors[i];
    uint8_t f_r = fade_rgb[0];
    uint8_t f_g = fade_rgb[1];
    uint8_t f_b = fade_rgb[2];

    uint8_t* rgb = prev_rgb_vals[i];
    uint8_t r = rgb[0];
    uint8_t g = rgb[1];
    uint8_t b = rgb[2];
      
    // if fade is done, choose new color
    // does while loop due to off chance of getting the same color again
    while(r == f_r && g == f_g && b == f_b){
      if(f_r != 0 || f_g != 0 || f_b != 0){
        fade_colors[i][0] = 0;
        fade_colors[i][1] = 0;
        fade_colors[i][2] = 0;
        f_r = 0;
        f_g = 0;
        f_b = 0;
      }
      else{
        f_r = (uint8_t) random(0, 256);
        f_g = (uint8_t) random(0, 256);
        f_b = (uint8_t) random(0, 256);
        fade_colors[i][0] = f_r;
        fade_colors[i][1] = f_g;
        fade_colors[i][2] = f_b;
      }      
    }

    if(r < f_r){
      r++;
    }else if(r > f_r){
      r--;
    }

    if(g < f_g){
      g++;
    }else if(g > f_g){
      g--;
    }

    if(b < f_b){
      b++;
    }else if(b > f_b){
      b--;
    }

    set_pixel(i, r, g, b);
  }
  delay(10);
}

int rainbow_fade_color = 0;
uint8_t rainbow_fade_rgb[3] = {rainbow_colors[0][0], rainbow_colors[0][1], rainbow_colors[0][2]};
void rainbow_fade(){
for(int i = 0; i < NUM_PIXELS; i++){
    uint8_t f_r = rainbow_fade_rgb[0];
    uint8_t f_g = rainbow_fade_rgb[1];
    uint8_t f_b = rainbow_fade_rgb[2];

    uint8_t* rgb = prev_rgb_vals[i];
    uint8_t r = rgb[0];
    uint8_t g = rgb[1];
    uint8_t b = rgb[2];
      
    // if fade is done, choose new color
    // does while loop due to off chance of getting the same color again
    if(r == f_r && g == f_g && b == f_b){
      if(f_r != 0 || f_g != 0 || f_b != 0){
        rainbow_fade_rgb[0] = 0;
        rainbow_fade_rgb[1] = 0;
        rainbow_fade_rgb[2] = 0;
      }
      else{
        rainbow_fade_color = (rainbow_fade_color + 1) % NUM_PIXELS;
        rainbow_fade_rgb[0] = rainbow_colors[rainbow_fade_color][0];
        rainbow_fade_rgb[1] = rainbow_colors[rainbow_fade_color][1];
        rainbow_fade_rgb[2] = rainbow_colors[rainbow_fade_color][2];
      }      
      uint8_t f_r = rainbow_fade_rgb[0];
      uint8_t f_g = rainbow_fade_rgb[1];
      uint8_t f_b = rainbow_fade_rgb[2];
    }

    if(r < f_r){
      r++;
    }else if(r > f_r){
      r--;
    }

    if(g < f_g){
      g++;
    }else if(g > f_g){
      g--;
    }

    if(b < f_b){
      b++;
    }else if(b > f_b){
      b--;
    }

    set_pixel(i, r, g, b);
  }
  delay(10);
}

int left_streak_pos = -1, right_streak_pos = -1;
unsigned long left_streak_spawn_time = 0, right_streak_spawn_time = 0;
unsigned long left_streak_spawn_timeout = 0, right_streak_spawn_timeout = 0;
unsigned long fire_animation_timeout = 0;
void fire(){
  if(millis() - fire_animation_timeout > 100){
    fire_animation_timeout = millis();
  
    for(int i = -8; i <= 8; i++){
      set_pixel(i, 255, 0, 0);
    }
    for(int i = NUM_PIXELS/2 - 8; i < NUM_PIXELS/2 + 8; i++){
      set_pixel(i, 255, 255, 0);
    }
    for(int i = 9; i < 23; i++){
      set_pixel(i, 255, (i - 9) * 10, 0);
    }
    for(int i = -9; i > -24; i--){
      set_pixel(i, 255, -1 * (i + 9) * 10, 0);
    }
    for(int i = 34; i >= 23; i--){
      set_pixel(i, 255, 255 + (i - 34)*10, 0);
    }
    for(int i = 34; i >= 23; i--){
      set_pixel(i, 255, 255 + (i - 34)*10, 0);
    }
    for(int i = NUM_PIXELS/2 + 8; i <= NUM_PIXELS/2 + 20; i++){
      set_pixel(i, 255, 255 - (i - NUM_PIXELS/2 - 8)*10, 0);
    }

    if(left_streak_pos == -1 && millis() - left_streak_spawn_time > left_streak_spawn_timeout){
      left_streak_pos = random(0, 7);
      left_streak_spawn_time = millis();
      left_streak_spawn_timeout = random(1500, 2500);
    }
    if(right_streak_pos == -1 && millis() - right_streak_spawn_time > right_streak_spawn_timeout){
      right_streak_pos = random(NUM_PIXELS - 8, NUM_PIXELS - 1);
      right_streak_spawn_time = millis();
      right_streak_spawn_timeout = random(1500, 2500);
    }

    if(left_streak_pos != -1){
      for(int i = 0; i < 4; i++){
        set_pixel(left_streak_pos + i, 255, 255, 0);
      }
      left_streak_pos++;
      if(left_streak_pos > NUM_PIXELS/2 - 8){
        left_streak_pos = -1;
      }
    }
    if(right_streak_pos != -1){
      for(int i = 0; i < 4; i++){
        set_pixel(right_streak_pos - i, 255, 255, 0);
      }
      right_streak_pos--;
      if(right_streak_pos < NUM_PIXELS/2 - 8){
        right_streak_pos = -1;
      }
    }
  }
}

void four_corners(){
  set_pixel(0, 0, 0, 0);

  uint8_t r = random(175, 256);
  uint8_t g = random(0, 150);
  uint8_t b = random(0, 150);

  for(int i = 1; i <= 21; i++){
    set_pixel(i, r, g, b);
  }
    
  r = random(0, 150);
  g = random(0, 150);
  b = random(175, 256);

  for(int i = 21; i <= 41; i++){
    set_pixel(i, r, g, b);
  }

 
  r = random(0, 150);
  g = random(175, 256);
  b = random(0, 150);
  for(int i = 42; i <= 63; i++){
    set_pixel(i, r, g, b);
  }

  r = random(100, 256);
  g = random(100, 256);
  b = random(100, 256);

  for(int i = 64; i < NUM_PIXELS; i++){
    set_pixel(i, r, g, b);
  }

  delay(3000);
}

int sun_offset = 0;
void sun_and_moon(){
  for(int i = 0; i < NUM_PIXELS/2; i++){
    set_pixel((i + sun_offset) % NUM_PIXELS, 135, 206, 235);
  }

  for(int i = -2; i < 3; i++){
    set_pixel((i + sun_offset + NUM_PIXELS/4) % NUM_PIXELS, 255, 255, 0);
  }
  
  for(int i = NUM_PIXELS/2; i < NUM_PIXELS; i++){
    set_pixel((i + sun_offset) % NUM_PIXELS, 0, 0, 0);
  }

  for(int i = -2; i < 3; i++){
    set_pixel((i + sun_offset + NUM_PIXELS - NUM_PIXELS/4) % NUM_PIXELS, 255, 255, 255);
  }
  
  sun_offset = (sun_offset + 1) % NUM_PIXELS;
  delay(50);
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

void pickMode(){
  switch (mode){
    case 0:
      rainbow();
    break;
    case 1:
      dark_sparkle();
    break;
  }
}

void setup(void) {
  Serial.begin(115200);
  SerialBT.begin("ESP32Skateboard");
  while (!Serial)
    delay(10);

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

  for(int i = 0; i < NUM_PIXELS; i++){
    fade_colors[i][0] = (uint8_t) random(0, 256);
    fade_colors[i][1] = (uint8_t) random(0, 256);
    fade_colors[i][2] = (uint8_t) random(0, 256);
    
    black_fade_colors[i][0] = (uint8_t) random(0, 256);
    black_fade_colors[i][1] = (uint8_t) random(0, 256);
    black_fade_colors[i][2] = (uint8_t) random(0, 256);
  }

  for(int i = 0; i < READING_ARR_SIZE; i++){
    get_acceleration();
    get_gyro();
  }
  
  for(int i = 0; i < NUM_PIXELS; i++){
    uint8_t* rgb = rainbow_colors[i % NUM_PIXELS];
    set_pixel(i, 255, 255, 255);
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
<<<<<<< HEAD
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

  // update_brightness();
  // dark_sparkle();
  // stripe();
  // dark_sparkle();
  // color_wheel();
  // rainbow_fade();
  // four_corners();
  // sun_and_moon();
  fire();
=======
  update_brightness();
  get_acceleration();
  double x = accel[0];
  double y = accel[1];
  double z = accel[2]; 

  if (SerialBT.available()) {
    btInput = SerialBT.readString();
    btInput.trim();

    if (btInput.toInt() != mode) {
      SerialBT.println(btInput.toInt());
      if(btInput.toInt() == INDICATOR_ON_OFF){
        IndicatorOn = !IndicatorOn;
      }
      else if(btInput.toInt() == ONOFF){
        OFF = !OFF;
      }
      else{
        reset();
        mode = btInput.toInt();
      }
    }
  }

  if(!OFF){
    reset(); 
    return;
  }
 
  if(IndicatorOn == true){
    if(y > 2){
      reset();
      left_indicate();
      reset();
    }
    else if(y < -2){
      reset();
      right_indicate();
      reset();
    }else{
      pickMode();
    }
  }
  else{
    pickMode();
  }

<<<<<<< HEAD
=======

 SerialBT.println(mode);
 Serial.println(mode);

//  if(mode == DarkSparkleMode){
//    dark_sparkle();
//    SerialBT.println("Sparkle");
//  }

//  if(mode == RainbowMode){
//    rainbow();
//    SerialBT.println("rainbow");
//  }


>>>>>>> 3e91ed092005d68217fa6fdbb5bb545cc4900123
>>>>>>> a28c9ea414060680505f050288394e2234b058d3
  pixel.show();
}