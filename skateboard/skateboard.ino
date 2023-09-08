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
int mode = 0;
String btInput;

// DEFINE ALL GLOBALS HERE
#define RainbowMode 0
#define DarkSparkleMode 1
#define StripeMode 2
#define HeadlightMode 3
#define PinWheelMode 4
#define ColorWheelMode 5
#define FadeMode 6
#define BlackFadeMode 7
#define RainbowFadeMode 8
#define FireMode 9
#define FourCornersMode 10
#define SunAndMoonMode 11
#define ONOFF 12
#define INDICATOR_ON_OFF 13

bool IndicatorOn = false;
bool OFF = true;
bool indicatorRun = false;

double accel_readings[3][5];
double gyro_readings[3][5];
const char READING_ARR_SIZE = 5;
double accel[3];
double prev_accel[3] = {0.0, 0.0, 0.0};
double MAX_ACCEL = 2;
double gyro[3];
char accel_idx = 0, gyro_idx = 0;

uint8_t curr_brightness = 50;
uint8_t BASELINE_BRIGHTNESS = 50;
uint8_t prev_rgb_vals[NUM_PIXELS][3];
int brightness_arr[5];
int brightness_arr_pos = 0;
int BRIGHTNESS_ARR_SIZE = 5;

unsigned long last_read_time = 0;
double velocity = 0.0;
double speed = 0;

uint8_t rainbow_colors[7][3] = {{255, 0, 0}, {255, 100, 0}, {255, 255, 0}, {0, 205, 0}, {0, 0, 255}, {0, 255, 166}, {200, 0, 200}};
int rainbow_shift = 0;


double MAX_SPEED = 6.0;
double speed_accel_offset = -0.52;
void reset(){
  for(int i = 0; i < NUM_PIXELS; i++){
    set_pixel(i, 0, 0, 0);
  }
  pixel.show();
}

double get_double_average(double* arr, int k){
  double sum = 0;
  for(int i = 0; i < k; i++){
    sum += arr[i];
  }

  return sum/k;
}

double get_int_average(int* arr, int k){
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
    prev_accel[i] = accel[i];
    accel[i] = get_double_average(accel_readings[i], READING_ARR_SIZE);
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
    gyro[i] = get_double_average(gyro_readings[i], READING_ARR_SIZE);
  }

  return gyro;
}

void calculate_velocity(){
  get_acceleration();

  unsigned long curr_time = millis();
  double d_t = ((double) (curr_time - last_read_time)/1000);
  last_read_time = curr_time;

  double x = accel[0];
  double prev_x = prev_accel[0];
  // orientation of arduino within box makes it have a constant acceleration of
  // .65 on top of movement caused acceleration
  x += speed_accel_offset;
  prev_x += speed_accel_offset;

  if(-0.1 <= x && x <= 0.1){
    return;
  }

  velocity += x * d_t;

}

double get_speed(){
  calculate_velocity();
  if(velocity < 0){
    return -velocity;
  }
  return velocity;
}

double get_speed_percentage(){
  double speed = get_speed();
  if(speed > MAX_SPEED){
    return 1.0;
  }
  if(speed <= 0){
    return 0.0;
  }

  return speed / MAX_SPEED;
}

uint8_t update_brightness(){
  double* accel = get_acceleration();
  double* gyro = get_gyro();
  double speed = get_speed();

  double a_mag = sqrt(accel[0] * accel[0] + accel[1] * accel[1]);
  uint8_t brightness = (uint8_t) ((a_mag / MAX_ACCEL) * (255 -  BASELINE_BRIGHTNESS)) + BASELINE_BRIGHTNESS;

  if(brightness > 255){
    brightness = 255;
  }

  brightness_arr[brightness_arr_pos] = brightness;
  brightness_arr_pos = (brightness_arr_pos + 1) % BRIGHTNESS_ARR_SIZE;
  
  curr_brightness = (uint8_t) get_int_average(brightness_arr, BRIGHTNESS_ARR_SIZE);
  for(int i = 0; i < NUM_PIXELS; i++){
    uint8_t* rgb = prev_rgb_vals[i];

    set_pixel(i, rgb[0], rgb[1], rgb[2]);
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

unsigned long prev_rainbow_timeout = 0;
unsigned long rainbow_timeout = 250;
void rainbow(){
  if(millis() <= prev_rainbow_timeout + (1.0/(1 + 9*get_speed_percentage()))*(rainbow_timeout)){
    return;
  }
  prev_rainbow_timeout = millis();

  for(int i = 0; i < NUM_PIXELS; i++){
    uint8_t* rgb = rainbow_colors[(i + rainbow_shift)% 7];
    set_pixel(i, rgb[0], rgb[1], rgb[2]);
  }
  rainbow_shift = (rainbow_shift + 1)%7;
  pixel.show();
}

int q_idx = 0;
int q_size = NUM_PIXELS/2;
int q[NUM_PIXELS/2];
unsigned long prev_dark_sparkle_timeout = 0;
unsigned long dark_sparkle_timeout = 250;
void dark_sparkle(){
  if(millis() <= prev_dark_sparkle_timeout + (1.0/(1 + 9*get_speed_percentage())) * dark_sparkle_timeout){
    return;
  }
  prev_dark_sparkle_timeout = millis();

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
}

int stripe_offset = 0;
int num_stripes = (NUM_PIXELS / 17);
unsigned long prev_stripe_timeout = 0;
unsigned long stripe_timeout = 75;
void stripe(){
  if(millis() <= prev_stripe_timeout + (1.0/(1 + 9*get_speed_percentage())) * stripe_timeout){
    return;
  }
  prev_stripe_timeout = millis();

  for(int i = 0; i < num_stripes; i++){
    for(int j = 0; j < 10; j++){
      set_pixel((j + stripe_offset + i * 17) % NUM_PIXELS, 0, 0, 0);
    }
    for(int j = 10; j < 17; j++){
      uint8_t* rgb = rainbow_colors[j - 10];
      set_pixel((j + stripe_offset + i * 17) % NUM_PIXELS, rgb[0], rgb[1], rgb[2]);
    }
  }
  stripe_offset = (stripe_offset + 1) % NUM_PIXELS;
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
unsigned long prev_pin_wheel_timeout = 0;
unsigned long pin_wheel_timeout = 100;
void pin_wheel(){
  if(millis() <= prev_pin_wheel_timeout + (1.0/(1 + 9*get_speed_percentage())) * pin_wheel_timeout){
    return;
  }
  prev_pin_wheel_timeout = millis();

  for(int i = 0; i < 7; i++){
    for(int j = 0; j < 7; j++){
      uint8_t* rgb = rainbow_colors[i];
      set_pixel((i + offset + i*num_pixels_per_color) % NUM_PIXELS, rgb[0], rgb[1], rgb[2]);
    }
  }
  offset = (offset + 1) % NUM_PIXELS;
}

int color = 0, color_wheel_pos = 0;
unsigned long prev_color_wheel_timeout = 0;
unsigned long color_wheel_timeout = 25;
void color_wheel(){
  if(millis() <= prev_color_wheel_timeout + (1.0 / (1 + 9*get_speed_percentage()))*color_wheel_timeout){
    return;
  }
  prev_color_wheel_timeout = millis();

  uint8_t* rgb = rainbow_colors[color];
  set_pixel(color_wheel_pos, rgb[0], rgb[1], rgb[2]);

  color_wheel_pos += 1;
  if(color_wheel_pos == NUM_PIXELS){
    color_wheel_pos = 0;
    color = (color + 1) % 7;
  }

}

uint8_t fade_colors[NUM_PIXELS][3];
unsigned long prev_fade_timeout = 0;
unsigned long fade_timeout = 10;
void fade(){
  if(millis() <= prev_fade_timeout + (1.0/(1 + 19*get_speed_percentage()))*fade_timeout){
    return;
  }
  prev_fade_timeout = millis();

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
}

uint8_t black_fade_colors[NUM_PIXELS][3];
unsigned long prev_black_fade_timeout = 0;
unsigned long black_fade_timeout = 10;
void black_fade(){
  if(millis() <= prev_black_fade_timeout + (1.0/(1 + 19*get_speed_percentage()))*black_fade_timeout){
    return;
  }
  prev_black_fade_timeout = millis();

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
}

int rainbow_fade_color = 0;
uint8_t rainbow_fade_rgb[3] = {rainbow_colors[0][0], rainbow_colors[0][1], rainbow_colors[0][2]};
unsigned long prev_rainbow_fade_timeout = 0;
unsigned long rainbow_fade_timeout = 10;
void rainbow_fade(){
  if(millis() <= prev_rainbow_fade_timeout + (1.0/(1 + 19*get_speed_percentage())) * rainbow_fade_timeout){
    return;
  }
  prev_rainbow_fade_timeout = millis();

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
}

int left_streak_pos = -1, right_streak_pos = -1;
unsigned long left_streak_spawn_time = 0, right_streak_spawn_time = 0;
unsigned long left_streak_spawn_timeout = 0, right_streak_spawn_timeout = 0;
unsigned long fire_animation_timeout = 0;
void fire(){
  if(millis() - fire_animation_timeout > (1.0/(1 + 9*get_speed_percentage())) * 100){
    fire_animation_timeout = millis();
  
    // set front to red
    for(int i = 76; i < NUM_PIXELS; i++){
      set_pixel(i, 255, 0, 0);
    }
    for(int i = 0; i <= 8; i++){
      set_pixel(i, 255, 0, 0);
    }

    // set back to yellow
    for(int i = NUM_PIXELS/2 - 8; i < NUM_PIXELS/2 + 8; i++){
      set_pixel(i, 255, 255, 0);
    }


    for(int i = 9; i < 23; i++){
      set_pixel(i, 255, (i - 9) * 10, 0);
    }
    for(int i = 75; i > 60; i--){
      set_pixel(i, 255, -1 * (i - 75) * 10, 0);
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

    if(left_streak_pos == -1 && millis() - left_streak_spawn_time > (1.0/(1 + 9*get_speed_percentage())) * left_streak_spawn_timeout){
      left_streak_pos = random(0, 7);
      left_streak_spawn_time = millis();
      left_streak_spawn_timeout = random(1500, 2500);
    }
    if(right_streak_pos == -1 && millis() - right_streak_spawn_time > (1.0/(1 + 9*get_speed_percentage())) * right_streak_spawn_timeout){
      right_streak_pos = random(77, 84);
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

unsigned long prev_four_corners_timeout = 0;
unsigned long four_corners_timeout = 1500;
void four_corners(){
  if(millis() <= prev_four_corners_timeout + (1.0/(1 + 9*get_speed_percentage())) * four_corners_timeout){
    return;
  }
  prev_four_corners_timeout = millis();

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

}

int sun_offset = 0;
unsigned long prev_sun_and_moon_timeout = 0;
unsigned long sun_and_moon_timeout = 50;
void sun_and_moon(){
  if(millis() <= prev_sun_and_moon_timeout + (1.0/(1 + 9*get_speed_percentage())) * sun_and_moon_timeout){
    return;
  }
  prev_sun_and_moon_timeout = millis();

  for(int i = 0; i < NUM_PIXELS/2; i++){
    set_pixel((i + sun_offset) % NUM_PIXELS, 0, 89, 255);
  }

  for(int i = -3; i < 4; i++){
    set_pixel((i + sun_offset + NUM_PIXELS/4) % NUM_PIXELS, 255, 255, 0);
  }
  
  for(int i = NUM_PIXELS/2; i < NUM_PIXELS; i++){
    set_pixel((i + sun_offset) % NUM_PIXELS, 0, 0, 0);
  }

  for(int i = -1; i < 2; i++){
    set_pixel((i + sun_offset + NUM_PIXELS - NUM_PIXELS/4) % NUM_PIXELS, 255, 255, 255);
  }
  
  sun_offset = (sun_offset + 1) % NUM_PIXELS;
}

unsigned long prev_right_indicate_timeout = 0;
unsigned long right_indicate_timeout = 10;
int rightpixNum = 0;
void right_indicate(){
  if(millis() <= prev_right_indicate_timeout + right_indicate_timeout){
    return;
  }
  prev_right_indicate_timeout = millis();
  indicatorRun = true;

  for(int i =0; i<rightpixNum; i++){
    set_pixel(i, 240, 166, 7);
  }
  
  rightpixNum = rightpixNum+1;
  if(rightpixNum == NUM_PIXELS/2){
    rightpixNum = 0; 
    reset();
  }
  pixel.show();

  
}

unsigned long prev_left_indicate_timeout = 0;
unsigned long left_indicate_timeout = 10;
int leftpixNum = NUM_PIXELS-1;
void left_indicate(){
  if(millis() <= prev_left_indicate_timeout + left_indicate_timeout){
    return;
  }
  prev_left_indicate_timeout = millis();
  indicatorRun = true;

  for(int i =NUM_PIXELS-1; i>=leftpixNum; i--){
    set_pixel(i, 240, 166, 7);
  }

  leftpixNum = leftpixNum -1;
    if(leftpixNum == NUM_PIXELS/2-1){
      leftpixNum = NUM_PIXELS-1;
      reset();
    }
    pixel.show();

  
}

void pickMode(){
  switch (mode){
    case RainbowMode:
      rainbow();
    break;
    case DarkSparkleMode:
      dark_sparkle();
    break;
    case StripeMode:
      stripe();
    break;
    case HeadlightMode:
      headlights();
    break;
    case PinWheelMode:
      pin_wheel();
    break;
    case ColorWheelMode:
      color_wheel();
    break;
    case FadeMode:
      fade();
    break;
    case BlackFadeMode:
      black_fade();
    break;
    case RainbowFadeMode:
      rainbow_fade();
    break;
    case FireMode:
      fire();
    break;
    case FourCornersMode:
      four_corners();
    break;
    case SunAndMoonMode:
      sun_and_moon();
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
  
  for(int i = 0; i < 3; i++){
    for(int i = 0; i < NUM_PIXELS; i++){
      uint8_t* rgb = rainbow_colors[i % NUM_PIXELS];
      set_pixel(i, 127, 255, 0);
    }
    delay(250);
    pixel.show();
    delay(250);
    reset();
  }
}


// x accel = from pins to ITG/MPU label
// y accel = from vcc to int label
// z accel = from botton of board to top of board

// change in y accel means moving forward or back
// change in x accel means moving left or right
// change in z accel means moving up or down

// when stable there should be x ~= 0, y ~= 0, z ~= 9.8
int print_timeout = 0;
bool auto_brightness_on = true;
int print_timeout_setting = 1000;
double indicator_threshold = .5;
void loop() { 
  if(auto_brightness_on){
    update_brightness();
  }else{
    curr_brightness = 50;
  }
  get_gyro();
  double length_aligned_gyroscope = gyro[2]; 

  if (SerialBT.available()) {
    btInput = SerialBT.readString();
    btInput.trim();
    if(btInput.startsWith("ACCEL: ")){
      btInput.replace("ACCEL: ", "");
      MAX_ACCEL = btInput.toDouble();
      SerialBT.print("Max Accel Set To: ");
      SerialBT.println(MAX_ACCEL);
      return;
    }
    if(btInput.startsWith("SPEED: ")){
      btInput.replace("SPEED: ", "");
      MAX_SPEED = btInput.toDouble();
      SerialBT.print("Max Speed Set To: ");
      SerialBT.println(MAX_SPEED);
      return;
    }
    if(btInput.startsWith("AUTOBRIGHT")){
      auto_brightness_on = !auto_brightness_on;
      return; 
    }
    if(btInput.startsWith("PRINTRATE: ")){
      btInput.replace("PRINTRATE: ", "");
      print_timeout_setting = btInput.toInt();
      return;
    }
    if(btInput.startsWith("OFFSET: ")){
      btInput.replace("OFFSET: ", "");
      speed_accel_offset = btInput.toDouble();
      return;
    }
    if(btInput.startsWith("INDICATOR: ")){
      btInput.replace("INDICATOR: ", "");
      indicator_threshold = btInput.toDouble();
      return;
    }
    if(btInput.startsWith("BASELINE: ")){
      BASELINE_BRIGHTNESS = (uint8_t) btInput.toInt();      
    }
    
    if (btInput.toInt() != mode) {
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

  if(OFF){
    reset(); 
    velocity = 0.0;
    MAX_SPEED = 6.0;
    MAX_ACCEL = 2;
    print_timeout_setting = 1000;
    auto_brightness_on = true;
    speed_accel_offset = -.52;
    indicator_threshold = .5;
    BASELINE_BRIGHTNESS = 50;
    return;
  }

  get_gyro();
  if(millis() - print_timeout > print_timeout_setting){
    // SerialBT.print("CURR SPEED: ");
    // SerialBT.println(get_speed());
    // SerialBT.print("CURR VELOCITY");
    // SerialBT.println(velocity);
    // SerialBT.print("Curr Speed Percentage: ");
    // SerialBT.println(get_speed_percentage());
    // SerialBT.print("Curr Accel W/ Offset: ");
    // SerialBT.println(accel[0] + speed_accel_offset);
    // SerialBT.print("Curr Offset: ");
    // SerialBT.println(speed_accel_offset);
    SerialBT.print("X Gyro");
    SerialBT.println(gyro[0]);
    SerialBT.print("Y Gyro");
    SerialBT.println(gyro[1]);
    SerialBT.print("Z Gyro");
    SerialBT.println(gyro[2]);
    
    // SerialBT.println("Curr Accel: ");
    // SerialBT.print("x: ");
    // SerialBT.println(accel[0]);
    // SerialBT.print("y: ");
    // SerialBT.println(accel[1]);
    // SerialBT.print("z: ");
    // SerialBT.println(accel[2]);
    // SerialBT.print("Brightness: ");
    // SerialBT.println(curr_brightness);
    // SerialBT.print("MAX ACCEL: ");
    // SerialBT.println(MAX_ACCEL);
    // SerialBT.print("BASELINE BRIGHTNESS: ");
    // SerialBT.println(BASELINE_BRIGHTNESS);
    
    SerialBT.println("-------------------------");
    print_timeout = millis();
  }

  if(IndicatorOn == true){
    if(length_aligned_gyroscope < -1 * indicator_threshold){
      if(indicatorRun == false){
        reset();
      }
      left_indicate();
    }
    else if(length_aligned_gyroscope > indicator_threshold){
      if(indicatorRun == false){
        reset();
      }
      right_indicate();
    }else{
      if(indicatorRun == true){
        rightpixNum = 0;
        leftpixNum = NUM_PIXELS-1;
        reset();
        indicatorRun = false;
      }
      pickMode();
    }
  }
  else{
    pickMode();
  }

  pixel.show();
}